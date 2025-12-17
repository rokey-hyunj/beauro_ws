import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import firebase_admin
from firebase_admin import credentials, db
import time
import math
import threading

# 서비스 및 토픽 메시지 (제어용 MoveJoint, MoveLine 추가)
from dsr_msgs2.srv import GetRobotState, GetRobotMode, GetCurrentPosx, MoveJoint, MoveLine
from sensor_msgs.msg import JointState

# 상태 매핑
ROBOT_STATE_MAP = {
    0: "STATE_UNKNOWN", 1: "STATE_INITIALIZING", 2: "STATE_STANDBY",
    3: "STATE_MOVING", 4: "STATE_SAFE_OFF", 5: "STATE_TEACHING",
    6: "STATE_SAFE_STOP", 7: "STATE_EMERGENCY_STOP", 8: "STATE_HOMING", 9: "STATE_RECOVERY"
}

class BeauroFullSystem(Node):
    def __init__(self):
        super().__init__('beauro_full_system')
        self.get_logger().info("🚀 Beauro Full System Started (Monitor + Control)")

        # 1. Firebase 접속
        try:
            if not firebase_admin._apps:
                cred = credentials.Certificate("serviceAccountKey.json")
                firebase_admin.initialize_app(cred, {
                    'databaseURL': 'https://beauro-ac0ad-default-rtdb.asia-southeast1.firebasedatabase.app/' 
                })
            self.ref_status = db.reference('robot_state')
            self.ref_command = db.reference('manual_command')
            self.get_logger().info("✅ Firebase Connected")
        except Exception as e:
            self.get_logger().error(f"Firebase Error: {e}")

        # 2. 데이터 저장소
        self.robot_data = {
            'is_online': False, 'state_code': 0, 'status': "DISCONNECTED",
            'mode': 0, 'tcp': {'x':0,'y':0,'z':0,'a':0,'b':0,'c':0}, 
            'joint': [0]*6, 'timestamp': 0
        }
        self.last_alive_time = 0 

        # 3. [Monitor] 클라이언트 & 구독 설정
        qos_profile = QoSProfile(depth=10)
        self.create_subscription(JointState, '/dsr01/joint_states', self.topic_callback_joint, qos_profile)
        
        self.cli_state = self.create_client(GetRobotState, '/dsr01/system/get_robot_state')
        self.cli_mode = self.create_client(GetRobotMode, '/dsr01/system/get_robot_mode')
        self.cli_tcp = self.create_client(GetCurrentPosx, '/dsr01/aux_control/get_current_posx')

        # 4. [Control] 제어 서비스 클라이언트 추가
        self.cli_movej = self.create_client(MoveJoint, '/dsr01/motion/move_joint')
        self.cli_movel = self.create_client(MoveLine, '/dsr01/motion/move_line')

        # 5. [Control] Firebase 명령 리스너 시작 (별도 스레드)
        self.listener_thread = threading.Thread(target=self.start_command_listener)
        self.listener_thread.daemon = True
        self.listener_thread.start()

        # 6. [Monitor] 주기적 업데이트 타이머 (0.5초)
        self.timer = self.create_timer(0.5, self.routine_check)

    # ------------------------------------------------------------------
    # Part 1: Control Logic (수동 제어)
    # ------------------------------------------------------------------
    def start_command_listener(self):
        """Firebase 'manual_command' 경로 감시"""
        self.ref_command.listen(self.on_command_received)

    def on_command_received(self, event):
        """명령이 들어오면 실행"""
        if event.data is None: return # 데이터 삭제됨
        
        # 전체 데이터 가져오기 (이벤트 데이터가 부분적일 수 있어서 get() 사용)
        cmd = self.ref_command.get()
        if not cmd: return

        self.get_logger().info(f"📩 Command Received: {cmd.get('type')}")
        
        try:
            # 1. TCP 이동 명령 (MoveLine)
            if cmd['type'] == 'move_line':
                target = cmd['target']
                # {x,y,z,a,b,c} -> [x,y,z,a,b,c] 리스트 변환
                pos_list = [
                    float(target['x']), float(target['y']), float(target['z']),
                    float(target['a']), float(target['b']), float(target['c'])
                ]
                self.execute_move_line(pos_list)

            # 2. 관절 이동 명령 (MoveJoint)
            elif cmd['type'] == 'move_joint':
                # target: [j1, j2, j3, j4, j5, j6]
                pos_list = [float(x) for x in cmd['target']]
                self.execute_move_joint(pos_list)

            # 3. 명령 실행 후 삭제 (중복 방지)
            self.ref_command.delete()
            
        except Exception as e:
            self.get_logger().error(f"Command Execution Failed: {e}")

    def execute_move_line(self, pos):
        if not self.cli_movel.service_is_ready():
            self.get_logger().warn("⚠️ MoveLine service not ready")
            return
        
        req = MoveLine.Request()
        req.pos = pos
        req.vel = [100.0, 100.0] # 속도 (mm/s, deg/s) - 필요시 조정
        req.acc = [200.0, 200.0] # 가속도
        req.time = 0.0           # 시간 지정 안함
        req.radius = 0.0         # 블렌딩 반경 0
        req.ref = 0              # Base 기준
        req.mode = 0             # Absolute Mode
        req.blend_type = 0       
        req.sync_type = 0        # 0:Async (비동기)

        self.cli_movel.call_async(req)
        self.get_logger().info(f"🚀 Moving Linear to: {pos}")

    def execute_move_joint(self, pos):
        if not self.cli_movej.service_is_ready():
            self.get_logger().warn("⚠️ MoveJoint service not ready")
            return

        req = MoveJoint.Request()
        req.pos = pos
        req.vel = 30.0 # 속도 (deg/s) - 안전을 위해 낮게 설정
        req.acc = 30.0 # 가속도
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0   # Absolute Mode
        req.blend_type = 0
        req.sync_type = 0

        self.cli_movej.call_async(req)
        self.get_logger().info(f"🚀 Moving Joint to: {pos}")

    # ------------------------------------------------------------------
    # Part 2: Monitor Logic (상태 감시) - 기존 코드 유지
    # ------------------------------------------------------------------
    def routine_check(self):
        # Watchdog
        if time.time() - self.last_alive_time > 3.0:
            if self.robot_data['is_online']:
                self.ref_status.update({'is_online': False, 'status': "DISCONNECTED"})
                self.robot_data['is_online'] = False
        
        # Service Update Loop
        if self.cli_state.service_is_ready():
            self.cli_state.call_async(GetRobotState.Request()).add_done_callback(self.cb_state)

    # --- B. 토픽 처리 (Alive 신호 갱신 + 순서 정렬) ---
    # --- B. 토픽 처리 (Alive 신호 갱신 + 순서 정렬) ---
    def topic_callback_joint(self, msg):
        try:
            self.last_alive_time = time.time() # 생존 신호
            
            # [1] 데이터 매핑: (이름: 값) 딕셔너리 생성
            # 예: {'joint_1': 0.01, 'joint_4': 0.13, ...}
            joint_map = dict(zip(msg.name, msg.position))
            
            # [2] 강제 정렬: 우리가 원하는 순서대로 값을 뽑아냄 (J1 -> J6)
            # ★★★ 수정된 부분: 이름에 언더바(_) 추가 ★★★
            target_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            
            ordered_rads = []
            for name in target_names:
                # 딕셔너리에서 이름으로 값을 찾음 (없으면 0.0)
                val = joint_map.get(name, 0.0) 
                ordered_rads.append(val)

            # [3] 라디안 -> 도(Degree) 변환
            joints_deg = [round(math.degrees(rad), 3) for rad in ordered_rads]
            
            # 디버깅: 이제 순서가 맞는지 확인!
            # print(f"Sorted: {joints_deg}") 

            # Firebase 전송 (0.1초 제한)
            if time.time() - self.robot_data['timestamp'] / 1000 > 0.1:
                self.robot_data['joint'] = joints_deg
                self.robot_data['timestamp'] = int(time.time() * 1000)
                self.robot_data['is_online'] = True
                
                self.ref_status.update({
                    'joint': joints_deg,
                    'timestamp': self.robot_data['timestamp'],
                    'is_online': True
                })
        except Exception as e:
            # self.get_logger().warn(f"Joint map error: {e}")
            pass

    def cb_state(self, future):
        try:
            res = future.result()
            self.last_alive_time = time.time()
            self.robot_data['state_code'] = res.robot_state
            self.robot_data['status'] = ROBOT_STATE_MAP.get(res.robot_state, f"UNKNOWN({res.robot_state})")
            
            if self.cli_mode.service_is_ready():
                self.cli_mode.call_async(GetRobotMode.Request()).add_done_callback(self.cb_mode)
        except Exception: pass

    def cb_mode(self, future):
        try:
            res = future.result()
            self.robot_data['mode'] = res.robot_mode
            if self.cli_tcp.service_is_ready():
                req = GetCurrentPosx.Request()
                req.ref = 0
                self.cli_tcp.call_async(req).add_done_callback(self.cb_tcp)
        except Exception: pass

    def cb_tcp(self, future):
        try:
            res = future.result()
            if hasattr(res, 'task_pos_info'): raw = res.task_pos_info[0].data
            elif hasattr(res, 'posx'): raw = res.posx
            else: return

            self.robot_data['tcp'] = {
                'x': round(raw[0], 2), 'y': round(raw[1], 2), 'z': round(raw[2], 2),
                'rx': round(raw[3], 2), 'ry': round(raw[4], 2), 'rz': round(raw[5], 2)
            }
            
            self.ref_status.update({
                'state': self.robot_data['state_code'],
                'status': self.robot_data['status'],
                'mode': self.robot_data['mode'],
                'tcp': self.robot_data['tcp'],
                'is_online': True
            })
        except Exception: pass

def main(args=None):
    rclpy.init(args=args)
    node = BeauroFullSystem()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ref_status.update({'is_online': False, 'status': "DISCONNECTED"})
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()