# 🔧 BeauRo 트러블슈팅 가이드

이 문서는 BeauRo 시스템 운영 중 발생할 수 있는 문제들과 해결 방법을 정리합니다.

## 목차

- [빠른 진단 가이드](#빠른-진단-가이드)
- [시스템 시작 문제](#시스템-시작-문제)
- [로봇 제어 문제](#로봇-제어-문제)
- [네트워크 및 통신 문제](#네트워크-및-통신-문제)
- [Firebase 연동 문제](#firebase-연동-문제)
- [작업 실행 문제](#작업-실행-문제)
- [에러 복구 문제](#에러-복구-문제)
- [그리퍼 및 I/O 문제](#그리퍼-및-io-문제)
- [성능 및 정확도 문제](#성능-및-정확도-문제)
- [데이터 및 설정 문제](#데이터-및-설정-문제)
- [긴급 상황 대처](#긴급-상황-대처)

---

## 빠른 진단 가이드

### 증상별 빠른 찾기

| 증상 | 가능한 원인 | 섹션 |
|------|------------|------|
| 프로그램이 시작되지 않음 | Python 패키지, Firebase | [시스템 시작](#시스템-시작-문제) |
| 로봇이 움직이지 않음 | Servo-Off, 네트워크 | [로봇 제어](#로봇-제어-문제) |
| Firebase 연결 실패 | 인증 키, 인터넷 | [Firebase](#firebase-연동-문제) |
| 작업 중 멈춤 | 에러 상태, 충돌 | [작업 실행](#작업-실행-문제) |
| 좌표가 정확하지 않음 | 티칭 오류, TCP | [성능](#성능-및-정확도-문제) |
| 그리퍼가 동작하지 않음 | I/O 설정, 공압 | [그리퍼](#그리퍼-및-io-문제) |

### 기본 체크리스트

문제 발생 시 가장 먼저 확인:

- [ ] 로봇 전원이 켜져 있는가?
- [ ] 로봇이 Servo-On 상태인가?
- [ ] E-Stop이 해제되어 있는가?
- [ ] 네트워크 케이블이 연결되어 있는가?
- [ ] ROS2 환경이 소싱되어 있는가?
- [ ] 인터넷 연결이 정상인가?
- [ ] 작업 공간에 장애물이 없는가?

---

## 시스템 시작 문제

### 문제 1: `beauro.py` 실행 시 ModuleNotFoundError

**증상**:
```
ModuleNotFoundError: No module named 'firebase_admin'
```

**원인**:
- Python 패키지가 설치되지 않음
- 잘못된 Python 환경 사용

**해결 방법**:

```bash
# 1. 현재 Python 확인
which python3
python3 --version

# 2. pip 확인
which pip3

# 3. 패키지 재설치
pip3 install --user firebase-admin PyYAML

# 또는 시스템 전역 설치
sudo pip3 install firebase-admin PyYAML

# 4. 설치 확인
python3 -c "import firebase_admin; print('OK')"

# 5. 가상환경 사용 중이라면
source venv/bin/activate
pip install -r requirements.txt
```

---

### 문제 2: ROS2 관련 에러

**증상**:
```
ModuleNotFoundError: No module named 'rclpy'
```

**원인**:
- ROS2 환경이 소싱되지 않음

**해결 방법**:

```bash
# 1. ROS2 소싱
source /opt/ros/humble/setup.bash
source ~/doosan_ws/install/setup.bash

# 2. 자동 소싱 설정 확인
cat ~/.bashrc | grep ros

# 3. 없다면 추가
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/doosan_ws/install/setup.bash" >> ~/.bashrc

# 4. 새 터미널 열기 또는
source ~/.bashrc

# 5. 확인
ros2 --version
```

---

### 문제 3: DR_init 모듈 에러

**증상**:
```
ModuleNotFoundError: No module named 'DR_init'
```

**원인**:
- 두산 로보틱스 패키지가 빌드되지 않음
- Python 경로 문제

**해결 방법**:

```bash
# 1. 두산 패키지 확인
ros2 pkg list | grep dsr

# 패키지가 없다면:
cd ~/doosan_ws
colcon build --symlink-install
source install/setup.bash

# 2. Python 경로 확인
python3 -c "import sys; print('\n'.join(sys.path))"

# 3. PYTHONPATH 설정 (필요시)
export PYTHONPATH=$PYTHONPATH:~/doosan_ws/install/dsr_msgs2/lib/python3.10/site-packages
echo "export PYTHONPATH=\$PYTHONPATH:~/doosan_ws/install/dsr_msgs2/lib/python3.10/site-packages" >> ~/.bashrc
```

---

## 로봇 제어 문제

### 문제 4: 로봇이 명령에 반응하지 않음

**증상**:
- `beauro.py` 실행 후 "Waiting for Web Order..." 이후 아무 반응 없음
- 또는 "Robot initialization failed" 메시지

**원인**:
- 로봇이 Servo-Off 상태
- E-Stop 활성화
- 네트워크 연결 불량

**진단**:

```bash
# 1. 로봇 Ping 테스트
ping 192.168.137.100

# 성공:
# 64 bytes from 192.168.137.100: icmp_seq=1 ttl=64 time=0.5 ms

# 실패:
# From 192.168.137.10 icmp_seq=1 Destination Host Unreachable

# 2. ROS2 토픽 확인
ros2 topic list | grep dsr01

# 정상:
# /dsr01/state/current_pose
# /dsr01/state/robot_state
# ...

# 3. 로봇 상태 확인
ros2 topic echo /dsr01/state/robot_state --once
```

**해결 방법**:

```bash
# Case 1: 네트워크 문제
# - 케이블 재연결
# - 로봇 제어기 IP 확인
# - PC 네트워크 설정 재확인

# Case 2: E-Stop
# - 티칭 펜던트에서 E-Stop 해제
# - 제어기의 물리적 E-Stop 버튼 확인

# Case 3: Servo-Off
# - 펜던트에서 Servo-On 버튼 누름
# - 또는 자동 복구 시도:
python3 << EOF
from DSR_ROBOT2 import *
DR_init()
set_robot_control(CONTROL_SERVO_ON)
EOF
```

---

### 문제 5: 로봇이 에러 상태로 멈춤

**증상**:
- 작업 중간에 로봇이 정지
- "❌ Robot Error Detected" 메시지

**원인**:
- Safe Stop (상태 3)
- Protective Stop (상태 6)
- Servo-Off (상태 9)

**해결 방법**:

```bash
# 1. 현재 에러 상태 확인
ros2 topic echo /dsr01/state/robot_state --once

# 2. 자동 복구 시도
# BeauRo는 자동으로 복구를 시도하므로 웹에서 "Resume" 버튼 클릭

# 3. 수동 복구
python3 << EOF
from DSR_ROBOT2 import *
DR_init()

# Safe Stop 해제
set_safe_stop_reset_type(SAFE_STOP_RESET_TYPE_DEFAULT)
set_robot_control(CONTROL_RESET_SAFET_STOP)

# Servo-On
for i in range(3):
    result = set_robot_control(CONTROL_SERVO_ON)
    if result: break
    time.sleep(1)
EOF
```

---

## 네트워크 및 통신 문제

### 문제 6: 네트워크 연결 불가

**증상**:
```bash
ping 192.168.137.100
# Destination Host Unreachable
```

**원인**:
- IP 주소 충돌
- 서브넷 불일치
- 케이블 불량

**진단**:

```bash
# 1. 네트워크 인터페이스 확인
ip addr show

# 2. 라우팅 테이블 확인
ip route

# 3. 케이블 연결 확인
sudo ethtool enp3s0  # 인터페이스 이름 확인 후
```

**해결 방법**:

```bash
# 1. 네트워크 재시작
sudo systemctl restart NetworkManager

# 2. 고정 IP 재설정
sudo nano /etc/netplan/01-network-manager-all.yaml
```

```yaml
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    enp3s0:  # 실제 인터페이스 이름으로 변경
      dhcp4: no
      addresses:
        - 192.168.137.10/24
      routes:
        - to: default
          via: 192.168.137.1
```

```bash
# 3. 적용
sudo netplan apply

# 4. 확인
ping 192.168.137.100
```

---

### 문제 7: ROS2 통신 끊김

**증상**:
- 토픽은 보이지만 데이터가 오지 않음
- 서비스 호출 타임아웃

**원인**:
- DDS 설정 문제
- 방화벽 차단

**해결 방법**:

```bash
# 1. DDS 확인
echo $RMW_IMPLEMENTATION

# 2. FastDDS로 변경 (권장)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc

# 3. ROS_DOMAIN_ID 설정
export ROS_DOMAIN_ID=30
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc

# 4. 방화벽 확인
sudo ufw status

# 5. 필요시 포트 허용
sudo ufw allow from 192.168.137.0/24
```

---

## Firebase 연동 문제

### 문제 8: Firebase 초기화 실패

**증상**:
```
❌ Init Error: Could not find the Firebase credentials file
```

**원인**:
- `serviceAccountKey.json` 파일 없음
- 파일 경로 오류

**해결 방법**:

```bash
# 1. 파일 존재 확인
ls -l ~/beauro/serviceAccountKey.json

# 2. 없다면 Firebase Console에서 재다운로드
# 프로젝트 설정 → 서비스 계정 → 새 비공개 키 생성

# 3. 올바른 위치로 이동
cp ~/Downloads/beauro-xxxxx-firebase-adminsdk-xxxxx.json ~/beauro/serviceAccountKey.json

# 4. 권한 설정
chmod 600 ~/beauro/serviceAccountKey.json

# 5. beauro.py에서 경로 확인
grep "FIREBASE_KEY_PATH" beauro.py
```

---

### 문제 9: Database URL 오류

**증상**:
```
firebase_admin.exceptions.InvalidArgumentError: Invalid database URL
```

**원인**:
- Database URL이 잘못됨
- 프로젝트 ID 불일치

**해결 방법**:

```bash
# 1. Firebase Console에서 Database URL 확인
# Realtime Database → 데이터베이스 URL 복사

# 2. beauro.py 수정
nano beauro.py
```

```python
# 올바른 URL 형식 확인
FIREBASE_DB_URL = 'https://beauro-xxxxx.firebaseio.com/'
# 또는
FIREBASE_DB_URL = 'https://beauro-xxxxx.asia-southeast1.firebasedatabase.app/'
```

---

### 문제 10: Firebase 데이터 읽기/쓰기 실패

**증상**:
```
firebase_admin.exceptions.PermissionDeniedError: Permission denied
```

**원인**:
- Database 보안 규칙 제한
- 인증 문제

**해결 방법**:

```bash
# Firebase Console → Realtime Database → 규칙

# 테스트 모드 (개발용):
{
  "rules": {
    ".read": true,
    ".write": true
  }
}

# 운영 모드:
{
  "rules": {
    "current_order": {
      ".read": true,
      ".write": true
    },
    "robot_state": {
      ".read": true,
      ".write": true
    },
    "task_checkpoint": {
      ".read": true,
      ".write": true
    }
  }
}

# 규칙 게시 버튼 클릭
```

---

## 작업 실행 문제

### 문제 11: 주문이 시작되지 않음

**증상**:
- Firebase에 주문을 생성했지만 로봇이 반응 없음
- "Waiting for Web Order..." 메시지만 계속 출력

**원인**:
- `status` 필드가 `start`가 아님
- DoE Matrix 형식 오류

**진단**:

Firebase Console → Realtime Database에서 확인:

```json
{
  "current_order": {
    "order_id": "order_20240111_001",
    "status": "start",  // ← 이 값이 "start"여야 함
    "doe_matrix": [...]
  }
}
```

**해결 방법**:

```bash
# 1. Firebase Console에서 직접 수정
# current_order/status를 "start"로 변경

# 2. 또는 Python으로 주문 생성
python3 << EOF
import firebase_admin
from firebase_admin import credentials, db

cred = credentials.Certificate('serviceAccountKey.json')
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://your-project.firebaseio.com/'
})

ref = db.reference('current_order')
ref.set({
    'order_id': 'test_order_001',
    'status': 'start',
    'doe_matrix': [
        {
            'well_id': 'A1',
            'counts': {'l1': 1, 'l2': 0, 'p1': 0, 'p2': 0}
        }
    ]
})
print("주문 생성 완료")
EOF
```

---

### 문제 12: DoE Matrix 파싱 에러

**증상**:
```
KeyError: 'well_id'
```

**원인**:
- DoE Matrix 형식이 잘못됨

**올바른 형식**:

```json
{
  "current_order": {
    "order_id": "order_001",
    "status": "start",
    "doe_matrix": [
      {
        "well_id": "A1",
        "counts": {
          "l1": 2,
          "l2": 1,
          "p1": 0,
          "p2": 0
        }
      },
      {
        "well_id": "A2",
        "counts": {
          "l1": 0,
          "l2": 1,
          "p1": 1,
          "p2": 1
        }
      }
    ]
  }
}
```

**주의사항**:
- `well_id`는 필수 (A1, A2, B1 형식)
- `counts`의 키는 정확히 `l1`, `l2`, `p1`, `p2`
- 값은 정수(int)

---

### 문제 13: 특정 웰에서만 에러 발생

**증상**:
- A1, A2는 성공하지만 B1에서 에러

**원인**:
- 트레이 좌표 계산 오류
- Pitch 값 잘못 설정

**진단**:

```python
# 좌표 계산 확인
base = [100, 450, 280, 0, 180, 0]
pitch_x = 57.0
pitch_y = 38.0

# B1 계산: base.X + pitch_x
b1_x = 100 + 57.0  # = 157.0

# 수동 이동 테스트
from DSR_ROBOT2 import movel, posx
movel(posx([157.0, 450, 280, 0, 180, 0]), vel=500, acc=200)
```

**해결 방법**:

```bash
# 1. 트레이 좌표 재티칭
# coordinate_teaching.md 참조

# 2. Pitch 값 재측정
# 실제 캘리퍼스로 웰 간격 확인

# 3. material_library.yaml 수정
nano material_library.yaml
```

---

## 에러 복구 문제

### 문제 14: 자동 복구가 실패함

**증상**:
- "Resume" 버튼을 눌렀지만 계속 에러 발생
- "Retry failed" 메시지 반복

**원인**:
- 하드웨어 문제 (E-Stop 등)
- 심각한 에러 상태

**해결 방법**:

```bash
# 1. 티칭 펜던트 확인
# - E-Stop 상태
# - 에러 메시지 확인

# 2. 수동 개입
# Case A: E-Stop
#   → E-Stop 버튼 회전하여 해제
#   → 펜던트에서 Servo-On

# Case B: 충돌 감지
#   → 로봇을 Free Drive로 안전한 위치로 이동
#   → Protective Stop 해제

# Case C: 전원 문제
#   → 로봇 제어기 재부팅
#   → 프로그램 재시작

# 3. 체크포인트 강제 삭제 (최후 수단)
python3 << EOF
import firebase_admin
from firebase_admin import credentials, db

cred = credentials.Certificate('serviceAccountKey.json')
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://your-project.firebaseio.com/'
})

db.reference('task_checkpoint').delete()
print("체크포인트 삭제 완료")
EOF
```

---

### 문제 15: 복구 후 잘못된 위치에서 재개

**증상**:
- Resume 후 예상치 못한 위치로 이동

**원인**:
- 체크포인트 데이터 손상
- 수동 이동으로 인한 위치 불일치

**해결 방법**:

```bash
# 1. 현재 체크포인트 확인
# Firebase Console → task_checkpoint

# 2. 안전한 위치로 수동 이동
# Free Drive 모드로 HOME 위치로 이동

# 3. 체크포인트 초기화
python3 << EOF
from firebase_admin import db
db.reference('task_checkpoint').delete()
db.reference('current_order/status').set('start')
print("주문 처음부터 재시작")
EOF

# 4. beauro.py 재실행
```

---

## 그리퍼 및 I/O 문제

### 문제 16: 그리퍼가 동작하지 않음

**증상**:
- `gripper_control("hold")` 호출해도 반응 없음

**원인**:
- 공압 연결 불량
- 디지털 출력 설정 오류

**진단**:

```bash
# 1. 공압 압력 확인
# 컴프레서 압력: 0.5~0.6 MPa

# 2. 에어 호스 연결 확인
# 제어기 → 그리퍼 연결 상태

# 3. 디지털 출력 테스트
python3 << EOF
from DSR_ROBOT2 import *
DR_init()

# DO 1번 ON
set_digital_output(1, ON)
time.sleep(1)

# DO 1번 OFF
set_digital_output(1, OFF)
EOF

# 그리퍼 동작 육안 확인
```

**해결 방법**:

```bash
# Case 1: 공압 문제
# - 컴프레서 전원 확인
# - 압력 조절 밸브 확인
# - 에어 호스 누기 점검

# Case 2: 신호 문제
# - 로봇 제어기 I/O 설정 확인
# - 케이블 연결 재확인

# Case 3: beauro.py 코드 수정
nano beauro.py
```

```python
# gripper_control 함수의 신호 매핑 확인
GRIPPER_SIGNALS = {
    "init": [1, 0, 0, 0, 0, 0],    # DO 1번 ON
    "hold": [0, 1, 0, 0, 0, 0],    # DO 2번 ON
    # ...
}

# 실제 하드웨어 연결에 맞게 수정
```

---

### 문제 17: 피펫 흡입/분주 불량

**증상**:
- 액체를 흡입하지 못함
- 또는 엉뚱한 곳에 분주

**원인**:
- 피펫 위치 부정확
- 흡입 시간 부족
- 에어 누기

**해결 방법**:

```python
# beauro.py에서 흡입/분주 파라미터 조정

# 1. 흡입 시간 연장
time.sleep(1.5)  # 기본 1초 → 1.5초

# 2. 피펫 깊이 조정
PIPETTE_DIP_DEPTH = 10  # mm, 더 깊이 담금

# 3. 분주 대기 시간
time.sleep(0.8)  # 액체가 완전히 떨어질 때까지

# 4. 좌표 재티칭
# coordinate_teaching.md 참조
```

---

## 성능 및 정확도 문제

### 문제 18: 좌표 정확도 불량

**증상**:
- 티칭한 위치와 실제 도달 위치가 다름

**원인**:
- TCP 설정 불일치
- 로봇 캘리브레이션 필요
- 페이로드 설정 오류

**진단**:

```python
# 현재 TCP 확인
from DSR_ROBOT2 import get_current_tcp
print(get_current_tcp())

# 현재 페이로드 확인
from DSR_ROBOT2 import get_current_tool
print(get_current_tool())
```

**해결 방법**:

```bash
# 1. TCP 재설정
# 티칭 펜던트 → 설정 → TCP

# 2. beauro.py에서 TCP 지정
nano beauro.py
```

```python
# 로봇 초기화 시
ROBOT_TCP = "GripperDA_v1"  # 실제 사용 중인 TCP 이름

# 또는 수동 설정
from DSR_ROBOT2 import set_tcp
set_tcp("GripperDA_v1")
```

```bash
# 3. 페이로드 설정
python3 << EOF
from DSR_ROBOT2 import *
DR_init()

# 그리퍼 무게 설정 (kg, 질량중심 좌표)
set_tool("Tool Weight", 0.5, [0, 0, 100], [1, 0, 0, 0, 1, 0, 0, 0, 1])
EOF
```

---

### 문제 19: 속도가 너무 느림 또는 빠름

**증상**:
- 작업 속도가 부적절함

**원인**:
- 속도 파라미터 설정

**해결 방법**:

```python
# beauro.py에서 속도 조정

# 빠른 이동 (재료 간 이동)
VEL_MOVE = 2000  # mm/s (기본값)
# 너무 느리면 → 2500~3000
# 너무 빠르면 → 1500~1800

# 작업 속도 (분주, 교반)
VEL_WORK = 500   # mm/s
# 정밀도 향상 필요 → 300~400
# 속도 향상 필요 → 600~800

# 스푼 작업 (분말)
VEL_SPOON = 250  # mm/s (천천히)
```

---

### 문제 20: 교반이 불충분함

**증상**:
- 스틱 교반 후에도 재료가 섞이지 않음

**원인**:
- 교반 패턴 부족
- 교반 속도 부적절

**해결 방법**:

```python
# beauro.py의 execute_sticks 함수 수정

# 1. 교반 반복 횟수 증가
for _ in range(3):  # 기본 3회
    # ...

# → 5회로 증가
for _ in range(5):

# 2. 교반 속도 조정
VEL_WORK = 500  # 기본값
# → 더 강한 교반이 필요하면 600~700

# 3. 교반 패턴 추가
# STIR_POSES_BASE에 더 많은 위치 추가
STIR_POSES_BASE = [
    [0, 0, 0],
    [10, 0, 0],
    [0, 10, 0],
    [-10, 0, 0],
    [0, -10, 0],
    [5, 5, 0],   # 추가
    [-5, -5, 0], # 추가
]
```

---

## 데이터 및 설정 문제

### 문제 21: YAML 파싱 에러

**증상**:
```
yaml.scanner.ScannerError: while scanning...
```

**원인**:
- YAML 문법 오류
- 들여쓰기 불일치

**해결 방법**:

```bash
# 1. YAML 문법 검사
python3 << EOF
import yaml

try:
    with open('material_library.yaml', 'r') as f:
        data = yaml.safe_load(f)
    print("✅ YAML 파일 정상")
except Exception as e:
    print(f"❌ YAML 에러: {e}")
EOF

# 2. 일반적인 YAML 문법 오류들:

# 잘못된 들여쓰기 (탭 사용)
liquid_A:
	name: "..."  # ❌ 탭 사용

# 올바른 들여쓰기 (스페이스 2칸)
liquid_A:
  name: "..."  # ✅

# 콜론 뒤 공백 없음
position:[400, 0, 300]  # ❌

position: [400, 0, 300]  # ✅

# 3. 백업에서 복원
cp material_library.yaml.backup material_library.yaml
```

---

### 문제 22: 설정 변경이 반영되지 않음

**증상**:
- `material_library.yaml` 수정했지만 이전 좌표로 동작

**원인**:
- 파일을 저장하지 않음
- 잘못된 파일 편집

**해결 방법**:

```bash
# 1. 파일 저장 확인
nano material_library.yaml
# Ctrl+O (저장), Enter, Ctrl+X (종료)

# 2. 파일 위치 확인
ls -l ~/beauro/material_library.yaml

# 3. beauro.py에서 올바른 경로 로드하는지 확인
grep "LIBRARY_YAML_PATH" beauro.py

# 4. 프로그램 재시작 (변경사항 적용)
# Ctrl+C로 종료 후 재실행
python3 beauro.py
```

---

## 긴급 상황 대처

### 상황 1: 로봇이 충돌 위험

**즉시 조치**:
1. **비상정지 버튼** 누름 (제어기 또는 펜던트)
2. 사람이 안전 거리 확보
3. 충돌 가능성 있는 물체 제거

**복구**:
```bash
# 1. E-Stop 해제
# 2. Free Drive로 안전한 위치로 이동
# 3. 좌표 재확인
# 4. 프로그램 재시작
```

---

### 상황 2: 재료 유출/비산

**즉시 조치**:
1. 로봇 정지 (비상정지)
2. 유출 재료 처리
3. 그리퍼/용기 점검

**재발 방지**:
```python
# 속도 감소
VEL_SPOON = 200  # 분말 속도 더 감소
ACC_SPOON = 150   # 가속도도 감소

# 이동 경로 최적화
# 급격한 방향 전환 → radius 파라미터 추가
movel(target, vel=500, acc=200, radius=20)
```

---

### 상황 3: 프로그램 응답 없음

**증상**:
- Ctrl+C로도 종료되지 않음

**조치**:
```bash
# 1. 강제 종료
ps aux | grep beauro.py
kill -9 <PID>

# 2. 로봇 안전 정지
# 펜던트에서 Safe Stop

# 3. ROS2 노드 정리
ros2 daemon stop
ros2 daemon start

# 4. 체크포인트 확인
# Firebase Console에서 task_checkpoint 상태 확인
```

---

## 로그 및 디버깅

### 디버그 모드 활성화

```python
# beauro.py 시작 부분에 추가
import logging

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler('beauro.log'),
        logging.StreamHandler()
    ]
)
```

### 로그 분석

```bash
# 실시간 로그 확인
tail -f beauro.log

# 에러만 필터링
grep "ERROR" beauro.log

# 특정 작업 검색
grep "liquid_A" beauro.log
```

---

## 예방적 유지보수

### 일일 점검

- [ ] 로봇 베이스 고정 상태
- [ ] 케이블 손상 여부
- [ ] 그리퍼 동작 테스트
- [ ] 공압 압력 확인
- [ ] 재료 용기 고정 상태

### 주간 점검

- [ ] 좌표 정확도 검증
- [ ] 네트워크 연결 안정성
- [ ] Firebase 데이터 백업
- [ ] 로그 파일 정리
- [ ] 소프트웨어 업데이트 확인

### 월간 점검

- [ ] 로봇 캘리브레이션
- [ ] 그리퍼 교체/청소
- [ ] 전체 시스템 성능 평가
- [ ] 설정 파일 백업
- [ ] 하드웨어 점검 (볼트, 나사)

---

## 지원 받기

### 자가 진단 후에도 해결되지 않으면

1. **로그 수집**
   ```bash
   # 시스템 정보
   ros2 doctor > system_info.txt
   
   # 로그 압축
   tar -czf beauro_logs_$(date +%Y%m%d).tar.gz beauro.log system_info.txt
   ```

2. **이슈 등록**
   - GitHub Issues: https://github.com/rokey-hyunj/beauro/issues
   - 제목: [BUG] 간단한 설명
   - 내용: 증상, 재현 방법, 로그 첨부

3. **연락처**
   - 이메일: hyunjongkim0524@gmail.com

---

**📝 마지막 업데이트**: 2025-12-19
**📌 작성자**: BeauRo Team  
**🔄 버전**: v1.0