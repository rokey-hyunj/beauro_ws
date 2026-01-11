# 🎯 좌표 티칭 가이드

이 문서는 BeauRo 시스템에서 사용되는 모든 작업 위치의 좌표를 티칭하는 방법을 설명합니다.

## 목차

- [티칭 개요](#티칭-개요)
- [준비 사항](#준비-사항)
- [좌표계 이해](#좌표계-이해)
- [티칭 방법](#티칭-방법)
- [각 작업별 좌표 티칭](#각-작업별-좌표-티칭)
- [좌표 검증](#좌표-검증)
- [팁과 모범 사례](#팁과-모범-사례)
- [문제 해결](#문제-해결)

---

## 티칭 개요

### 티칭이 필요한 이유

BeauRo 시스템은 다음 위치들의 정확한 좌표가 필요합니다:
- 재료 용기 위치 (액상/분말)
- 그리퍼 보관 위치
- 트레이 베이스 위치
- 작업 완료 후 출력 위치

각 작업장의 레이아웃에 따라 이러한 위치가 달라지므로, 설치 후 반드시 좌표를 티칭해야 합니다.

### 티칭 필요 항목 체크리스트

- [ ] 액상 재료 A, B 위치
- [ ] 분말 재료 A, B 위치
- [ ] 피펫 그리퍼 위치 (파지/반납)
- [ ] 스푼 그리퍼 위치 (파지/반납)
- [ ] 스틱 그리퍼 위치 (파지/반납/드롭)
- [ ] 트레이 베이스 좌표 및 웰 간격
- [ ] 트레이 출력 위치

---

## 준비 사항

### 1. 하드웨어 준비

- **로봇**: 전원 ON, Servo-On 상태
- **티칭 펜던트**: 배터리 충전 완료
- **그리퍼**: 각 종류별 장착 준비
- **재료 용기**: 실제 사용할 용기 배치
- **트레이**: 실제 사용할 웰 플레이트 배치
- **안전 장비**: 안전 장갑, 보안경

### 2. 소프트웨어 준비

```bash
# ROS2 환경 소싱
source /opt/ros/humble/setup.bash
source ~/doosan_ws/install/setup.bash

# material_library.yaml 백업
cd ~/beauro
cp material_library.yaml material_library.yaml.backup

# 텍스트 에디터로 열기
nano material_library.yaml
```

### 3. 안전 확인

- [ ] 로봇 주변 장애물 제거
- [ ] 비상정지 버튼 위치 확인
- [ ] 작업 공간 내 사람 확인
- [ ] 그리퍼 공압 연결 확인
- [ ] 재료 용기 고정 상태 확인

---

## 좌표계 이해

### 두산 로보틱스 좌표계

BeauRo는 **Base 좌표계**와 **Task 좌표계 (posx)**를 사용합니다.

#### Base 좌표계 (베이스 기준)
```
        Z↑ (수직 상방)
         |
         |___→ Y
        /
       /
      ↓ X

원점: 로봇 베이스 중심
```

#### Task 좌표계 (posx)
```
posx = [X, Y, Z, Rx, Ry, Rz]

위치:
- X: 전후 (mm) - 로봇 기준 앞(+) / 뒤(-)
- Y: 좌우 (mm) - 로봇 기준 왼쪽(+) / 오른쪽(-)
- Z: 상하 (mm) - 위(+) / 아래(-)

자세:
- Rx: X축 회전 (degree) - Roll
- Ry: Y축 회전 (degree) - Pitch
- Rz: Z축 회전 (degree) - Yaw
```

### 예시 좌표
```python
# 로봇 정면 400mm, 높이 300mm 위치, 그리퍼 수직 하향
posx([400, 0, 300, 0, 180, 0])

# 설명:
# X=400: 로봇에서 앞으로 400mm
# Y=0: 중앙
# Z=300: 베이스에서 300mm 위
# Rx=0, Ry=180 (그리퍼 아래 방향), Rz=0
```

---

## 티칭 방법

### 방법 1: 티칭 펜던트 사용 (권장)

#### 단계별 절차

1. **로봇 모드 변경**
   - 펜던트에서 `Manual` 모드 선택
   - Servo-On 확인

2. **자유 드라이브 활성화**
   - 펜던트의 `Free Drive` 버튼 누름
   - 로봇을 손으로 움직일 수 있음

3. **목표 위치로 이동**
   - 로봇 암을 부드럽게 원하는 위치로 이동
   - 그리퍼가 재료/대상물과 정렬되도록 조정
   - 높이는 안전 여유를 두고 설정

4. **현재 좌표 확인**
   - 펜던트 화면에서 `좌표` 메뉴 선택
   - `Current Position` → `Task` 선택
   - posx 값 확인 및 기록

5. **좌표 저장**
   - 종이에 기록하거나
   - 펜던트에서 `Save Position` 기능 활용
   - 또는 직접 `material_library.yaml`에 입력

#### 펜던트 화면 예시
```
현재 위치 (Task):
X:  423.45 mm
Y:  -12.33 mm
Z:  287.91 mm
Rx:   0.12 deg
Ry: 179.88 deg
Rz:  -0.05 deg
```

### 방법 2: ROS2 토픽 사용

```bash
# 현재 로봇 위치 실시간 확인
ros2 topic echo /dsr01/state/current_pose

# 출력 예시:
# pos: [423.45, -12.33, 287.91, 0.12, 179.88, -0.05]
```

### 방법 3: Python 스크립트 (고급)

```python
# teach_positions.py
import rclpy
from dsr_msgs2.msg import RobotState

def pose_callback(msg):
    pos = msg.current_pose
    print(f"posx([{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}, "
          f"{pos[3]:.2f}, {pos[4]:.2f}, {pos[5]:.2f}])")

rclpy.init()
node = rclpy.create_node('teach_helper')
subscription = node.create_subscription(
    RobotState,
    '/dsr01/state/robot_state',
    pose_callback,
    10
)

print("Free Drive로 로봇을 이동시키세요. Ctrl+C로 종료.")
rclpy.spin(node)
```

실행:
```bash
python3 teach_positions.py
```

---

## 각 작업별 좌표 티칭

### 1. 액상 재료 위치 (Liquid A, B)

#### 티칭 목표
피펫 그리퍼가 용기 중앙 상단에 위치하여 흡입할 수 있도록 설정합니다.

#### 절차

1. **피펫 그리퍼 장착**
   - 로봇 끝단에 피펫 그리퍼 장착
   - TCP 설정 확인: `GripperDA_v1`

2. **액상 용기 A 배치**
   - 작업 공간에 액상 재료 A 용기 고정
   - 용기 내부에 충분한 액체 있어야 함

3. **피펫 위치 조정**
   - Free Drive로 피펫 끝이 용기 중앙 **5-10mm 위**에 오도록 조정
   - 피펫이 수직(Ry=180)인지 확인
   - 용기 벽에 닿지 않도록 여유 확보

4. **좌표 기록**
   ```yaml
   liquid_A:
     name: "Hyaluronic Acid Solution"
     type: "liquid"
     position: [450.2, -80.5, 285.0, 0, 180, 0]
   ```

5. **액상 B도 동일하게 반복**
   ```yaml
   liquid_B:
     name: "Vitamin C Serum"
     type: "liquid"
     position: [450.2, 20.5, 285.0, 0, 180, 0]
   ```

#### 주의사항
- 피펫이 용기 바닥에 닿으면 막힐 수 있으므로 높이 여유 필요
- 용기가 움직이지 않도록 테이프나 고정대로 고정
- 액체 레벨이 낮아지면 좌표 재조정 필요

---

### 2. 분말 재료 위치 (Powder A, B)

#### 티칭 목표
스푼 그리퍼가 분말을 떠올릴 수 있는 위치를 설정합니다.

#### 절차

1. **스푼 그리퍼 장착**
   - 피펫 제거 후 스푼 그리퍼 장착

2. **분말 용기 배치**
   - 분말이 담긴 용기를 평평한 곳에 고정
   - 용기 높이가 일정한지 확인

3. **스쿱 위치 설정**
   - 스푼이 분말 표면 **바로 위**에 오도록 조정
   - 스푼 각도는 수평 또는 약간 기울임
   - X, Y는 용기 중앙

4. **좌표 기록**
   ```yaml
   powder_A:
     name: "Titanium Dioxide"
     type: "powder"
     position: [520.0, -100.0, 240.0, 0, 180, 0]
     scoop_depth: 15  # 떠올릴 깊이 (mm)
   ```

5. **레벨링 위치 (선택사항)**
   - 스푼 평탄화를 위한 별도 위치 필요 시
   ```yaml
   powder_A_leveler:
     position: [520.0, -100.0, 235.0, 0, 180, 0]
   ```

#### 주의사항
- 분말이 흩날리지 않도록 속도 제어 필요
- 용기 깊이에 따라 Z값 조정
- 분말이 소진되면 높이 변화 고려

---

### 3. 그리퍼 파지/반납 위치

#### 3-1. 스틱 그리퍼

스틱을 교반 작업에 사용하기 위한 위치들입니다.

```yaml
stick:
  ready_1:    # 접근 준비 위치 (높은 곳)
    position: [350.0, -250.0, 450.0, 0, 180, 0]
  
  ready_2:    # 파지 직전 위치 (중간 높이)
    position: [350.0, -250.0, 320.0, 0, 180, 0]
  
  grab:       # 실제 파지 위치
    position: [350.0, -250.0, 280.0, 0, 180, 0]
  
  drop:       # 사용 후 버리는 위치 (공중)
    position: [200.0, 200.0, 450.0, 0, 180, 0]
  
  finished:   # 작업 완료 후 대기 위치
    position: [200.0, 200.0, 500.0, 0, 180, 0]
```

#### 티칭 팁
- `ready_1` → `ready_2` → `grab` 순서로 안전하게 하강
- `grab` 위치에서 그리퍼를 닫았을 때 스틱이 확실히 파지되는지 확인
- `drop` 위치는 스틱 수거함 위 공중

---

### 4. 트레이 좌표

#### 트레이 베이스 및 웰 간격

트레이는 **하나의 기준점 + 간격**으로 모든 웰 위치를 계산합니다.

```yaml
tray_base:
  base: [100.0, 450.0, 280.0, 0, 180, 0]  # A1 웰 위치
  pitch_x: 57.0   # X축 웰 간격 (mm)
  pitch_y: 38.0   # Y축 웰 간격 (mm)
```

#### 티칭 절차

1. **A1 웰 중앙 정렬**
   - 피펫 또는 스틱을 트레이 A1 웰 중앙 상단에 위치
   - 높이는 웰 상단에서 20-30mm 여유

2. **기준 좌표 기록**
   - 이 위치를 `base`로 저장

3. **웰 간격 측정**
   - 실제 트레이의 웰 간격을 캘리퍼스로 측정
   - 일반적인 96-well plate: X=9mm, Y=9mm
   - 24-well plate: X=19.3mm, Y=19.3mm
   - **BeauRo 기본값**: X=57mm, Y=38mm (커스텀 트레이)

4. **검증**
   - A2 위치로 이동: `base.Y + pitch_y`
   - B1 위치로 이동: `base.X + pitch_x`
   - 실제 웰 중앙에 정확히 오는지 확인

#### 트레이 배치 규칙

```
     1      2      3      4   ...
   ┌─────┬─────┬─────┬─────┐
A  │ A1  │ A2  │ A3  │ A4  │
   ├─────┼─────┼─────┼─────┤
B  │ B1  │ B2  │ B3  │ B4  │
   ├─────┼─────┼─────┼─────┤
C  │ C1  │ C2  │ C3  │ C4  │
   └─────┴─────┴─────┴─────┘

- Y축: 열 방향 (1, 2, 3...)
- X축: 행 방향 (A, B, C...)

A2 위치 = base + [0, pitch_y, 0, 0, 0, 0]
B1 위치 = base + [pitch_x, 0, 0, 0, 0, 0]
B3 위치 = base + [pitch_x, 2*pitch_y, 0, 0, 0, 0]
```

---

### 5. 트레이 출력 위치

완성된 트레이를 옮기는 경로입니다.

```yaml
tray_out:
  ready_1:    # 트레이 접근 준비
    position: [150.0, 500.0, 400.0, 0, 180, 0]
  
  ready_2:    # 트레이 바로 위
    position: [150.0, 500.0, 290.0, 0, 180, 0]
  
  grab:       # 트레이 파지 (그리퍼 닫힘)
    position: [150.0, 500.0, 250.0, 0, 180, 0]
  
  drop:       # 출력 위치 (완제품 보관)
    position: [750.0, 300.0, 280.0, 0, 180, 0]
  
  finished:   # 최종 복귀 위치
    position: [750.0, 300.0, 450.0, 0, 180, 0]
```

---

## 좌표 검증

티칭 완료 후 반드시 검증이 필요합니다.

### 검증 스크립트

```python
# verify_positions.py
import yaml
from DSR_ROBOT2 import *

# 라이브러리 로드
with open('material_library.yaml', 'r') as f:
    library = yaml.safe_load(f)

# 로봇 초기화
DR_init()

def test_position(name, pos, description):
    print(f"\n테스트: {name} - {description}")
    input("로봇이 이동합니다. 계속하려면 Enter...")
    
    movel(posx(pos), vel=500, acc=200)
    
    response = input("위치가 정확합니까? (y/n): ")
    return response.lower() == 'y'

# 액상 재료 테스트
print("\n=== 액상 재료 위치 검증 ===")
test_position("Liquid A", library['liquid_A']['position'], "피펫 흡입 위치")
test_position("Liquid B", library['liquid_B']['position'], "피펫 흡입 위치")

# 분말 재료 테스트
print("\n=== 분말 재료 위치 검증 ===")
test_position("Powder A", library['powder_A']['position'], "스푼 스쿱 위치")
test_position("Powder B", library['powder_B']['position'], "스푼 스쿱 위치")

# 트레이 웰 테스트
print("\n=== 트레이 위치 검증 ===")
base = library['tray_base']['base']
px = library['tray_base']['pitch_x']
py = library['tray_base']['pitch_y']

test_position("A1", base, "트레이 기준점")
test_position("A2", [base[0], base[1]+py, base[2], base[3], base[4], base[5]], "Y축 간격")
test_position("B1", [base[0]+px, base[1], base[2], base[3], base[4], base[5]], "X축 간격")

print("\n=== 검증 완료 ===")
movej([0, 0, 90, 0, 90, 0], time=3)
```

실행:
```bash
python3 verify_positions.py
```

### 수동 검증 체크리스트

각 위치에서 확인할 사항:

**액상 위치**:
- [ ] 피펫이 용기 중앙에 위치
- [ ] 피펫 끝이 액체 표면 5-10mm 위
- [ ] 용기 벽에 닿지 않음
- [ ] 흡입 후 정상 동작

**분말 위치**:
- [ ] 스푼이 분말 표면 바로 위
- [ ] 스푼이 용기 내부에 위치
- [ ] 스쿱 동작 후 적정량 채취
- [ ] 레벨링 동작 원활

**트레이 위치**:
- [ ] A1 웰 정확히 중앙
- [ ] A2, B1 등 인접 웰도 정확
- [ ] 모든 웰에서 간섭 없음
- [ ] 높이가 안전 범위 내

---

## 팁과 모범 사례

### 1. 안전 우선

```yaml
# 나쁜 예: 너무 낮은 접근 고도
position: [400, 0, 200, 0, 180, 0]  # 충돌 위험!

# 좋은 예: 충분한 여유
position: [400, 0, 350, 0, 180, 0]  # 안전
```

### 2. 계층적 접근

항상 **높은 곳 → 중간 → 낮은 곳** 순서로 이동하도록 좌표 설정:

```yaml
grab_sequence:
  ready_high:  [x, y, 500, rx, ry, rz]  # 1단계: 높이
  ready_mid:   [x, y, 350, rx, ry, rz]  # 2단계: 중간
  grab_low:    [x, y, 280, rx, ry, rz]  # 3단계: 작업
```

### 3. 대칭성 활용

재료가 대칭으로 배치되어 있으면 하나만 티칭 후 오프셋 적용:

```python
# Liquid A 티칭: [450, -80, 285, 0, 180, 0]
# Liquid B는 Y축 100mm 이동
liquid_B = [450, -80+100, 285, 0, 180, 0]
# = [450, 20, 285, 0, 180, 0]
```

### 4. 기준점 마킹

- 작업대에 테이프로 재료 용기 위치 마킹
- 트레이 고정 지그 사용
- 재티칭 시 일관성 확보

### 5. 문서화

```yaml
# 좋은 문서화 예시
liquid_A:
  name: "Hyaluronic Acid Solution"
  type: "liquid"
  position: [450.2, -80.5, 285.0, 0, 180, 0]
  notes: "티칭 날짜: 2024-01-10, 담당자: 홍길동"
  container: "100ml 비커, 테이프 고정"
  teaching_gripper: "피펫 그리퍼"
```

### 6. 백업

```bash
# 티칭 후 즉시 백업
cp material_library.yaml material_library_YYYYMMDD.yaml

# Git으로 버전 관리
git add material_library.yaml
git commit -m "좌표 티칭 완료 - 2024-01-10"
```

---

## 문제 해결

### 문제 1: 좌표로 이동했는데 위치가 맞지 않음

**원인**:
- TCP(Tool Center Point) 설정 불일치
- 그리퍼 무게로 인한 처짐

**해결**:
```python
# TCP 확인
from DSR_ROBOT2 import get_current_tcp
print(get_current_tcp())

# beauro.py에서 설정된 TCP와 일치하는지 확인
ROBOT_TCP = "GripperDA_v1"
```

### 문제 2: 트레이 웰 간격이 맞지 않음

**원인**:
- 측정 오류
- 트레이 각도 틀어짐

**해결**:
```bash
# 트레이를 수평계로 확인
# 웰 간격을 캘리퍼스로 재측정
# A1, A6, F1, F6 등 네 모서리 확인
```

### 문제 3: 재료 용기 위치가 자주 바뀜

**원인**:
- 용기 고정 불충분
- 작업 중 충격

**해결**:
```yaml
# 고정 지그 제작 또는
# 용기 바닥에 논슬립 패드 부착
# 테이블에 위치 마킹 테이프
```

### 문제 4: 그리퍼 교체 후 좌표가 맞지 않음

**원인**:
- 각 그리퍼의 TCP 다름
- 무게 차이로 인한 처짐

**해결**:
```python
# 그리퍼별 TCP 별도 저장
tcp_configs = {
    "pipette": "GripperDA_v1",
    "spoon": "GripperDA_v2",
    "stick": "GripperDA_v3"
}

# 각 그리퍼별로 좌표 재티칭하거나
# 오프셋 계산 적용
```

---

## 좌표 데이터 완성 예시

모든 티칭이 완료된 `material_library.yaml` 예시:

```yaml
# ========================================
# BeauRo Material Library - 좌표 설정
# 티칭 날짜: 2024-01-10
# 티칭 담당자: 김로봇
# ========================================

# 액상 재료
liquid_A:
  name: "Hyaluronic Acid Solution"
  type: "liquid"
  position: [450.2, -80.5, 285.0, 0.0, 180.0, 0.0]
  container: "100ml 비커"
  
liquid_B:
  name: "Vitamin C Serum"
  type: "liquid"
  position: [450.2, 20.5, 285.0, 0.0, 180.0, 0.0]
  container: "100ml 비커"

# 분말 재료
powder_A:
  name: "Titanium Dioxide"
  type: "powder"
  position: [520.0, -100.0, 240.0, 0.0, 180.0, 0.0]
  container: "분말 트레이"
  
powder_B:
  name: "Mica Powder"
  type: "powder"
  position: [520.0, 0.0, 240.0, 0.0, 180.0, 0.0]
  container: "분말 트레이"

# 스틱 그리퍼 위치
stick:
  ready_1: [350.0, -250.0, 450.0, 0.0, 180.0, 0.0]
  ready_2: [350.0, -250.0, 320.0, 0.0, 180.0, 0.0]
  grab: [350.0, -250.0, 280.0, 0.0, 180.0, 0.0]
  drop: [200.0, 200.0, 450.0, 0.0, 180.0, 0.0]
  finished: [200.0, 200.0, 500.0, 0.0, 180.0, 0.0]

# 트레이 좌표
tray_base:
  base: [100.0, 450.0, 280.0, 0.0, 180.0, 0.0]  # A1 웰
  pitch_x: 57.0   # 행 간격 (A→B)
  pitch_y: 38.0   # 열 간격 (1→2)

# 트레이 출력
tray_out:
  ready_1: [150.0, 500.0, 400.0, 0.0, 180.0, 0.0]
  ready_2: [150.0, 500.0, 290.0, 0.0, 180.0, 0.0]
  grab: [150.0, 500.0, 250.0, 0.0, 180.0, 0.0]
  drop: [750.0, 300.0, 280.0, 0.0, 180.0, 0.0]
  finished: [750.0, 300.0, 450.0, 0.0, 180.0, 0.0]

# ========================================
# 티칭 메모
# - 모든 좌표는 피펫 그리퍼 기준
# - TCP: GripperDA_v1
# - 안전 높이: 400mm 이상
# - 트레이: 커스텀 24-well 플레이트
# ========================================
```

---

## 다음 단계

좌표 티칭이 완료되었다면:

1. **검증 스크립트 실행**: 모든 위치가 정확한지 확인
2. **시뮬레이션 테스트**: 실제 재료 없이 동작 확인
3. **실제 운영**: 소량 재료로 첫 배합 테스트
4. **미세 조정**: 필요시 좌표 수정

---

## 참고 자료

- 두산 로보틱스 매뉴얼: TCP 설정 방법
- ROS2 DSR_ROBOT2 문서: 좌표계 설명
- BeauRo README: 전체 시스템 개요

---

**📝 마지막 업데이트**: 2025-12-19
**📌 작성자**: BeauRo Team