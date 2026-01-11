# 📦 BeauRo 설치 가이드

이 문서는 BeauRo 시스템의 완전한 설치 과정을 단계별로 안내합니다.

## 목차

- [시스템 요구사항](#시스템-요구사항)
- [1단계: 운영체제 설정](#1단계-운영체제-설정)
- [2단계: ROS2 설치](#2단계-ros2-설치)
- [3단계: 두산 로보틱스 패키지 설치](#3단계-두산-로보틱스-패키지-설치)
- [4단계: Python 환경 구성](#4단계-python-환경-구성)
- [5단계: Firebase 설정](#5단계-firebase-설정)
- [6단계: 네트워크 구성](#6단계-네트워크-구성)
- [7단계: BeauRo 설치 및 설정](#7단계-beauro-설치-및-설정)
- [8단계: 설치 확인](#8단계-설치-확인)
- [부록: 일반적인 설치 문제](#부록-일반적인-설치-문제)

---

## 시스템 요구사항

### 하드웨어

- **로봇**: 두산 로보틱스 M0609
- **제어 PC**: 
  - CPU: Intel i5 (8세대) 이상 또는 AMD Ryzen 5 이상
  - RAM: 8GB 이상 (16GB 권장)
  - 저장공간: 50GB 이상 여유 공간
  - 네트워크: 이더넷 포트 1개 이상
- **그리퍼**: 공압 그리퍼 3종 (피펫, 스푼, 스틱)
- **컴프레서**: 공압 시스템용 (0.5~0.6 MPa)

### 소프트웨어

- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **ROS2**: Humble Hawksbill
- **Python**: 3.8 이상 (3.10 권장)
- **인터넷**: Firebase 통신을 위한 안정적인 연결

---

## 1단계: 운영체제 설정

### Ubuntu 22.04 LTS 설치

1. **Ubuntu 다운로드**
   ```bash
   # Ubuntu 공식 사이트에서 ISO 다운로드
   # https://ubuntu.com/download/desktop
   ```

2. **부팅 USB 생성**
   - Rufus (Windows) 또는 Etcher (Mac/Linux) 사용
   - ISO 이미지를 USB에 기록

3. **Ubuntu 설치**
   - BIOS에서 USB 부팅 설정
   - 설치 프로그램 실행
   - "Normal Installation" 선택
   - "Install third-party software" 체크

4. **기본 업데이트**
   ```bash
   sudo apt update
   sudo apt upgrade -y
   sudo apt install -y build-essential git curl wget
   ```

### 시스템 설정

```bash
# 한국어 입력기 설치 (선택사항)
sudo apt install -y ibus-hangul

# 개발 도구 설치
sudo apt install -y vim nano terminator htop
```

---

## 2단계: ROS2 설치

### ROS2 Humble 전체 설치

```bash
# 1. UTF-8 로케일 설정
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. ROS2 저장소 추가
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# 3. ROS2 GPG 키 추가
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# 4. 저장소를 소스 리스트에 추가
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 5. 패키지 목록 업데이트
sudo apt update
sudo apt upgrade -y

# 6. ROS2 Desktop 설치 (권장)
sudo apt install -y ros-humble-desktop

# 또는 ROS2 Base 설치 (최소 설치)
# sudo apt install -y ros-humble-ros-base

# 7. 개발 도구 설치
sudo apt install -y ros-dev-tools
```

### ROS2 환경 설정

```bash
# bashrc에 ROS2 자동 소싱 추가
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 설치 확인
ros2 --version
# 출력 예: ros2 cli version: 0.18.5
```

### Colcon 빌드 도구 설정

```bash
# colcon 자동완성 추가
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3단계: 두산 로보틱스 패키지 설치

### 작업 공간 생성

```bash
# ROS2 작업 공간 생성
mkdir -p ~/doosan_ws/src
cd ~/doosan_ws/src
```

### 두산 로보틱스 ROS2 패키지 클론

```bash
# 공식 저장소에서 클론
git clone -b humble https://github.com/doosan-robotics/doosan-robot2.git

# 또는 특정 버전 클론
# git clone -b v2.1.0 https://github.com/doosan-robotics/doosan-robot2.git
```

### 의존성 설치

```bash
cd ~/doosan_ws

# rosdep 초기화 (처음 한 번만)
sudo rosdep init
rosdep update

# 의존성 자동 설치
rosdep install --from-paths src --ignore-src -r -y

# 추가 필요 패키지 설치
sudo apt install -y \
  ros-humble-control-msgs \
  ros-humble-trajectory-msgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  python3-pip
```

### 빌드

```bash
cd ~/doosan_ws

# 전체 빌드
colcon build --symlink-install

# 성공 메시지 확인
# Summary: X packages finished [시간]

# 환경 소싱
source install/setup.bash

# bashrc에 자동 소싱 추가
echo "source ~/doosan_ws/install/setup.bash" >> ~/.bashrc
```

### 빌드 확인

```bash
# 패키지 목록 확인
ros2 pkg list | grep dsr

# 예상 출력:
# dsr_msgs2
# dsr_control2
# dsr_description2
# dsr_launcher2
```

---

## 4단계: Python 환경 구성

### Python 및 pip 업그레이드

```bash
# Python 버전 확인 (3.10 이상이어야 함)
python3 --version

# pip 업그레이드
python3 -m pip install --upgrade pip

# virtualenv 설치 (선택사항)
python3 -m pip install virtualenv
```

### BeauRo 필수 패키지 설치

```bash
# 시스템 전역 설치
sudo pip3 install firebase-admin PyYAML

# 또는 사용자 레벨 설치 (권장)
pip3 install --user firebase-admin PyYAML

# 추가 유틸리티 패키지
pip3 install --user numpy python-dotenv
```

### 가상 환경 사용 (선택사항)

```bash
# 프로젝트 디렉토리에서 가상환경 생성
cd ~/beauro
python3 -m venv venv

# 가상환경 활성화
source venv/bin/activate

# 패키지 설치
pip install firebase-admin PyYAML numpy

# 비활성화
deactivate
```

---

## 5단계: Firebase 설정

### Firebase 프로젝트 생성

1. **Firebase Console 접속**
   - https://console.firebase.google.com/ 방문
   - Google 계정으로 로그인

2. **프로젝트 생성**
   - "프로젝트 추가" 클릭
   - 프로젝트 이름: `beauro` (또는 원하는 이름)
   - Google Analytics: 선택사항 (비활성화 가능)
   - "프로젝트 만들기" 클릭

3. **Realtime Database 설정**
   - 좌측 메뉴에서 "빌드" → "Realtime Database" 선택
   - "데이터베이스 만들기" 클릭
   - 위치: `asia-southeast1` (싱가포르) 선택
   - 보안 규칙: "테스트 모드에서 시작" 선택 (나중에 변경 가능)
   - "사용 설정" 클릭

4. **데이터베이스 URL 확인**
   - Database 페이지 상단에서 URL 확인
   - 형식: `https://beauro-xxxxx.firebaseio.com/`
   - 또는: `https://beauro-xxxxx.asia-southeast1.firebasedatabase.app/`

### 서비스 계정 키 생성

1. **프로젝트 설정 이동**
   - Firebase Console 좌측 상단 ⚙️ → "프로젝트 설정"

2. **서비스 계정 탭**
   - "서비스 계정" 탭 선택
   - "새 비공개 키 생성" 클릭
   - "키 생성" 확인

3. **키 파일 저장**
   - 다운로드된 JSON 파일을 안전한 위치에 저장
   - 파일명을 `serviceAccountKey.json`으로 변경

### 보안 규칙 설정 (운영 환경)

Firebase Console → Realtime Database → 규칙 탭:

```json
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
    },
    "command": {
      ".read": true,
      ".write": true
    }
  }
}
```

**보안 강화 (선택사항)**:
```json
{
  "rules": {
    ".read": "auth != null",
    ".write": "auth != null"
  }
}
```

---

## 6단계: 네트워크 구성

### 로봇 제어기 네트워크 설정

1. **로봇 제어기 IP 확인**
   - 로봇 티칭 펜던트에서 확인
   - 기본값: `192.168.137.100`

2. **PC 네트워크 설정**
   ```bash
   # 네트워크 인터페이스 확인
   ip addr show
   
   # 예: enp3s0, eth0 등
   ```

3. **고정 IP 설정 (Ubuntu GUI)**
   - Settings → Network
   - 유선 연결 옆 ⚙️ 클릭
   - IPv4 탭:
     - IPv4 Method: **Manual**
     - Address: `192.168.137.10`
     - Netmask: `255.255.255.0`
     - Gateway: `192.168.137.1`
   - Apply

4. **고정 IP 설정 (명령줄)**
   ```bash
   # Netplan 설정 파일 편집
   sudo nano /etc/netplan/01-network-manager-all.yaml
   ```
   
   내용:
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
   
   적용:
   ```bash
   sudo netplan apply
   ```

5. **연결 확인**
   ```bash
   # 로봇 제어기 Ping 테스트
   ping 192.168.137.100
   
   # 성공 시:
   # 64 bytes from 192.168.137.100: icmp_seq=1 ttl=64 time=0.5 ms
   ```

### 방화벽 설정

```bash
# UFW 상태 확인
sudo ufw status

# ROS2 통신 포트 허용
sudo ufw allow from 192.168.137.0/24

# 또는 특정 포트만 허용
sudo ufw allow 7400:7500/tcp
sudo ufw allow 7400:7500/udp
```

---

## 7단계: BeauRo 설치 및 설정

### 프로젝트 클론

```bash
# 홈 디렉토리로 이동
cd ~

# GitHub에서 클론
git clone https://github.com/your-organization/beauro.git

# 또는 압축 파일 다운로드 후 압축 해제
# wget https://github.com/your-organization/beauro/archive/main.zip
# unzip main.zip
# mv beauro-main beauro
```

### 디렉토리 구조 확인

```bash
cd ~/beauro
tree -L 2

# 예상 출력:
# beauro/
# ├── beauro.py
# ├── material_library.yaml
# ├── requirements.txt
# ├── README.md
# ├── docs/
# │   ├── installation.md
# │   ├── coordinate_teaching.md
# │   └── troubleshooting.md
# └── config/
#     └── robot_config.yaml
```

### Python 의존성 설치

```bash
cd ~/beauro

# requirements.txt로 일괄 설치
pip3 install -r requirements.txt

# 또는 수동 설치
pip3 install firebase-admin PyYAML
```

### Firebase 키 파일 설정

```bash
# 서비스 계정 키를 프로젝트 디렉토리로 복사
cp ~/Downloads/beauro-xxxxx-firebase-adminsdk-xxxxx.json ~/beauro/serviceAccountKey.json

# 권한 설정 (보안)
chmod 600 ~/beauro/serviceAccountKey.json

# .gitignore에 추가 (이미 추가되어 있어야 함)
echo "serviceAccountKey.json" >> .gitignore
```

### 설정 파일 편집

#### beauro.py 수정

```bash
nano beauro.py
```

다음 항목 확인/수정:
```python
# Firebase Database URL (본인의 URL로 변경)
FIREBASE_DB_URL = 'https://your-project-id.firebaseio.com/'

# 로봇 ID (필요시 변경)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
```

#### material_library.yaml 생성

```bash
nano material_library.yaml
```

기본 템플릿 (좌표는 티칭 후 업데이트):
```yaml
liquid_A:
  name: "Liquid Material A"
  type: "liquid"
  position: [400, 0, 300, 0, 180, 0]

liquid_B:
  name: "Liquid Material B"
  type: "liquid"
  position: [400, 100, 300, 0, 180, 0]

powder_A:
  name: "Powder Material A"
  type: "powder"
  position: [500, 0, 250, 0, 180, 0]

powder_B:
  name: "Powder Material B"
  type: "powder"
  position: [500, 100, 250, 0, 180, 0]

stick:
  ready_1: [300, -200, 400, 0, 180, 0]
  ready_2: [300, -200, 300, 0, 180, 0]
  grab: [300, -200, 250, 0, 180, 0]
  drop: [200, 200, 400, 0, 180, 0]
  finished: [200, 200, 500, 0, 180, 0]

tray_out:
  ready_1: [600, 0, 400, 0, 180, 0]
  ready_2: [600, 0, 300, 0, 180, 0]
  grab: [600, 0, 220, 0, 180, 0]
  drop: [700, 200, 250, 0, 180, 0]
  finished: [700, 200, 400, 0, 180, 0]

tray_base:
  base: [0, 400, 250, 0, 180, 0]
  pitch_x: 57.0
  pitch_y: 38.0
```

---

## 8단계: 설치 확인

### ROS2 환경 테스트

```bash
# 터미널 1: ROS2 소싱
source /opt/ros/humble/setup.bash
source ~/doosan_ws/install/setup.bash

# ROS2 데몬 확인
ros2 daemon status

# 예상 출력: The daemon is running
```

### 로봇 연결 테스트

```bash
# 로봇 제어기가 켜져 있고 네트워크 연결이 되어 있는지 확인
ping 192.168.137.100

# 두산 런처 실행 (테스트용)
ros2 launch dsr_launcher2 single_robot_rviz.launch.py \
  host:=192.168.137.100 \
  mode:=virtual

# RViz에서 로봇 모델이 보이면 성공
```

### BeauRo 초기 실행 테스트

```bash
cd ~/beauro

# Python 문법 체크
python3 -m py_compile beauro.py

# Firebase 연결 테스트
python3 -c "
import firebase_admin
from firebase_admin import credentials, db

cred = credentials.Certificate('serviceAccountKey.json')
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://your-project-id.firebaseio.com/'
})

# 테스트 데이터 쓰기
ref = db.reference('test')
ref.set({'status': 'connected'})

# 테스트 데이터 읽기
data = ref.get()
print(f'Firebase Test: {data}')

# 테스트 데이터 삭제
ref.delete()
print('Firebase connection successful!')
"
```

### 로봇 실행 전 체크리스트

- [ ] Ubuntu 22.04 설치 완료
- [ ] ROS2 Humble 설치 및 소싱 확인
- [ ] 두산 로보틱스 패키지 빌드 성공
- [ ] Python 패키지 설치 완료
- [ ] Firebase 프로젝트 생성 및 키 파일 설정
- [ ] 로봇 제어기 네트워크 연결 (Ping 성공)
- [ ] BeauRo 프로젝트 클론 및 설정 완료
- [ ] Firebase 연결 테스트 성공
- [ ] material_library.yaml 작성 완료
- [ ] 로봇 전원 ON, Servo-On 상태 확인

---

## 부록: 일반적인 설치 문제

### 문제 1: ROS2 패키지를 찾을 수 없음

**증상**:
```
Package 'ros-humble-desktop' has no installation candidate
```

**해결**:
```bash
# GPG 키 재설치
sudo rm /usr/share/keyrings/ros-archive-keyring.gpg
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# 패키지 목록 업데이트
sudo apt update

# 재시도
sudo apt install ros-humble-desktop
```

### 문제 2: Colcon 빌드 실패

**증상**:
```
CMake Error: Could not find CMAKE_ROOT
```

**해결**:
```bash
# CMake 재설치
sudo apt remove --purge cmake
sudo apt install cmake

# 빌드 디렉토리 정리
cd ~/doosan_ws
rm -rf build install log

# 다시 빌드
colcon build --symlink-install
```

### 문제 3: Python 모듈 import 오류

**증상**:
```python
ModuleNotFoundError: No module named 'firebase_admin'
```

**해결**:
```bash
# 시스템 Python 확인
which python3

# pip 경로 확인
which pip3

# 올바른 pip로 재설치
python3 -m pip install --user firebase-admin

# 또는 sudo로 시스템 전역 설치
sudo pip3 install firebase-admin
```

### 문제 4: 네트워크 연결 실패

**증상**:
```
ping: connect: Network is unreachable
```

**해결**:
```bash
# 네트워크 인터페이스 확인
ip link show

# 인터페이스 활성화
sudo ip link set enp3s0 up

# 네트워크 매니저 재시작
sudo systemctl restart NetworkManager

# Netplan 재적용
sudo netplan apply
```

### 문제 5: Firebase 인증 실패

**증상**:
```
google.auth.exceptions.DefaultCredentialsError: File serviceAccountKey.json was not found.
```

**해결**:
```bash
# 파일 존재 확인
ls -l ~/beauro/serviceAccountKey.json

# 파일이 없으면 다시 다운로드
# Firebase Console → 프로젝트 설정 → 서비스 계정

# 권한 확인
chmod 600 ~/beauro/serviceAccountKey.json
```

### 문제 6: ROS2 노드가 로봇을 찾지 못함

**증상**:
```
[WARN] Waiting for service /dsr01/...
```

**해결**:
```bash
# 1. 로봇 제어기 IP 확인
ping 192.168.137.100

# 2. ROS_DOMAIN_ID 설정
export ROS_DOMAIN_ID=30
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc

# 3. DDS 설정 (FastDDS 권장)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc

# 4. 로봇 런처 재실행
ros2 launch dsr_launcher2 single_robot_rviz.launch.py \
  host:=192.168.137.100 \
  mode:=real
```

---

## 다음 단계

설치가 완료되었다면:

1. **좌표 티칭**: [coordinate_teaching.md](coordinate_teaching.md) 참조
2. **첫 실행**: README.md의 "사용 방법" 섹션 참조
3. **문제 해결**: [troubleshooting.md](troubleshooting.md) 참조

---

## 지원

설치 관련 문제가 해결되지 않으면:

- GitHub Issues: https://github.com/rokey-hyunj/beauro_ws/issues
- 이메일: hyunjongkim0524@gmail.com
- 두산 로보틱스 기술지원: https://www.doosan-robotics.com/kr/support/

---

**📝 마지막 업데이트**: 2025-12-11 9
**📌 작성자**: BeauRo Team