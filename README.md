# iCIR MuJoCo + ROS 2 Humble

연구실 구성원이 **Ubuntu 22.04와 ROS 2 Humble**에서 모바일 양팔 로봇 시뮬레이터를 설치하고, 주행·SLAM·Nav2·MoveIt·집기 및 운반 기능을 확인하기 위한 안내입니다.

MuJoCo가 Vic Pinky 이동 베이스와 OpenARM 양팔의 물리 시뮬레이션을 담당하고, ROS 2를 통해 Nav2와 MoveIt에 연결됩니다. 기본 테스트에는 실제 로봇이나 API 키가 필요하지 않습니다.

## 1. 환경과 작업공간

| 항목 | 기준 |
|---|---|
| OS | Ubuntu 22.04 LTS |
| ROS | ROS 2 Humble, 시스템 패키지 설치 |
| Python | 시스템 Python 3.10으로 만든 `venv` (`--system-site-packages`) |
| 물리 엔진 | MuJoCo 3.6.0 |
| 화면 | 로컬 데스크톱에서 MuJoCo 뷰어와 RViz 실행 |
| 셸 | Bash |
| 브랜치 | `humble` |

이 문서의 Python 버전 목록은 기존 `~/mobile_openarm_ws/.venv-mobile-openarm`에서 확인한 설치 환경을 기준으로 합니다. **새 PC에 그 폴더가 없어도 아래 절차로 설치할 수 있습니다.**

ROS 2가 아직 설치되지 않았다면 먼저 [공식 Humble 설치 안내](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)에 따라 Ubuntu 22.04용 `ros-humble-desktop`을 설치하세요. ROS 바이너리와 Python 버전을 맞추기 위해 `/usr/bin/python3`으로 venv를 만듭니다. [ROS의 Python 환경 안내](https://docs.ros.org/en/humble/How-To-Guides/Using-Python-Packages.html)

이후 경로는 다음과 같이 통일합니다. **빌드는 작업공간 루트**, 소스와 테스트 스크립트는 **저장소 폴더**에 있습니다.

```text
~/icir_mujoco_nav2/                      # 작업공간 루트
├── .venv-mobile-openarm/               # Python 가상환경
├── requirements-humble-venv.txt        # 아래에서 작성할 버전 목록
├── src/
│   └── icir_mujoco_nav2/               # 이 Git 저장소
│       ├── src/                       # ROS 패키지 10개
│       ├── sim_config.yaml
│       ├── scripts/
│       ├── lessons/
│       └── tests/
├── build/                             # 빌드 후 생성
├── install/
└── log/
```

> 저장소에 남아 있는 `scripts/install_lecture.sh`, `scripts/mobile_openarm`, `scripts/env.sh`는 Jazzy 전용 설정을 포함합니다. Humble에서는 이 문서의 `python -m colcon`, `ros2 launch`, `ros2 run` 명령을 사용하세요. 기존 `docs/`·`lessons/`의 설치 설명도 Jazzy 기준인 경우가 있습니다.

## 2. 시스템 의존성 설치와 소스 준비

다른 ROS 배포판이나 Conda 환경이 활성화되지 않은 새 터미널에서 시작합니다.

```bash
source /opt/ros/humble/setup.bash

echo "$ROS_DISTRO"          # humble
/usr/bin/python3 --version  # Python 3.10.x

sudo apt update
sudo apt install -y \
  git build-essential cmake python3-dev python3-venv python3-pip \
  python3-colcon-common-extensions python3-rosdep python3-pytest \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-moveit \
  ros-humble-xacro ros-humble-joint-state-publisher-gui \
  ros-humble-teleop-twist-keyboard ros-humble-vision-msgs \
  ros-humble-tf2-geometry-msgs ros-humble-rmw-fastrtps-cpp \
  libgl1-mesa-dri libglfw3
```

처음 내려받는 경우:

```bash
mkdir -p ~/icir_mujoco_nav2/src
cd ~/icir_mujoco_nav2/src
git clone --branch humble https://github.com/iCIRLab/icir_mujoco_nav2.git
```

이미 위 경로에 저장소가 있다면 clone은 생략하고, 저장소에서 `git branch --show-current`로 `humble` 브랜치인지 확인합니다.

패키지에 선언된 ROS 의존성을 설치합니다.

```bash
cd ~/icir_mujoco_nav2

# rosdep을 처음 설정하는 PC에서만 초기화
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y \
  --skip-keys ament_python
```

`ament_python`은 위에서 설치한 colcon의 Python 빌드 지원을 사용합니다. 나머지 의존성 설치가 실패하면 오류에 표시된 패키지를 해결한 뒤 다음 단계로 진행합니다.

## 3. Python 가상환경과 버전 고정

`--system-site-packages`를 사용하면 시스템에 설치된 `rclpy` 등 ROS Python 패키지와 venv의 MuJoCo·NumPy를 함께 사용할 수 있습니다.

```bash
cd ~/icir_mujoco_nav2
/usr/bin/python3 -m venv --system-site-packages .venv-mobile-openarm
source .venv-mobile-openarm/bin/activate
touch .venv-mobile-openarm/COLCON_IGNORE
export PYTHONNOUSERSITE=1

cat > requirements-humble-venv.txt <<'REQ'
absl-py==2.5.1
etils==1.13.0
fsspec==2026.9.0
glfw==2.10.2
importlib_resources==7.1.0
mujoco==3.6.0
numpy==1.26.4
opencv-python==4.10.0.84
pillow==12.3.0
pycollada==0.9.3
PyOpenGL==3.1.10
PyYAML==6.0.3
python-dateutil==2.9.0.post0
scipy==1.11.4
trimesh==4.12.2
typing_extensions==4.15.0
REQ

python -m pip install -r requirements-humble-venv.txt
```

이 목록은 기존 venv의 `pip freeze --local` 결과에, 기존 환경이 `~/.local`에서 가져오던 `python-dateutil`과 `typing_extensions`의 버전을 추가한 것입니다. ROS와 빌드 도구 등 시스템 패키지의 버전은 포함하지 않습니다. `PYTHONNOUSERSITE=1`은 개인 계정의 패키지가 섞이지 않도록 하며, 새 PC에서는 위 두 의존성도 venv에 설치합니다.

특히 **NumPy 1.26.4와 OpenCV 4.10.0.84를 유지**하세요. 저장소의 `requirements-mobile-openarm.txt`는 일부 버전 범위가 더 넓고, `requirements-lecture.txt`는 LLM·학습 도구까지 포함하므로 이 기본 환경 설치에는 위 목록을 사용합니다. 기본 카메라는 ArUco와 시뮬레이션의 정답 정보를 사용하며, YOLO·PyTorch·OpenAI 패키지는 필요하지 않습니다.

설치를 확인합니다.

```bash
python - <<'PY'
import sys
import rclpy
import mujoco
import numpy
import scipy
import cv2
print('Python:', sys.executable)
print('MuJoCo:', mujoco.__version__)
print('NumPy:', numpy.__version__)
print('SciPy:', scipy.__version__)
print('OpenCV:', cv2.__version__)
print('ArUco:', hasattr(cv2.aruco, 'ArucoDetector'))
PY
```

Python 경로가 `~/icir_mujoco_nav2/.venv-mobile-openarm/bin/python`이고, MuJoCo `3.6.0`, NumPy `1.26.4`, SciPy `1.11.4`, OpenCV `4.10.0`, ArUco `True`가 출력되어야 합니다.

## 4. 빌드

같은 터미널에서 실행합니다. venv의 Python을 사용해 빌드하고, ROS 인터페이스 생성에도 같은 인터프리터를 지정합니다.

```bash
cd ~/icir_mujoco_nav2
source /opt/ros/humble/setup.bash
source .venv-mobile-openarm/bin/activate
export PYTHONNOUSERSITE=1

python -m colcon build --symlink-install --base-paths src \
  --cmake-args -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python3"

source install/local_setup.bash
ros2 pkg prefix mobile_openarm_bringup
```

`Summary: 10 packages finished`가 나오고 패키지 경로가 `~/icir_mujoco_nav2/install/mobile_openarm_bringup`을 가리키면 빌드와 패키지 선택이 정상입니다. `~/mobile_openarm_ws/install/...`이 나오면 이전 작업공간이 선택된 것이므로 새 터미널에서 다음 환경 설정을 적용하세요.

## 5. 실행 전 공통 환경 설정

**시뮬레이터 터미널과 명령을 보내는 터미널 모두** 아래 블록을 먼저 실행합니다. 다른 작업공간의 `install/setup.bash`가 `.bashrc`에서 자동으로 적용된다면 해당 설정을 확인하세요.

```bash
source /opt/ros/humble/setup.bash
source ~/icir_mujoco_nav2/.venv-mobile-openarm/bin/activate
source ~/icir_mujoco_nav2/install/local_setup.bash

export MOBILE_REPO="$HOME/icir_mujoco_nav2/src/icir_mujoco_nav2"
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=43
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export MOBILE_OPENARM_CONFIG="$MOBILE_REPO/sim_config.yaml"
export MOBILE_OPENARM_MODEL_DIR="$HOME/icir_mujoco_nav2/artifacts/mobile_generated"
cd "$MOBILE_REPO"
```

모든 터미널에서 domain과 RMW를 동일하게 설정합니다. 이 안내는 기존 PC에 설치된 Fast DDS를 사용하고, `ROS_LOCALHOST_ONLY=1`로 같은 PC 안에서 통신하도록 설정합니다. `MOBILE_OPENARM_CONFIG`는 저장소 안의 설정 파일을 찾도록 지정하며, 생성된 MuJoCo 모델은 작업공간의 `artifacts/`에 저장합니다.

다음 기능 테스트는 **한 번에 하나의 시뮬레이터만** 실행합니다. 모드를 바꿀 때는 기존 launch 터미널에서 `Ctrl+C`를 누르고 프로세스가 종료된 뒤 새 명령을 실행하세요. 제어 명령도 하나가 끝난 후 다음 명령을 보냅니다.

## 6. 기본 시뮬레이션과 수동 주행

**터미널 A — MuJoCo와 RViz 실행:**

```bash
ros2 launch mobile_openarm_bringup warehouse.launch.py \
  mode:=drive moveit:=false camera_handler:=none
```

처음 실행할 때는 로봇 모델 생성에 시간이 걸릴 수 있습니다. MuJoCo에 창고와 로봇이 나타나고 RViz에 로봇·센서 정보가 표시되는지 확인합니다.

**터미널 B — ROS 연결 확인:**

```bash
ros2 topic list
ros2 topic echo /clock --once
ros2 topic echo /joint_states --once
ros2 topic hz /scan
```

`/clock`, `/joint_states`, `/odom`, `/scan`, `/tf`가 보여야 합니다. `/clock`이 갱신되고 LiDAR 메시지가 들어오면 시뮬레이터와 ROS 연결이 동작하고 있는 것입니다. `hz` 확인은 `Ctrl+C`로 끝냅니다.

**터미널 B — 키보드 주행:**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.25 -p turn:=0.5
```

키보드 입력은 이 터미널에 포커스를 둔 상태에서 보냅니다. `i`: 전진, `,`: 후진, `j`/`l`: 회전, `k`: 정지입니다. 로봇이 움직이고 RViz의 위치와 LiDAR 표시가 변하는지 확인합니다.

## 7. SLAM으로 지도 만들기

기존 시뮬레이터를 종료한 뒤 **터미널 A**에서 실행합니다.

```bash
ros2 launch mobile_openarm_bringup warehouse.launch.py \
  mode:=slam moveit:=false camera_handler:=none
```

**터미널 B**에서 6절의 키보드 주행 명령으로 창고를 돌아다닙니다. RViz에서 이동에 따라 지도가 확장되는지 확인합니다.

지도를 저장할 때는 로봇을 정지하고, 공통 환경을 설정한 **별도 터미널**에서 실행합니다. SLAM launch는 계속 켜 둡니다.

```bash
mkdir -p ~/icir_mujoco_nav2/maps
ros2 run nav2_map_server map_saver_cli \
  -f "$HOME/icir_mujoco_nav2/maps/warehouse_slam" \
  --ros-args -p use_sim_time:=true
```

`warehouse_slam.yaml`과 지도 이미지 파일이 생성되는지 확인합니다. 저장한 지도를 Nav2에서 사용하려면 다음 절의 launch 명령에 `map:=$HOME/icir_mujoco_nav2/maps/warehouse_slam.yaml`을 추가합니다. 별도 지도 인자를 주지 않으면 저장소에 포함된 창고 지도를 사용합니다.

## 8. Nav2 자율주행

기존 시뮬레이터를 종료한 뒤 **터미널 A**에서 실행합니다.

```bash
ros2 launch mobile_openarm_bringup warehouse.launch.py \
  mode:=nav moveit:=false camera_handler:=none
```

**터미널 B**에서 Nav2가 활성화될 때까지 기다린 뒤 목표를 보냅니다.

```bash
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
ros2 run mobile_openarm_navigation goal 1.0 0.6 0.0
```

두 lifecycle 상태가 `active`여야 합니다. 목표 좌표는 `map` 기준의 `x`, `y`(m), `yaw`(rad)입니다. 로봇이 목표로 이동하고 `NavigateToPose result: status=4 (SUCCEEDED=4)`가 출력되면 성공입니다. RViz의 `Nav2 Goal` 도구로도 목표를 지정할 수 있습니다.

## 9. MoveIt 양팔·그리퍼·Cartesian 이동

기존 시뮬레이터를 종료한 뒤 **터미널 A**에서 실행합니다.

```bash
ros2 launch mobile_openarm_bringup warehouse.launch.py \
  mode:=drive moveit:=true camera_handler:=none
```

**터미널 B — 이름 자세로 양팔 이동:**

```bash
ros2 run mobile_openarm_moveit_config joint_goal both ready
ros2 run mobile_openarm_moveit_config joint_goal both transport
```

각 명령이 `status=4, error_code=1`로 끝나고 양팔이 목표 자세로 이동하는지 확인합니다. `left` 또는 `right`로 한쪽 팔만 선택할 수 있습니다.

**터미널 B — TCP 위치, IK, 직선 이동, 그리퍼 확인:**

```bash
cd "$MOBILE_REPO"
python lessons/05_moveit/ee_control.py where
python lessons/05_moveit/ee_control.py named both ready
python lessons/05_moveit/ee_control.py ik left 0.45 0.20 0.95
python lessons/05_moveit/ee_control.py pose left 0.45 0.20 0.95
python lessons/05_moveit/ee_control.py cartesian left 0 0 -0.05
python lessons/05_moveit/ee_control.py gripper left close
python lessons/05_moveit/ee_control.py gripper left open
python lessons/05_moveit/ee_control.py cartesian left 0 0 0.05
python lessons/05_moveit/ee_control.py named both transport
```

각 명령의 성공을 확인한 뒤 다음 줄을 실행합니다. `ik`는 계산한 관절각만 출력하고, `pose`는 실제 시뮬레이션 팔을 움직입니다. 위치는 `base_footprint` 기준이며 `cartesian`의 인자는 현재 TCP 위치에서의 상대 이동량(m)입니다. 직선 이동은 `followed 100 %` 출력과 화면의 움직임을 확인합니다.

이 테스트는 Humble의 Cartesian 서비스에 없는 속도·가속도 배율 필드를 처리하고, 반환 궤적의 시간을 조정하는 호환 코드도 사용합니다. 주행 테스트로 돌아갈 때는 팔을 `transport` 자세로 복귀시키세요.

## 10. Nav2 + MoveIt 집기·운반 통합 테스트

새 시뮬레이션으로 시작합니다. **터미널 A:**

```bash
ros2 launch mobile_openarm_bringup warehouse.launch.py \
  mode:=nav moveit:=true skills:=true locate:=truth
```

이 모드에서는 기본 카메라 파이프라인을 켭니다. 물체 위치는 시뮬레이터의 정답 좌표를 사용하지만, 작업대 도킹에는 카메라의 ArUco 마커가 필요하므로 `camera_handler:=none`을 추가하지 마세요.

**터미널 B:**

```bash
ros2 lifecycle get /bt_navigator
ros2 action list
ros2 run warehouse_skills pick_place red
```

Nav2가 `active`이고 `/pick_place`, `/move_action`, `/execute_trajectory` 액션이 준비된 뒤 실행합니다. 로봇은 픽업 작업대로 주행하고, 마커로 도킹한 다음 빨간 상자를 집어 적재 작업대에 놓습니다.

결과에서 `PickPlace status=4`와 `success=True`를 확인하고, 화면에서도 상자가 옮겨졌는지 확인합니다. 파란 상자와 노란 상자는 각각 `red` 대신 `blue`, `yellow`를 사용합니다. **각 색상의 독립 테스트는 시뮬레이터를 재시작**해 물체와 로봇 위치를 초기화합니다.

카메라·마커 출력을 확인하려면:

```bash
ros2 topic echo /vision/stats/aruco --once
ros2 topic echo /vision/markers --once
```

마커 검출은 작업대 마커가 카메라에 보일 때 확인할 수 있습니다. 카메라 영상은 시뮬레이터 내부 Python 처리에 사용하므로 `/image_raw` 같은 영상 토픽이 없는 것이 정상입니다.

## 11. 로봇 모델과 자동 검사

**URDF만 확인:** 시뮬레이터를 종료한 상태에서 실행합니다. RViz와 관절 슬라이더로 링크·관절 구성을 확인할 수 있습니다.

```bash
ros2 launch mobile_openarm_description display.launch.py
```

**기본 단위 테스트:** 별도 ROS launch 없이 공통 환경을 설정한 터미널에서 실행합니다.

```bash
cd "$MOBILE_REPO"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest \
  tests/test_mobile_openarm.py tests/test_skills.py -q
```

로봇 모델, 물리·궤적 처리, 도킹 기하 등을 검사합니다. `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`은 시스템 또는 개인 환경의 불필요한 pytest 플러그인이 로드되는 것을 막습니다. 전체 `tests/`에는 선택 기능용 의존성이 필요한 테스트도 있으므로 기본 환경에서는 위 범위를 먼저 확인하세요.

**실행 중인 시스템 통합 검사:** 10절의 전체 launch를 새로 실행하고, 다른 제어 명령이 없는 상태에서 실행합니다. 이 스크립트는 시뮬레이션 로봇을 실제로 움직입니다.

```bash
cd "$MOBILE_REPO"
python scripts/check_mobile_openarm.py --nav \
  --output "$HOME/icir_mujoco_nav2/artifacts/mobile_openarm_integration.json"
```

프로세스의 성공 종료와 결과 JSON을 확인합니다. 이 검사는 PickPlace 전체 동작을 대신하지 않으므로 집기·운반은 10절에서 별도로 확인합니다.

## 12. 실행 설정과 문제 해결

[sim_config.yaml](sim_config.yaml)에서 `profile`, `viewer`, `rviz`, 카메라 크기·FPS 등을 조정할 수 있습니다. 설정 파일 수정은 재빌드 없이 다음 실행에 반영됩니다. launch 인자는 파일의 값보다 우선합니다.

| 상황 | 적용 방법 |
|---|---|
| 화면이 느림 | launch에 `profile:=minimal rviz:=false` 추가 |
| RViz만 끄기 | `rviz:=false` 추가 |
| 카메라가 필요 없는 주행·팔 테스트 | `camera_handler:=none` 추가 |
| 작업대 앞에서 시작 | `spawn:=pick_table` 추가 |
| launch 인자 목록 확인 | `ros2 launch mobile_openarm_bringup warehouse.launch.py --show-args` |

| 증상 | 확인 및 조치 |
|---|---|
| `ModuleNotFoundError: mujoco` | 5절의 venv를 활성화하고 `python -c "import mujoco"` 확인. `echo "$VIRTUAL_ENV"`가 현재 작업공간을 가리키는지 확인 |
| `rclpy` 또는 ROS 메시지 import 실패 | `/opt/ros/humble/setup.bash` 적용, Python 3.10과 `--system-site-packages` 사용 여부 확인 |
| NumPy 관련 바이너리 오류 | venv에서 3절의 버전 목록을 다시 설치하고 인터프리터 확인. 다른 Python으로 빌드한 이력이 있으면 아래 재빌드 절차 사용 |
| 패키지가 없거나 이전 코드가 실행됨 | `ros2 pkg prefix mobile_openarm_bringup` 확인 후 현재 작업공간을 빌드하고 `install/local_setup.bash` 적용 |
| 토픽이 보이지 않음 | 모든 터미널의 `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`, `RMW_IMPLEMENTATION`을 5절과 맞추고 `ros2 daemon stop` 실행 후 재조회 |
| `Source ROS 2 Jazzy first` | 기존 wrapper 대신 이 README의 직접 실행 명령 사용 |
| `simulator is already running` | 이전 launch 터미널에서 `Ctrl+C`로 종료. lock 파일 삭제만으로 실행 중인 프로세스가 종료되지는 않음 |
| 주행 명령에 반응하지 않음 | 팔을 `transport` 자세로 복귀시키고 다른 팔·운반 액션이 실행 중인지 확인 |
| PickPlace 도킹 실패 | 기본 카메라 활성화, 작업대 마커 시야, `/vision/markers` 확인 |
| MuJoCo 창·카메라 렌더링 오류 | 우선 로컬 데스크톱에서 실행하고 OpenGL 드라이버 확인. `viewer:=false`만으로 카메라 렌더링까지 꺼지지는 않음 |

Python 환경 변경 후 CMake 캐시를 갱신해야 한다면 실행 중인 노드를 종료하고, 현재 venv를 활성화한 터미널에서 재빌드합니다.

```bash
cd ~/icir_mujoco_nav2
python -m colcon build --symlink-install --base-paths src --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python3"
source install/local_setup.bash
```

소스나 ROS 패키지 설정을 수정한 후에도 재빌드하고 관련 launch를 다시 시작하세요. 특히 `--symlink-install`이어도 설치 데이터 파일의 반영 여부는 파일 종류에 따라 다를 수 있습니다.

## 검증 범위와 출처

Humble 호환 변경에 대해 이 작업공간에서 ROS 패키지 10개 빌드가 통과했습니다. README 작성 시 위 기본 단위 테스트 17개도 통과했고, launch 인자와 명령 블록의 Bash 문법을 확인했습니다. Python 검사는 기존 venv와 빌드 결과를 사용하되, `~/.local` 전체를 불러오는 대신 위에 명시한 추가 의존성 두 개만 임시 경로에서 제공하여 진행했습니다. 새 PC에서의 전체 설치와 GUI를 포함한 주행·집기·운반 성공은 이 기록에 포함되지 않으며, 각 PC에서 위 기능별 절차로 확인합니다.

이 저장소는 [PinkLAB의 모바일 OpenARM MuJoCo 프로젝트](https://github.com/PinkWink/pinklab_mobile_openarm_ros2_mujoco)를 기반으로 합니다. 로봇 모델과 메시의 라이선스·출처는 `src/openarm_description`, `src/vicpinky_description` 내부 고지를 따릅니다.
