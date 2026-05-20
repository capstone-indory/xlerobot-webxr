# XLeRobot 공식 Web Simulation / IK 동작 정리

작성 기준: 2026-05-21

확인 대상:
- 공식 XLeRobot repo: `Vector-Wangel/XLeRobot` @ `51ca0ec31bdb48713b94bacdba828bf8d889296b`
- 공식 Web MuJoCo repo: `Vector-Wangel/MuJoCo-GS-Web` @ `0d60421c6cd8b16525695f32d2f8c5ba1329d45d`
- 로컬 WebXR repo: `xlerobot-webxr` @ `fa515c5`, branch `xlerobot-local-simulation`
- 로컬 Isaac sim repo: `indory_isaac_sim` @ `ee95a8a`, branch `codex/research-ik-teleop-control`

관련 upstream:
- XLeRobot simulation docs: <https://xlerobot.readthedocs.io/en/latest/simulation/>
- Web simulation repo: <https://github.com/Vector-Wangel/MuJoCo-GS-Web>
- Official web demo: <https://vector-wangel.github.io/MuJoCo-GS-Web/>
- XLeRobot GitHub repo: <https://github.com/Vector-Wangel/XLeRobot>
- WikiDocs WebRTC guide: <https://wikidocs.net/307147>

## 결론

공식 웹 시뮬레이션에서 로봇은 브라우저 안의 MuJoCo WASM 시뮬레이터가 직접 움직인다. 입력은 JavaScript keyboard controller가 받고, controller가 매 프레임 `data.ctrl` actuator target을 갱신한 뒤 `mujoco.mj_step()`이 물리를 진행한다.

XLeRobot 팔 IK는 Jacobian/DLS가 아니라 해석적 2-link planar IK다. `Pitch`와 `Elbow` 두 관절만 IK로 계산하고, shoulder yaw, wrist pitch 보정, wrist roll, gripper, head, mobile base는 별도 규칙으로 actuator target에 직접 반영한다.

우리 `xlerobot-webxr` / `indory_isaac_sim` 경로와는 설계가 다르다. 우리 경로에서는 WebXR page나 Mac proxy가 IK를 풀지 않고, `xlerobot_v1.1 arm_ee_pose_target`을 sim으로 보내며, IsaacLab sim-side `PositionOnlySevenDofIkAction`이 position-only DLS IK를 수행한다.

## 공식 Web Simulation 실행 흐름

공식 XLeRobot 문서는 Web MuJoCo simulation을 `MuJoCo-GS-Web`으로 연결한다. 이 web app은 MuJoCo 3.3.8 WebAssembly binding을 브라우저에서 로드하고, robot XML/assets를 Emscripten virtual filesystem에 올린 뒤 `MjModel` / `MjData`를 만든다.

런타임 루프는 다음 구조다.

```text
browser page
  -> load MuJoCo WASM
  -> load XLeRobot MJCF/XML and assets
  -> KeyboardController.enable("xlerobot")
  -> every render/physics tick:
       XLeRobotController.step(keyStates, model, data, mujoco)
       data.ctrl[...] = target actuator values
       mujoco.mj_step(model, data)
       Three.js / 3DGS scene sync and render
```

중요한 점은 controller가 joint state를 직접 "순간이동"시키지 않는다는 것이다. controller는 MuJoCo actuator control buffer인 `data.ctrl`에 목표값을 쓰고, 실제 관절 움직임은 MuJoCo actuator와 physics step이 만든다.

## 공식 XLeRobot Controller 구조

공식 web controller의 actuator mapping은 16개다.

```text
0  forward       base velocity
1  turn          base angular velocity
2  Rotation_L    left shoulder yaw
3  Pitch_L       left pitch, IK output
4  Elbow_L       left elbow, IK output
5  Wrist_Pitch_L wrist pitch compensation
6  Wrist_Roll_L
7  Jaw_L
8  Rotation_R
9  Pitch_R       right pitch, IK output
10 Elbow_R       right elbow, IK output
11 Wrist_Pitch_R wrist pitch compensation
12 Wrist_Roll_R
13 Jaw_R
14 head_pan
15 head_tilt
```

Keyboard mapping은 web README와 `XLeRobotController.getDescription()` 기준으로 다음과 같다.

```text
Base:      W/S forward, A/D turn
Left arm:  7/Y shoulder rotation, 8/U EE Y, 9/I EE X, 0/O pitch, -/P roll
Right arm: H/N shoulder rotation, J/M EE Y, K/, EE X, L/. pitch, ;/? roll
Gripper:   V/B toggle
Head:      R/T pan, F/G tilt
Reset:     X
```

컨트롤러 내부 상태는 다음 값을 유지한다.

```text
eePos1, eePos2       planar EE target [x, y]
pitch1, pitch2       desired wrist/end-effector pitch adjustment
targetJoints[16]     actuator target values
gripperOpen/cooldown toggle state
```

키 입력으로 `eePos*`가 조금씩 변하면 매 step마다 IK를 다시 계산해 `Pitch_*`, `Elbow_*`, `Wrist_Pitch_*`를 갱신한다.

## 공식 Web IK

공식 web IK 함수는 `inverseKinematics2Link(x, y, l1=0.1159, l2=0.1350)`이다. 입력은 한 팔의 sagittal plane 상 end-effector 목표점 `[x, y]`이고 출력은 URDF joint angle에 맞춘 `[joint2, joint3]`, 즉 `Pitch`와 `Elbow`다.

수식 흐름:

```text
l1 = 0.1159
l2 = 0.1350

theta1Offset = atan2(0.028, 0.11257)
theta2Offset = atan2(0.0052, 0.1349) + theta1Offset

r = sqrt(x^2 + y^2)
r is clamped to [abs(l1 - l2), l1 + l2]

cosTheta2 = -(r^2 - l1^2 - l2^2) / (2 l1 l2)
theta2 = pi - acos(clamp(cosTheta2, -1, 1))

beta = atan2(y, x)
gamma = atan2(l2 sin(theta2), l1 + l2 cos(theta2))
theta1 = beta + gamma

joint2 = clamp(theta1 + theta1Offset, -0.1, 3.45)
joint3 = clamp(theta2 + theta2Offset, -0.2, pi)
```

그 다음 controller가 actuator target을 이렇게 만든다.

```text
compensatedY = eePos.y + TIP_LENGTH * sin(pitch)
[pitchJoint, elbowJoint] = inverseKinematics2Link(eePos.x, compensatedY)

Pitch       = pitchJoint
Elbow       = elbowJoint
Wrist_Pitch = pitchJoint - elbowJoint + pitch
```

따라서 공식 web IK는 6-DoF pose IK가 아니다. 방향 제어는 단순화되어 있다.

- EE 위치 목표는 arm plane의 2D `[x, y]`다.
- `Pitch`와 `Elbow`만 analytical IK 결과다.
- shoulder yaw는 별도 키 입력으로 직접 증감한다.
- wrist pitch는 `Pitch - Elbow + pitchAdjustment` 보정식이다.
- wrist roll은 별도 키 입력으로 직접 증감한다.
- gripper/head/base도 IK와 무관하게 직접 actuator target을 바꾼다.

이 구조는 웹에서 가볍고 예측 가능하지만, 로봇 전체 3D 작업공간의 일반적인 IK solver는 아니다. XLeRobot 팔이 SO-Arm101 계열 5-DoF 구조이기 때문에, 공식 예제는 reachable planar geometry를 빠르게 계산하는 쪽으로 설계되어 있다.

## 공식 ManiSkill / VR 예제와의 관계

공식 XLeRobot repo의 ManiSkill 예제도 같은 계열의 analytical 2-link IK를 사용한다.

- `simulation/Maniskill/examples/demo_ctrl_action_ee_keyboard.py`
- `simulation/Maniskill/examples/demo_ctrl_action_ee_keyboard_single.py`
- `simulation/Maniskill/examples/demo_ctrl_action_ee_VR.py`

Python 예제의 `inverse_kinematics()`도 `l1=0.1159`, `l2=0.1350`, workspace radius clamp, law-of-cosines elbow solve, joint limit clamp를 사용한다. Keyboard 예제는 계산된 target joint와 current joint 차이에 P gain을 곱해 action을 만든다.

VR 예제는 VR controller 위치를 바로 IK solver에 넣는 것이 아니라, VR 위치를 arm 기준 목표로 재매핑한다.

```text
VR target position
  -> scaled x/y/z
  -> horizontal radius r = sqrt(x^2 + z^2)
  -> planar IK input [r, y]
  -> Pitch/Elbow analytical IK
  -> shoulder rotation from atan2(x, z)
  -> wrist/gripper from VR wrist/trigger signals
```

즉 공식 VR 예제도 "VR pose -> 3D 6-DoF IK"가 아니라 "VR pose를 planar target + shoulder yaw + wrist/gripper 값으로 분해"하는 방식이다.

## 공식 repo의 web_control / XLeVR / WikiDocs 구분

공식 XLeRobot repo에는 `web_control/`도 있다. 이것은 앞의 `MuJoCo-GS-Web` 브라우저 시뮬레이션과 다른 스택이다.

```text
React/Vite client
  -> Socket.IO websocket
  -> FastAPI + python-socketio server
  -> RemoteCore
  -> ZMQ PUSH command channel
  -> robot/sim host
  -> ZMQ PULL data/video channel
  -> Socket.IO video_frame / telemetry events
```

공식 `web_control/server/core/protocol.py`의 command type에는 `move`, `stop`, `reset`, `set_arm_joint`, `set_camera_position` 등이 정의되어 있다. 하지만 현재 `web_control/server/main.py`의 Socket.IO UI path는 `move_command`, video stream, camera reset/position 중심이고, `simulation/Maniskill/run_xlerobot_sim_host.py`도 실제 command handler에서 `move`, `reset`, `ping`, `get_state`를 처리한다. 즉 현재 확인한 공식 `web_control` 경로는 IK 문서의 핵심 근거가 아니라 remote dashboard / base movement / camera video 참고자료에 가깝다.

포트 의미도 우리 repo와 바로 호환되지 않는다.

```text
official web_control defaults:
  ROBOT_PORT_CMD=5555   RemoteCore PUSH -> sim host PULL
  ROBOT_PORT_DATA=5556  sim host PUSH -> RemoteCore PULL

local indory_isaac_sim:
  :5555 sensor/data PUB
  :5556 command PULL
```

따라서 공식 `web_control`을 우리 stack에 그대로 붙이면 command/data 방향과 socket type부터 adapter가 필요하다.

공식 XLeVR은 WebRTC가 아니라 HTTPS page + WSS WebSocket 기반이다. `XLeVR/README.md`와 `docs/en/source/simulation/getting_started/vr_sim.md` 기준으로 Quest browser가 HTTPS page를 열고, controller/headset data가 WebSocket server로 들어오며, Python 쪽은 `ControlGoal`을 읽는다.

```text
Quest browser XLeVR page
  -> WSS controller/headset JSON
  -> VRWebSocketServer
  -> ControlGoal(left/right/headset)
  -> ManiSkill VR example
  -> analytical 2-link IK + joint action
```

사용자가 준 WikiDocs `7.3 WebRTC 통신` 페이지는 secondary guide다. 이 페이지는 signaling, ICE candidate, WebRTC data channel, hand pose 60 FPS 전송, adaptive quality, reconnect/ICE restart 같은 일반적인 VR/WebRTC 설계를 설명한다. 공식 XLeRobot 현재 코드의 authoritative implementation은 아니며, IK나 공식 Web MuJoCo 동작의 근거로 쓰면 안 된다.

우리 repo와 비교하면 역할은 이렇게 나뉜다.

- WikiDocs WebRTC guide: WebRTC 설계 아이디어 참고. 특히 연결 상태 모니터링, 품질 조절, 재연결 흐름.
- 공식 XLeVR: WSS 기반 VR controller data capture 참고.
- 공식 `web_control`: Socket.IO dashboard + ZMQ remote host 참고.
- 공식 `MuJoCo-GS-Web`: 브라우저 MuJoCo simulation과 analytical IK의 직접 근거.
- 로컬 `xlerobot-webxr`: WebRTC는 video media에만 사용하고, control pose는 WebSocket/WSS -> ZMQ -> bridge -> sim command로 보낸다.

## 우리 WebXR / Isaac Sim 경로와 다른 점

우리 repo의 page-side preview는 IK solver가 아니라 operator intent preview다. `tools/webxr/index.html`에는 URDF/STL preview와 position-only CCD preview가 있지만, 코드 주석대로 실제 sim-side IK와 physics가 authoritative다.

우리 실제 텔레옵 경로는 다음과 같다.

```text
Quest WebXR page
  -> tools/mac_proxy.py WSS /ws
  -> ZMQ PUB tcp://<mac>:7001 topic pose.<robot_id>
  -> Home Server examples/vr_teleop_bridge.py
  -> xlerobot_v1.1 arm_ee_pose_target
  -> sim PULL tcp://127.0.0.1:5556
  -> command_decode.py
  -> IsaacLab PositionOnlySevenDofIkAction
  -> DifferentialIKControllerCfg(command_type="position", ik_method="dls")
```

좌표계 책임도 다르다.

- WebXR page payload는 `local-floor` frame이다.
- Mac proxy는 robot frame 변환이나 IK를 수행하지 않는다.
- bridge가 controller delta를 robot base frame으로 변환하고, 현재 EE pose와 anchor를 이용해 absolute EE target을 만든다.
- sim decoder가 `arm_ee_pose_target[*].pose`를 Isaac articulation root frame action으로 변환한다.
- IsaacLab action term이 xyz만 DLS IK controller에 넣는다.

로컬 Isaac path의 핵심 구현은 다음과 같다.

- `indory_isaac_sim/src/indoory_isaac_sim/env/build.py`
  - `DifferentialIKControllerCfg(command_type="position", ik_method="dls", ik_params={"lambda_val": ...})`
- `indory_isaac_sim/src/indoory_isaac_sim/env/mdp.py`
  - `PositionOnlySevenDofIkAction`이 7-D wire slot `xyz + quat` 중 `xyz`만 IK controller로 전달한다.
- `indory_isaac_sim/src/indoory_isaac_sim/wire/command_decode.py`
  - `base` 또는 `world` frame EE pose를 articulation root frame action으로 변환한다.
- `indory_isaac_sim/examples/vr_teleop_bridge.py`
  - WebXR pose를 low-pass, anchor, retarget, workspace check 후 `xlerobot_v1.1` command로 pack한다.
- `indory_isaac_sim/src/indoory_isaac_sim/vr_teleop/anchor.py`
  - clutch press 시 controller pose와 current EE pose를 동시에 capture해 press 순간 EE jump를 방지한다.
- `indory_isaac_sim/src/indoory_isaac_sim/vr_teleop/retarget.py`
  - `EE_target = EE_anchor + R * (controller_now - controller_anchor) * scale`
  - controller rotation은 swing/twist로 분해하고, swing은 EE target quaternion, twist는 shoulder_pan relative delta로 보낸다.

여기서 quaternion은 protocol/debug 호환성을 위해 남아 있지만, 현재 sim-side IK는 position-only다. 임의의 6-DoF orientation error를 XLeRobot 5-DoF arm에 동시에 강제하면 position convergence와 충돌할 수 있기 때문에 xyz 수렴을 우선한다.

## 비교 요약

| 구분 | 공식 Web MuJoCo | 공식 ManiSkill/VR 예제 | 로컬 xlerobot-webxr + Isaac |
|---|---|---|---|
| 실행 위치 | 브라우저 | Python ManiSkill | Quest/Mac + Home Isaac sim |
| 입력 | Keyboard | Keyboard 또는 VR | WebXR controller / keyboard client |
| command 대상 | MuJoCo `data.ctrl` | ManiSkill action | `xlerobot_v1.1` ZMQ command |
| IK 위치 | Browser JS controller | Python example script | Isaac sim action term |
| IK 방식 | Analytical 2-link planar IK | Analytical 2-link planar IK | Position-only DLS IK |
| IK 출력 | Pitch, Elbow | Pitch, Elbow | IK-owned arm joint targets |
| orientation 처리 | wrist compensation/direct roll | shoulder yaw + wrist mapping | quaternion carried, xyz-only IK objective |
| physics source of truth | MuJoCo WASM | ManiSkill/Sapien | IsaacLab |

| 구분 | 공식 web_control | 공식 XLeVR | WikiDocs 7.3 |
|---|---|---|---|
| 목적 | Remote dashboard/control server | VR controller data capture | WebRTC architecture guide |
| transport | Socket.IO + ZMQ | HTTPS + WSS WebSocket | WebRTC signaling/data-channel pattern |
| IK 근거성 | 낮음 | ManiSkill VR 예제 입력 쪽 참고 | 낮음 |
| 우리 stack에 줄 수 있는 힌트 | dashboard, rate-limit, telemetry, video frame relay | Quest pose capture, ControlGoal shape | reconnect, ICE, adaptive quality |

## 구현상 의미

공식 웹 시뮬레이션의 느낌을 그대로 재현하려면 browser/Mac 쪽에서 analytical 2-link IK를 풀고 joint target을 보내는 구조가 맞다. 하지만 이 방식은 우리 현재 `xlerobot_v1.1 arm_ee_pose_target` 계약과 다르며, sim-side IK를 source of truth로 둔 장점을 잃는다.

우리 스택에서는 IK를 한 곳에만 둬야 한다. 현재 구조에서는 sim-side IsaacLab DLS IK가 그 한 곳이다. WebXR page의 CCD preview나 공식 web analytical IK를 실제 command path에 추가하면 double IK가 되어 디버깅이 어려워질 수 있다.

공식 web 구현에서 참고할 만한 부분은 다음 정도다.

- planar reach clamp: target radius를 `[abs(l1-l2), l1+l2]`로 제한
- 단순하고 눈에 보이는 keyboard mapping
- wrist pitch compensation의 명확한 규칙
- IK-controlled actuator slider를 UI에서 비활성화하는 UX

반대로 우리 stack에서 유지해야 할 원칙은 다음이다.

- WebXR/Mac은 pose transport와 observability에 집중한다.
- bridge는 anchor-relative target 생성과 workspace check를 담당한다.
- 실제 IK solver는 sim-side action term 하나로 유지한다.
- command frame, 좌표계, source ownership, stale/e-stop behavior를 로그와 telemetry로 드러낸다.

## 확인한 주요 파일

공식 Web MuJoCo repo:

- `src/main.js`
- `src/mujocoUtils.js`
- `src/utils/KeyboardControl.js`
- `src/utils/controllers/XLeRobotController.js`
- `src/utils/controllers/BaseController.js`
- `src/utils/math/inverseKinematics.js`
- `README.md`

공식 XLeRobot repo:

- `docs/en/source/simulation/index.md`
- `docs/en/source/simulation/getting_started/vr_sim.md`
- `web_control/README.md`
- `web_control/server/core/config.py`
- `web_control/server/core/protocol.py`
- `web_control/server/core/remote_core.py`
- `web_control/server/main.py`
- `web_control/server/api/streaming.py`
- `web_control/client/src/hooks/useSocket.ts`
- `web_control/client/src/config/constants.ts`
- `simulation/Maniskill/run_xlerobot_sim_host.py`
- `simulation/mujoco/xlerobot_mujoco.py`
- `simulation/Maniskill/examples/demo_ctrl_action_ee_keyboard.py`
- `simulation/Maniskill/examples/demo_ctrl_action_ee_keyboard_single.py`
- `simulation/Maniskill/examples/demo_ctrl_action_ee_VR.py`
- `simulation/Maniskill/agents/xlerobot/xlerobot.py`
- `XLeVR/xlevr/inputs/base.py`
- `XLeVR/xlevr/inputs/vr_ws_server.py`
- `XLeVR/README.md`
- `XLeVR/web-ui/vr_app.js`

로컬 repo:

- `README.md`
- `tools/webxr/index.html`
- `tools/vr_direct_teleop.py`
- `tools/keyboard_teleop.py`
- `tools/ee_target_verify.py`

로컬 Isaac sim repo:

- `docs/mac_vr_arm_protocol.md`
- `docs/research_ik_teleop_stability.md`
- `src/indoory_isaac_sim/wire/schema.py`
- `src/indoory_isaac_sim/wire/command_decode.py`
- `src/indoory_isaac_sim/env/build.py`
- `src/indoory_isaac_sim/env/mdp.py`
- `src/indoory_isaac_sim/vr_teleop/anchor.py`
- `src/indoory_isaac_sim/vr_teleop/retarget.py`
- `examples/vr_teleop_bridge.py`
