# xlerobot-webxr

XLerobot dual-arm VR teleoperation 시스템의 **Mac + WebXR 클라이언트 쪽 전체**.
계획서 [`xlerobot_vr_teleop_plan.md`](../../Workspace/Efforts/coursework/캡스톤디자인(2)/xlerobot_vr_teleop_plan.md)
의 §4.1 (WebXR teleop page), §4.2 (Mac proxy daemon), §8 마일스톤 M0b / M3 / M4 가
이 폴더의 범위.

> 본격 통합(M5+)에는 `indoory_isaac_sim/tools/webxr/` 또는 Mac proxy 리포로 그대로
> 이전 가능하도록 self-contained 로 작성.

## 디렉토리 레이아웃

```
xlerobot-webxr/
├── README.md
├── M0b_dataflow.svg
├── docs/
│   └── production_tls.md  ← 운영 전환용 Caddy + Let's Encrypt DNS-01 메모
├── .gitignore
└── tools/
    ├── webxr/
    │   ├── index.html          ← three.js + WebXR + WebRTC consumer (§4.1)
    │   ├── serve.py            ← M0b 최소 HTTPS 서버 (echo). M3+ 부터는 mac_proxy 사용
    │   └── assets/demo.mp4     ← SMPTE bars + 타임코드, 10s
    ├── run.py                  ★ 단일 launcher — 한 명령으로 mac_proxy + 비디오 publisher 동시 기동
    ├── mac_proxy.py            ← M3+ 정식 데몬 (§4.2 전체 구현)
    ├── fake_producer.py        ← M3 검증용 가짜 Home Server (demo.mp4 loop publisher)
    ├── sim_video_bridge.py     ← indoory_isaac_sim 의 rgb.front.<i> JPEG → WebRTC track
    ├── sim_probe.py            ← Tailscale 연결성 + RPC fleet_info + proprio rate 검증
    ├── ice_probe.py            ← WebRTC ICE host candidate LAN/Tailscale 확인
    ├── fixtures/
    │   └── page_pose_v1_1.json ← page → mac payload smoke fixture
    └── requirements.txt        ← aiohttp, pyzmq, msgpack, aiortc, av, psutil, opencv-python
```

## 포트 / 프로토콜 맵 (§4.2)

| 포트 | 프로토콜 | 누가 쓰나 | 무엇 |
|---|---|---|---|
| `:8443` | HTTPS  | Quest 브라우저 | `GET /` → index.html · `GET /assets/*` · `GET /ws` WSS pose |
| `:8444` | HTTPS  | Quest 브라우저 + Home Server | `GET /signaling/quest` · `GET /signaling/server` (WSS WebRTC signaling) |
| `:7001` | ZMQ PUB tcp | VR bridge (Home Server) | topic `b"pose.<robot_id>"`, payload msgpack(dict) |
| `*`     | WebRTC media | Quest ↔ Mac ↔ Home Server | ICE 협상 후 동적 포트, 양 인터페이스(en0, utun) host candidate |

## 실행 — **한 명령으로** (권장)

```bash
cd ~/Lab/CapstoneDesign/Indory/xlerobot-webxr
python3 -m pip install -r tools/requirements.txt        # 처음 한 번만

# sim @ 100.80.87.68 의 robot 0 head camera 를 영상 소스로 사용 (default)
python3 tools/run.py

# 영상 소스 변경
python3 tools/run.py --sim-robot-id 1                   # 다른 로봇
python3 tools/run.py --sim-topic rgb.wrist              # wrist 카메라
python3 tools/run.py --video-source fake                # sim 안 켰을 때 demo.mp4 loop
python3 tools/run.py --video-source none                # 영상 없이 텔레옵 wire 만
python3 tools/run.py --pose-only                        # aiortc/PyAV 없이 page+/ws+ZMQ pose 만

# 도움말
python3 tools/run.py --help
```

`Ctrl+C` 한 번으로 두 자식 (mac_proxy + 비디오 publisher) 모두 깔끔 종료. 한쪽이
비정상 종료하면 다른 쪽도 같이 멈춤 — half-up 상태로 떠 있는 일 없음.

콘솔 출력은 `[proxy ]` / `[video ]` prefix 가 붙어 두 컴포넌트가 한 화면에 합쳐
보입니다. 메인 페이지 URL 은 `[proxy ]` 첫 블록에 박혀 나옵니다.

## 실행 — 컴포넌트 따로 띄우기 (디버깅용)

각 자식 스크립트는 자기완비라 단독 실행 가능:

### 1. Mac (이 폴더에서)

```bash
cd Indory/xlerobot-webxr

# 권장: 별도 conda env (계획서 §10) — 또는 기존 xlerobot env 활성화
# source ~/.anaconda3/etc/profile.d/conda.sh && conda activate xlerobot

# 'python3 -m pip' 을 쓰는 이유: 어느 python3 가 실행되든 같은 인터프리터에
# 설치되도록 보장. 'pip install' 만 쓰면 셸 PATH 와 conda env 가 갈리는
# 케이스에서 msgpack 같은 패키지를 다른 site-packages 에 깔게 됨.
python3 -m pip install -r tools/requirements.txt        # 처음 한 번만
python3 tools/mac_proxy.py

# 비디오 없이 VR controller teleop만 검증할 때:
python3 -m pip install aiohttp pyzmq msgpack psutil
python3 tools/mac_proxy.py --pose-only
```

기동 로그에 인터페이스 점검이 같이 찍힌다:

```
network interfaces (ipv4):
  en0  : 192.168.0.42
  utun4: 100.x.y.z                ← Tailscale. 없으면 경고 출력
```

`utun*` 미감지 → `sudo tailscale up` 후 재기동. 자기서명 인증서는
`tools/webxr/dev-cert.pem` 에 자동 생성 (M0b 의 `serve.py` 와 공유).

### 2. Quest 3

자기서명 개발 인증서를 쓰는 동안은 포트별 origin 예외를 따로 수락해야 한다.

1. `https://<mac-lan-ip>:8444/` 접속 → signaling 포트 인증서 예외 수락
2. `https://<mac-lan-ip>:8443/?robot=0` 접속 → *Enter AR*
3. 종료할 때는 페이지 overlay 의 *Exit AR* 버튼을 눌러 현재 WebXR session 을 끝낸다.

- `?robot=N` 가 없으면 default 0. 페이지는 첫 WS 메시지로 `{"select_robot": N}` 를 보내고,
  mac_proxy 는 이후 들어오는 모든 pose 페이로드를 토픽 `b"pose.<N>"` 으로 publish.
- Quest 3/3S 에서는 WebXR `immersive-ar` 세션으로 진입한다. 실제 주변 passthrough 위에
  head camera video plane, HUD, controller marker 만 오버레이하며 기존 VR fallback 은 없다.
  `immersive-ar 미지원` 또는 `requestSession(immersive-ar) failed` 가 뜨면 Quest passthrough
  설정, Quest Browser WebXR AR 지원 상태, HTTPS 인증서 예외를 먼저 확인한다.
- video plane 은 (a) WebRTC 트랙이 붙어 있으면 그것을, (b) 없으면 `assets/demo.mp4`
  를 흘려보낸다. HUD 의 오른쪽 작은 라인이 `video=WebRTC live` / `video=demo.mp4 fallback`
  으로 현재 상태를 명시.
- 오른쪽 컨트롤러 `B` 버튼은 HUD 의 `ready` / `E-STOP LATCHED` 상태를 토글한다.
  따라서 ready 조작을 위해 커서를 맞출 필요는 없다.
- head camera plane 이나 아래 HUD/dashboard 를 trigger 로 잡고 움직이면 두 패널이
  하나의 창처럼 같이 이동한다. 이 UI 조작 중 trigger 값은 pose payload 에서는
  `0.0` 으로 보내 gripper close 명령과 겹치지 않게 한다. 조준 디버깅이 필요할 때만
  `?show_cursor=1` 을 붙이면 controller ray/cursor 를 볼 수 있다.

컨트롤러가 없는 개발/시연 환경에서 sim 경로만 확인하려면 명시적으로 dev mode 를 켠다:

```text
https://<mac-lan-ip>:8443/?robot=0&dev_no_controller=1
```

이 모드는 HMD 앞쪽에 synthetic right-hand pose 를 만들고 `right.grip=1.0` 을 계속 보내
clutch-on 상태를 흉내낸다. 실제 로봇에는 쓰지 말고 sim/demo 검증용으로만 사용.
trigger close 경로까지 같이 검증하려면 `dev_trigger` 를 0..1 로 추가한다:

```text
https://<mac-lan-ip>:8443/?robot=0&dev_no_controller=1&dev_trigger=1
```

Home Server 에서 Mac proxy, sim, `examples/vr_teleop_bridge.py` 가 모두 떠 있는 상태라면
Quest 없이 전체 입력 경로를 확인할 수 있다:

```bash
python3 tools/teleop_probe.py \
  --mac-host <mac-tailscale-ip> \
  --sim-host 127.0.0.1 \
  --robot-id 0
```

이 probe 는 synthetic page payload 를 Mac `/ws` 로 보내 `pose.0` ZMQ publish 를 확인하고,
같은 payload 가 Home Server 의 VR bridge 를 거쳐 sim `tf.links.0/gripper_right` 에
표현되는지 측정한다. trigger close 까지 보려면 `--gripper` 를 추가한다:

```bash
python3 tools/teleop_probe.py \
  --mac-host <mac-tailscale-ip> \
  --sim-host 127.0.0.1 \
  --robot-id 0 \
  --gripper \
  --summary-json /tmp/xlerobot-teleop-probe.json
```

`--gripper` 는 sim Jaw 를 직접 한 번 열어 둔 뒤 Mac trigger payload 가 VR bridge 를 통해
Jaw 를 다시 닫는지 측정한다. `direct open` 은 되는데 `trigger close` 가 안 되면 Mac
proxy 가 아니라 Home Server bridge 실행, `estop`, `right.grip`, 또는 trigger→gripper
sign/slot 쪽을 본다. 초기 Jaw 가 이미 닫혀 있으면 trigger close 만으로는 눈에 띄는
변화가 없으므로 gripper 검증은 반드시 먼저 열어 둔 상태에서 해석한다.

실제 Quest Browser/WebXR session 이 이미 Mac proxy 에 붙어 있고 Home Server bridge 가
`--source zmq` 로 실행 중이면, synthetic payload 대신 live pose stream 을 직접 측정한다.
probe 를 실행한 뒤 표시되는 listen window 동안 controller grip 을 누른 채 팔을 움직인다:

```bash
python3 tools/teleop_probe.py \
  --mac-host <mac-tailscale-ip> \
  --sim-host 127.0.0.1 \
  --robot-id 0 \
  --live-quest \
  --listen-s 5 \
  --pose-min-hz 80 \
  --arm-threshold 0.10 \
  --no-gripper \
  --summary-json /tmp/xlerobot-live-quest-acceptance.json
```

이 live mode 는 synthetic `/ws` frame 을 주입하지 않는다. `pose.<robot_id>` ZMQ stream 의
count, observed Hz, Mac-publish-to-subscriber age, schema/robot/frame, 그리고 같은 시간대의
sim `tf.links` 최대 이동량을 기록한다. 최종 Quest acceptance 는 JSON artifact 와 함께
`observed_hz` 가 90Hz 근처인지, `max_move` 가 의도한 controller motion 만큼 나왔는지,
Home Server `stream_status` 가 90Hz control stream 을 유지했는지를 같이 본다.

### 3. 비디오 소스 — 세 가지 중 하나 선택

**(a) 가짜 demo.mp4 loop** (네트워크/Tailscale 미연결 검증용):

```bash
python3 tools/fake_producer.py \
    --url wss://localhost:8444/signaling/server \
    --source tools/webxr/assets/demo.mp4
```

**(b) 진짜 indoory_isaac_sim 의 head camera** (현재 권장 — sim 이 살아있으면 바로 보임):

```bash
# 먼저 sim 살아있는지 검증
python3 tools/sim_probe.py 100.80.87.68

# 살아있으면 rgb.front.<robot_id> 를 받아 WebRTC 트랙으로 publish
python3 tools/sim_video_bridge.py \
    --sim-host 100.80.87.68 --robot-id 0 --topic rgb.front
```

스펙 §4.2.3 그대로 — JPEG (10Hz) → cv2.imdecode (BGR) → av.VideoFrame(bgr24) →
VideoStreamTrack. sim 이 일시적으로 다운돼서 JPEG 가 끊겨도 트랙은 죽지 않고
'waiting for sim head camera...' 검은 프레임으로 fallback (페이지는 그대로 유지).

**(c) Home Server NVENC encoder** (M8):

같은 `/signaling/server` 엔드포인트에 Home Server 가 직접 NVENC offer 를 보내면 끝.
페이지/Mac proxy 양쪽 모두 새로고침 없이 자동으로 트랙이 swap (selective forwarder
의 `replaceTrack` 동작).

## VR bridge 가 받는 ZMQ 페이로드 (§4.2 ZMQ PUB)

mac_proxy 가 페이지의 raw `local-floor` payload 를 그대로 보내지는 않고, robot_id 와
mac 도착 시각을 메타로 붙인 msgpack dict 로 publish.

```python
# bridge 쪽 sub 예시
import zmq, msgpack
ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)
sub.connect("tcp://<mac-tailscale-ip>:7001")
sub.setsockopt(zmq.SUBSCRIBE, b"pose.0")          # default single robot path
while True:
    topic, body = sub.recv_multipart()
    pose = msgpack.unpackb(body, raw=False)
    # pose: {
    #   "schema":    "xlerobot_v1.1.page",
    #   "stamp_ns":  <mac mono ns>,
    #   "robot_id":  0,
    #   "frame":     "local-floor",
    #   "t_page_ms": <page perf.now() ms>,
    #   "hmd":   [x,y,z, qx,qy,qz,qw],            # local-floor frame (RH)
    #   "left":  {"pose":[...] | None, "grip":f, "trigger":f, "buttons":{...}},
    #   "right": { ... },
    #   "estop": bool,
    # }
```

bridge 는 `R_robot_from_xr` (§4.4) 적용 후 anchor/twist-swing/clamp 처리 → sim 으로 PUSH.
Page payload 는 의도적으로 robot 좌표계로 변환되어 있지 않다 — 그 변환은 bridge 책임.

Home Server 에서는 별도의 VR bridge 프로세스가 반드시 떠 있어야 한다. sim 이
`--vr-mode` 로 떠 있어도 bridge 가 `pose.0`을 SUB하지 않으면 로봇팔 command는 생성되지 않는다.

```bash
# Home Server 쪽, examples/vr_teleop_bridge.py 가 있는 repo에서 실행
python examples/vr_teleop_bridge.py \
  --host 127.0.0.1 \
  --source zmq \
  --pose-host <MAC_IP_OR_TAILSCALE_IP> \
  --pose-port 7001 \
  --robot-id 0 \
  -v
```

정상 동작의 최소 조건은 `topic=b"pose.0"`, `robot_id=0`, `estop=false`,
`right.pose != null`, `right.grip > 0.5` 이다. bridge 는 `right.grip` 첫 engaged
tick 에서 anchor 를 잡고, 이후 right controller delta 를 robot base frame 으로 변환해
sim `:5556` PULL 로 `arm_ee_pose_target.right` 를 보낸다.

Home Server bridge 없이 이 repo에서 바로 Quest/WebXR pose를 sim arm target으로
연결하려면 Mac 쪽에서 direct bridge를 같이 띄운다:

```bash
python3 tools/run.py \
  --vr-direct-teleop \
  --video-source none \
  --sim-host 100.80.87.68 \
  --sim-robot-id 0
```

이 모드는 `mac_proxy.py`가 `pose.<robot_id>`를 publish하고,
`tools/vr_direct_teleop.py`가 그 pose를 SUB해서 sim `:5556`으로 직접 PUSH한다.
오른쪽 grip을 0.5 이상 잡은 첫 frame에서 anchor를 캡처하고, 이후 controller 위치
delta를 기본 1:1 scale로 EE target delta로 변환한다. WebXR page의 pose 송신과 direct
bridge의 sim PUSH는 기본 90Hz로 맞춰져 있다. 미세 조작이 필요하면 `--vr-position-scale 0.5`
또는 `tools/vr_direct_teleop.py --position-scale 0.5`로 낮춘다.
Quest 없이 bridge 자체만 확인할 때는 synthetic
pose source로 sim arm 이동량을 바로 측정한다:

```bash
python3 tools/vr_direct_teleop.py \
  --source synthetic \
  --sim-host 100.80.87.68 \
  --robot-id 0 \
  --duration-s 5
```

Quest 없이 키보드로 sim 팔만 움직이려면 Mac 쪽에서 별도 non-VR 웹 서버를 띄운다.
이 경로는 `tf.links.<robot_id>`에서 최신 left/right gripper pose를 읽고, 브라우저의
W/S/R/F jog 입력을 `xlerobot_v1.1 arm_ee_pose_target`으로 변환해 sim `:5556`으로
직접 PUSH한다. sim 쪽 IK target slot을 안정적으로 먹이기 위해 매 command마다
선택한 팔 target과 반대쪽 팔 hold target을 같이 보낸다:

```bash
python3 tools/run.py \
  --keyboard-teleop \
  --sim-host 100.80.87.68 \
  --sim-port 5555 \
  --sim-pull-port 5556 \
  --sim-rep-port 5557 \
  --sim-robot-id 0
```

브라우저에서 `http://127.0.0.1:8765/?robot=0`을 열면 키보드 입력만으로 오른팔
EE target이 움직인다. 기본 tap nudge는 `step=0.10`m이고, 키를 누르고 있으면
브라우저 key-repeat을 기다리지 않고 `speed=0.80`m/s로 매 frame nudge를 누적한다.
W/S는 위/아래, R/F는 앞/뒤 EE target nudge, A/D는 shoulder pan, Z/X는 gripper
delta다. EE nudge는 `tf.links.<robot_id>`에서 잡은 anchor 기준 offset으로 누적되고,
기본 `--max-offset 0.80`m 범위와 sim EE reach sphere 안쪽으로 clamp된다.
Home Server decoder는 이후 URDF joint-limited workspace projection을 한 번 더 적용하므로,
sphere 안쪽이지만 실제로는 닿지 않는 target은 reachable xyz로 투영된다. realtime 경로는
operator가 민 방향을 최대한 보존하고, 같은 방향으로 충분히 전진하는 direct solve가 있으면
`best_effort_directional` projection으로 물리 workspace 경계까지 보낸다.
현재 joint seed가 local minimum에 걸리면 Home Server가 cached FK seed cloud 기반
multi-start refinement를 수행하므로, 이 repo의 browser/Mac 도구는 별도 IK solver를
두지 않는다. Home Server action term은 같은 position-only solver의 joint solution을
articulation target으로 적용하므로, browser/Mac 쪽은 anchor-relative EE target을 계속
보내는 책임만 가진다. Home Server VR-mode는 arm self-collision을 꺼서 SRDF상 허용된
arm fold가 USD/PhysX collision에 막히지 않게 한다.
`tools/run.py --keyboard-teleop`는 `--sim-rep-port`도 전달하므로 custom sim port
세트에서도 `tf.links`/`proprio` feedback stream을 90Hz로 올리는 RPC가 올바른 서버에
도달한다. 필요하면 `--keyboard-feedback-rate-hz 0`으로 이 자동 설정을 끌 수 있다.
브라우저 전송 loop는 `requestAnimationFrame`으로 계속 돌며 `hz` query 값으로만
throttle한다. 기본 `hz=90`이라 Quest/WebXR 90Hz cadence와 맞고, 60Hz 화면에서는
display frame rate까지 자연스럽게 내려간다. 물리 키보드의 첫 `keydown`과 `keyup`은
rAF tick을 기다리지 않고 즉시 WebSocket command를 보낸다. 입력 직후 active motion은 최신
EE target을 보내지만, zero-motion/idle 상태는 full EE target을 반복 송신하지 않고 stateless
hold heartbeat를 보낸다. 이렇게 해야 overreach target을 90Hz로 반복 projection하지 않고,
Home Server의 same-batch merge가 직전 tap nudge를 hold heartbeat에 의해 잃지 않는다. sim은
PULL 큐를 최신 명령 위주로 처리하고, active nudge/held-key command는 최신 target을 즉시
반영한다. hold/prime heartbeat는 source metadata를 붙이지 않으므로 active motion이 끝난 뒤
keyboard teleop lease를 계속 갱신하지 않는다. active lease가 살아있는 동안에는 zero-hold
heartbeat도 생략하고, 이후 background hold heartbeat는 최대 5Hz로 제한된다.
버튼은 JS pointer/keyboard handler가 실패해도 같은 동작을 `POST /api/nudge`
form fallback으로 보낸다. `/api/nudge`는 첫 command를 즉시 보낸 뒤 HTTP 응답을
반환하고, 이후 background loop는 low-rate source-free hold heartbeat만 맡는다. 이 경로는
WebXR session, Quest, WebRTC video, VR bridge를 시작하지 않는다.

latency를 분리해서 보려면 HTTP fallback은 `tools/keyboard_latency_probe.py`, 실제
physical-key WebSocket 경로는 `tools/keyboard_ws_latency_probe.py`로 측정한다. 최신
Home Server가 `tf.links.<robot_id>.vr_decode_debug`를 publish하면 두 probe는 sim-side
workspace projection 요약도 함께 출력한다. `cmd_echo_stamp_ns`가 있으면
`first_cmd_echo_ms`도 출력하므로 command publish latency와 첫 EE motion latency를 분리해서
볼 수 있다. `keyboard_ws_latency_probe.py --hold-s 0.5 --speed 0.8`처럼 실행하면
브라우저에서 키를 누르고 있는 동안의 rAF nudge cadence와 누적 delta도 확인할 수 있다.
이 held-key summary는 requested movement와 decoder-projected target movement를 분리해서
출력하므로, workspace 경계에서 정상적으로 잘린 경우와 실제 tracking underreach를 구분한다.
테스트는 embedded browser script의 rAF callback도 실행해서, 키를 누르고 있는 동안
OS key-repeat 없이 `speed * dt` nudge가 계속 생성되는지 검증한다. keyup 이후에는 한 번의
zero edge만 보내고, rAF loop가 90Hz idle zero frame을 계속 보내지 않는다.
keyboard/VR direct PUSH 경로는 freshness 우선으로 `SNDHWM=1`, `SNDTIMEO=0`,
`IMMEDIATE=1`, non-blocking send를 사용한다. mac_proxy의 `pose.<robot_id>` ZMQ PUB도
`SNDHWM=1`, `SNDTIMEO=0`, non-blocking send라 Quest controller pose가 밀릴 때 오래된
pose를 쌓지 않는다. sim PULL 또는 pose subscriber 쪽이 순간적으로 밀리면 오래된 frame을
기다리지 않고 drop하고 다음 tick의 최신 target/pose를 보낸다.
`tools/keyboard_teleop.py`와 `tools/vr_direct_teleop.py`는 시작 시 Home Server RPC로
`tf.links.<robot_id>`와 `proprio.<robot_id>`를 기본 90Hz로 올린다. sim default profile의
30Hz feedback cadence를 의도적으로 유지하려면 `--feedback-rate-hz 0`을 넘긴다.
그래도 90Hz처럼 보이지 않으면 Home Server의 `stream_status` RPC에서
`observed_loop_hz`, `avg_env_step_ms`, topic별 `rate_hz_target`을 먼저 확인한다.
과거에는 `RateLimiter`가 작은 sleep overshoot를 매번 missed tick으로 처리해 nominal
90Hz 서버가 실제 45Hz로 도는 문제가 있었다.
Camera-enabled teleop 검증은 Home Server를 `--profile teleop --rate-hz 90 --enable_cameras`
로 띄우고 봐야 한다. 이 profile은 `tf.links`/`proprio` 90Hz, RGB 15Hz, depth/lidar disabled
구성이라 키보드/WebXR control feedback을 먼저 보장한다.

같은 `robot_id`에 여러 teleop/bridge 프로세스나 오래 열린 브라우저 탭이 동시에
명령을 보내면 Home Server의 command-source lease arbitration이 lower/same-priority
writer를 drop한다. 그래도 지연처럼 보이거나 움직임이 끊기면 먼저
`pgrep -fl 'keyboard_teleop|vr_direct_teleop|tools/run.py'`로 중복 writer를 확인하고,
Home Server `command_status`에서 `active_source_id`와 `last_rejected_reason`을 본다.

동일 서버를 단독 스크립트로 직접 띄우려면:

```bash
python3 tools/keyboard_teleop.py \
  --sim-host 100.80.87.68 \
  --robot-id 0
```

웹 서버까지 포함한 입력 경로를 CLI에서 확인하려면:

```bash
curl -X POST http://127.0.0.1:8765/api/nudge \
  -d code=KeyW
```

웹 입력 경로를 건너뛰고 sim arm이 ZMQ command로 움직이는지만 바로 확인하려면:

```bash
python3 tools/sim_nudge.py \
  --sim-host 100.80.87.68 \
  --robot-id 0 \
  --side right \
  --dx 0.02 \
  --dz -0.015
```

EE target이 실제 `tf.links` pose로 얼마나 정확히 수렴하는지 확인하려면 waypoint
검증 시나리오를 실행한다. 현재 gripper pose를 anchor로 잡고 `x/y/z` 단독 이동,
center 복귀, x-z 대각 이동을 순회하며 각 target의 최종 오차와 최소 오차를 CSV/JSONL로
남긴다:

```bash
python3 tools/ee_target_verify.py \
  --sim-host 100.80.87.68 \
  --robot-id 0 \
  --side right \
  --settle-s 4.0 \
  --observe-s 1.0 \
  --tolerance 0.012 \
  --out tools/reports/ee_target_verify_right_full
```

`status=fail`인 row는 IK가 해당 target에 tolerance 안으로 수렴하지 못한 지점이다.
`target_xyz`, `final_xyz`, `final_error_m`, `first_move_s`를 보면 어떤 방향/좌표에서
오차가 나는지 바로 재현할 수 있다.

## M3 통과 기준 (계획서 §8 row "3")

| # | 검증 항목 | 확인 방법 |
|---|---|---|
| (a) | Mac proxy 가 양 인터페이스 host candidate publish | `mac_proxy.py` 기동 로그에 en0 + utun 둘 다 enumerate. `fake_producer.py` 가 동일 호스트에서 붙고 connection state 가 `connected` |
| (b) | mac ↔ server end-to-end <10ms (가짜 producer) | `fake_producer.py` 가 mac_proxy 와 localhost 통신 시 트랙 도착까지 1초 미만, RTP 흐름 안정 |
| (c) | `?robot=N` 라우팅 | Quest 페이지에서 `?robot=2` 로 접속 후 mac_proxy 로그에 `pose ws[xxx] robot_id 0 -> 2`. `zmq` SUB 로 `b"pose.2"` 토픽 확인 |
| (d) | pose ZMQ PUB 90Hz | `tools/teleop_probe.py --live-quest --listen-s 5 --pose-min-hz 80 --summary-json ...` 또는 synthetic mode 로 `observed_hz` 확인 |

## M0b 통과 기준 (§8 row "0b")

(이미 통과되어 있다고 가정. mac_proxy 가 켜져 있어도 M0b 의 세 항목은 그대로 동작 —
WebRTC 안 붙으면 demo.mp4 가 fallback 으로 흐른다.)

| # | 검증 항목 | 확인 방법 |
|---|---|---|
| (i)   | immersive-ar 세션 진입 | Enter AR → 실제 주변 passthrough 위에 video plane 보임 |
| (ii)  | `<video>` 동시 렌더링 | video plane 에 SMPTE bars + 타임코드가 끊김 없이 흐름 |
| (iii) | B/Y/A/X 가 page JS 에 도달 | HUD 의 버튼 row 점등. 오른쪽 B → ready / E-STOP 토글. Y → 빨간 E-STOP |

## 페이지 → mac → bridge wire 흐름 (한 장)

[`M0b_dataflow.svg`](./M0b_dataflow.svg) 참조. M3+ 에서는 점선 → 실선으로 활성화되는
것들이 바로 이 문서의 §4.2 구현이다.

## 버튼 매핑 (Quest Touch Plus, §4.1)

| 입력 | Gamepad index | 동작 |
|---|---|---|
| Trigger | `buttons[0]` 아날로그 | gripper close (`right.trigger` / `left.trigger`). WebXR panel 을 가리킨 상태에서 누르면 UI drag 로 사용하고 payload trigger 는 `0.0` 으로 억제 |
| Grip    | `buttons[1]` 아날로그 | clutch hold (right/left 독립) |
| A (R) / X (L) | `buttons[4]` digital | anchor 강제 리셋 |
| B (R) | `buttons[5]` digital | `ready` / **E-STOP LATCHED** 토글 |
| Y (L) | `buttons[5]` digital | **E-STOP latched** |
| Menu (system) | `buttons[6]` if exposed | 계획서 목표: 1초 long-press 로 **E-STOP release** |
| Thumbstick click | `buttons[3]` digital | 현재 구현: M0b/M4 dev fallback release. 별도 `estop_release` edge 필드는 보내지 않음 |

현재 구현은 top-level `estop` latch 만 wire payload에 싣는다. 오른쪽 B는
`estop=true/false` 를 토글하고, 왼쪽 Y는 `estop=true` 로 latch 한다. menu long-press가
실제 런타임에서 노출되거나 thumbstick dev fallback을 누르면 `estop=false`로 복귀한다.
bridge는 이 false 복귀를 release로 해석해야 한다.

## 송출 WebSocket 페이로드 (§4.1 schema)

```jsonc
// 첫 메시지 (한 번):
{"select_robot": 2}

// 이후 매 frame (90Hz):
{
  "t": <ms>,                                // page perf clock
  "hmd": [x,y,z,qx,qy,qz,qw],               // local-floor frame, RH, m
  "left": {
    "pose": [x,y,z,qx,qy,qz,qw] | null,
    "grip": 0.0..1.0,
    "trigger": 0.0..1.0,
    "buttons": {"a":0,"b":0,"x":0,"y":0,"thumb":0,"menu":0}
  },
  "right": { /* same shape */ },
  "estop": false
}
```

## serve.py vs mac_proxy.py — 언제 무엇을 띄우나

- `serve.py` — M0b 검증 전용. WebRTC / ZMQ 없이 페이지 + /ws echo 만. 의존성 `aiohttp` 1개.
- `mac_proxy.py` — M3 이후 본 운영. 페이지 + /ws→ZMQ + WebRTC SFU + ICE relay 통째. 의존성은 `requirements.txt`.
- `mac_proxy.py --pose-only` — Quest pose/trigger/grip teleop만 필요할 때. WebRTC
  signaling/video를 끄므로 `aiohttp`, `pyzmq`, `msgpack`, `psutil`만 있으면 된다.

두 스크립트는 같은 `tools/webxr/dev-cert.pem` 자기서명 인증서를 공유. mac_proxy
가 켜져 있으면 serve.py 는 띄울 필요 없음.

## 운영 환경(§10) 전환 체크리스트

- 인증서: 자기서명 → Caddy + Let's Encrypt DNS-01. mac_proxy 는 `--port 8443` 을
  localhost-bind 로 바꾼 뒤 Caddy reverse proxy 받는 형태로 변경. 자세한 경로는
  [`docs/production_tls.md`](./docs/production_tls.md).
- ZMQ PUB bind: `0.0.0.0` → Tailscale 인터페이스 IP 로 명시 (`tcp://100.x.y.z:7001`).
- 방화벽: Mac 의 8443/8444 인바운드는 LAN, 7001 인바운드는 Tailnet 만 허용.
- WebRTC ICE: STUN 추가 (`--stun stun:stun.l.google.com:19302`) 가 필요한지 §6.2
  의 latency budget 결과로 결정.

## 디버깅 팁

- 인터페이스 점검: `python3 tools/mac_proxy.py --log-level DEBUG` → 기동 시 ipv4 enumerate 출력.
- ICE candidate 보기: `python3 tools/ice_probe.py` 로 LAN/Tailscale host candidate 확인.
  Tailscale이 없는 개발 환경에서 도구 자체만 확인하려면
  `python3 tools/ice_probe.py --allow-missing-tailscale`.
- pose 토픽 sniff: `python3 -c "import zmq,msgpack;c=zmq.Context();s=c.socket(zmq.SUB);s.connect('tcp://localhost:7001');s.setsockopt(zmq.SUBSCRIBE,b'');import sys;t,b=s.recv_multipart();print(t,msgpack.unpackb(b))"`
- 영상 codec: `ffprobe tools/webxr/assets/demo.mp4` — H.264 baseline 3.1 yuv420p.
- WebXR profile/버튼 매핑 검증: `https://immersive-web.github.io/webxr-samples/input-profiles.html` 같은 페이지를 Quest Browser 에서 별도로 띄워 확인.
