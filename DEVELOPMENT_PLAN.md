# xlerobot-webxr 개발 계획서

작성일: 2026-05-15

## 기준 문서

- [`mac_vr_arm_protocol.md`](https://github.com/capstone-indory/indory_isaac_sim/blob/main/docs/mac_vr_arm_protocol.md)
- [`xlerobot_vr_teleop_plan.md`](https://github.com/capstone-indory/indory_isaac_sim/blob/main/xlerobot_vr_teleop_plan.md)

이 문서는 위 두 문서와 현재 `xlerobot-webxr` repo를 대조한 뒤, Mac + Quest WebXR 쪽에서
수정하거나 검증해야 할 작업을 실행 순서로 정리한다.

## 범위

이 repo의 책임은 다음 네 가지다.

1. Quest Browser용 WebXR 페이지를 HTTPS로 제공한다.
2. WebXR pose와 버튼 상태를 WSS로 받아 ZMQ PUB `pose.<robot_id>`로 relay한다.
3. Home Server 쪽 비디오 producer와 Quest consumer 사이의 WebRTC signaling 및 selective forwarding을 담당한다.
4. 개발 중에는 `sim_video_bridge.py`와 `fake_producer.py`로 비디오 경로를 단독 검증한다.

Mac/Quest 쪽에서는 Isaac Sim, IsaacLab, torch, IK solver를 import하지 않는다. 좌표계 변환, anchor,
twist-swing 분해, workspace clamp, `xlerobot_v1.1` command packing은 Home Server의 VR bridge 책임이다.

## 현재 상태 요약

- `tools/mac_proxy.py`는 `:8443` 페이지 + pose WSS, `:8444` WebRTC signaling, `:7001` ZMQ PUB 구조를 이미 갖고 있다.
- `tools/webxr/index.html`은 `local-floor` 기준 HMD/controller pose, Grip/Trigger/A/B/X/Y/thumb 상태, WebRTC video fallback을 송출한다.
- `tools/run.py`는 `mac_proxy.py`와 video publisher를 한 번에 띄우는 launcher 역할을 한다.
- `tools/sim_video_bridge.py`는 `rgb.front.<robot_id>` JPEG ZMQ stream을 WebRTC track으로 변환한다.
- `tools/sim_probe.py`는 sim `:5555`, `:5556`, `:5557` 연결성을 검증한다.

## P0 수정 항목

### 1. `robot_id` 기본값을 0으로 통일

문제:

- 기준 문서와 sim smoke path는 단일 로봇 기본값으로 `robot_id=0`을 사용한다.
- 현재 WebXR page와 `mac_proxy.py`는 기본값이 `1`이다.
- 특히 page는 `?robot=0`을 `1`로 되돌리므로, 비디오는 `rgb.front.0`을 보면서 pose는 `pose.1`로 나가는 불일치가 생길 수 있다.

수정:

- `tools/webxr/index.html`
  - 기본 query fallback을 `"0"`으로 변경한다.
  - `v > 0` 조건을 `v >= 0`으로 변경해 `?robot=0`을 허용한다.
  - 주석과 HUD 설명의 기본값을 `0`으로 맞춘다.
- `tools/mac_proxy.py`
  - `DEFAULT_ROBOT_ID = 0`으로 변경한다.
  - 기동 로그의 Quest URL 예시를 `?robot=0`으로 변경한다.
- `README.md`
  - Quest 접속 예시와 M3 검증 예시를 `robot=0` 중심으로 정리한다.

검증:

```bash
python3 -m py_compile tools/*.py tools/webxr/serve.py
python3 tools/run.py --help
python3 tools/_smoke_test.py
```

수동 검증:

- Quest URL `https://<mac-lan-ip>:8443/?robot=0` 접속
- `mac_proxy.py` 로그에서 `robot_id=0`
- ZMQ subscriber로 `b"pose.0"` topic 수신 확인

### 2. e-stop release 정책을 명시적으로 결정

문제:

- 계획서는 `B/Y`로 e-stop latch, `menu long-press 1s`로 release를 정의한다.
- 현재 페이지는 Quest에서 menu가 OS에 잡힐 수 있다는 이유로 thumbstick click release fallback을 구현했다.
- 계획서에서는 thumbstick을 M9+ base teleop 예약으로 두고 있어 장기적으로 충돌 가능성이 있다.

결정안:

- M0b/M4 단계에서는 현재 thumbstick fallback을 유지하되, wire payload에 `estop_release` 같은 별도 edge 필드를 추가하지 않는다.
- VR bridge가 `estop=false` 복귀를 release로 해석할 수 있는지 확인한다.
- 실제 Quest에서 menu button 접근 가능성이 확인되면 menu long-press를 우선 release 경로로 추가하고, thumbstick은 dev fallback으로만 둔다.

수정:

- `tools/webxr/index.html`
  - 현재 thumbstick release가 임시 fallback임을 HUD/log/주석에서 명확히 표시한다.
  - menu가 실제로 노출될 경우를 대비해 long-press 상태 추적 구조를 추가할 수 있게 버튼 처리 함수를 분리한다.
- `README.md`
  - "현재 구현"과 "계획서 목표"를 분리해서 적는다.

검증:

- Quest Touch Plus에서 B/Y/A/X/thumb 이벤트가 HUD에 들어오는지 확인한다.
- e-stop latch 후 release 시 pose payload의 top-level `estop`이 `false`로 복귀하는지 확인한다.

## P1 수정 항목

### 3. `sim_video_bridge.py --fps`가 실제 frame pacing에 반영되게 수정

문제:

- `SimCameraTrack`은 `fps` 값을 저장하지만 `recv()`에서 aiortc 기본 `next_timestamp()`를 사용한다.
- 결과적으로 `--video-fps` 옵션이 실질적으로 30fps 고정으로 동작할 가능성이 높다.
- 계획서 M8 목표는 60fps head camera WebRTC track이다.

수정:

- `SimCameraTrack.recv()`에서 직접 pacing을 구현한다.
- `self._fps`에 따라 PTS와 sleep interval을 계산한다.
- 기본값은 현재 안정성을 위해 30fps로 유지하되, `--video-fps 60`이 실제로 60fps 송출되게 한다.

검증:

```bash
python3 tools/_smoke_test_sim_bridge.py
python3 tools/run.py --video-source sim --video-fps 60
```

수동 검증:

- Quest HUD에서 WebRTC live 전환 확인
- bridge 로그에서 RTP frame drain rate 확인
- CPU 사용량과 latency가 과도하지 않은지 확인

### 4. ICE gather timeout 로그 포맷 수정

문제:

- `tools/mac_proxy.py`의 ICE gather timeout 로그는 `%d`에 문자열 state를 넘긴다.
- timeout 상황에서 logging error가 발생해 실제 WebRTC 디버깅 로그를 가릴 수 있다.

수정:

- `%d candidates` 표현을 제거하거나 `%s`로 바꾼다.
- 가능하면 SDP candidate count를 계산해서 별도 값으로 찍는다.

검증:

```bash
python3 -m py_compile tools/mac_proxy.py
python3 tools/_smoke_test.py
```

### 5. `sim_probe.py` 출력에서 Unicode 의존 줄이기

문제:

- 현재 `sim_probe.py` 출력은 `✓`, `✗`, `→`, `·`를 사용한다.
- Mac 터미널에서는 문제 없지만, 일부 CI/log collector에서 깨질 수 있다.

수정:

- 기본 출력은 `[OK]`, `[FAIL]`, `->`, `-`로 변경한다.
- 필요하면 `--pretty` 옵션으로 기존 출력을 유지한다.

검증:

```bash
python3 tools/sim_probe.py --help
```

## P2 개선 항목

### 6. 개발용 인증서 UX 정리

현 상태:

- `:8443` main page와 `:8444` signaling root가 같은 self-signed cert를 쓴다.
- `:8444`는 WebSocket cert warning UI가 직접 뜨지 않기 때문에 root stub page를 제공한다.

개선:

- README의 Quest 접속 순서를 `:8444` cert 예외 수락 -> `:8443` main page로 고정한다.
- 운영 전환 전에는 Caddy + Let's Encrypt DNS-01 경로를 별도 문서로 분리한다.

### 7. WebRTC 후보 확인 도구 추가

목표:

- Mac이 LAN `en0`와 Tailscale `utun*` host candidate를 모두 publish하는지 빠르게 확인한다.

구현안:

- `tools/ice_probe.py` 또는 `--dump-ice` 옵션 추가
- answer SDP에서 `a=candidate` 라인을 추출해 interface별 IP를 출력한다.

검증:

- LAN IP와 Tailscale `100.x.y.z` 후보가 모두 보이면 통과
- Tailscale 후보가 없으면 `tailscale up`, firewall, utun 상태를 안내한다.

### 8. WebXR payload schema fixture 추가

목표:

- page -> mac -> bridge boundary가 깨지지 않게 샘플 payload와 smoke test를 고정한다.

구현안:

- `tests/fixtures/page_pose_v1_1.msgpack.json` 또는 `tools/fixtures/page_pose_v1_1.json`
- `mac_proxy.py` publish body가 기준 문서의 `xlerobot_v1.1.page` shape와 맞는지 검사한다.

검증:

```bash
python3 tools/_smoke_test.py
```

## 통합 검증 순서

### 1단계: 로컬 smoke

```bash
python3 -m py_compile tools/*.py tools/webxr/serve.py
python3 tools/run.py --help
python3 tools/_smoke_test.py
python3 tools/_smoke_test_sim_bridge.py
```

통과 기준:

- syntax error 없음
- fake Quest pose가 `pose.0` 또는 지정한 topic으로 publish됨
- fake producer 또는 sim bridge placeholder track이 Quest consumer까지 도달함

### 2단계: Mac 단독 실행

```bash
python3 tools/run.py --video-source fake
```

통과 기준:

- `:8443`, `:8444`, `:7001` 정상 bind
- 로그에 LAN IP와 Tailscale IP가 모두 표시됨
- fake video producer가 signaling server에 연결됨

### 3단계: Quest M0b/M4 확인

절차:

1. Quest에서 `https://<mac-lan-ip>:8444/` 방문 후 cert 예외 수락
2. Quest에서 `https://<mac-lan-ip>:8443/?robot=0` 방문
3. Enter AR
4. B/Y/A/X/thumb 입력 확인

통과 기준:

- immersive-ar session 진입
- video plane이 demo.mp4 또는 WebRTC live로 표시됨
- pose WSS가 90Hz 근처로 들어옴
- `pose.0` ZMQ topic 수신 가능

### 4단계: sim 연결성 확인

```bash
python3 tools/sim_probe.py 100.80.87.68 --robot-id 0
python3 tools/run.py --video-source sim --sim-host 100.80.87.68 --sim-robot-id 0
```

통과 기준:

- `fleet_info.ok = true`
- `topic_list`에 `proprio.0`, `tf.links.0`, `rgb.front.0` 계열 topic 존재
- `rgb.front.0` JPEG가 WebRTC track으로 Quest에 표시됨

### 5단계: Home Server VR bridge 통합

전제:

- Home Server의 `examples/vr_teleop_bridge.py`가 `pose.0`을 SUB한다.
- bridge가 `tf.links.0` anchor를 잡고 `xlerobot_v1.1` command를 `:5556`으로 PUSH한다.

통과 기준:

- Grip clutch on 동안 controller delta가 EE target으로 반영됨
- clutch off 시 EE hold, relative delta 0
- B/Y e-stop 시 delta 0, base velocity 0, IK target freeze 또는 measured sync
- stale 150ms 초과 시 자동 stop path

## 완료 기준

P0과 P1이 끝나면 이 repo는 계획서 기준 M3/M4 역할을 안정적으로 수행해야 한다.

- `robot_id=0` 기본 single robot path가 깨지지 않는다.
- Quest page가 raw `local-floor` pose를 `xlerobot_v1.1.page` schema로 Mac에 보낸다.
- Mac proxy가 pose를 변환하지 않고 robot_id 메타만 붙여 `pose.<robot_id>`로 publish한다.
- WebRTC video는 없으면 demo fallback, 있으면 live track으로 전환된다.
- Mac/Quest repo는 IK, Isaac Sim, `:5556` command packing을 직접 담당하지 않는다.
