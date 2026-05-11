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
├── .gitignore
└── tools/
    ├── webxr/
    │   ├── index.html         ← three.js + WebXR + WebRTC consumer (§4.1)
    │   ├── serve.py           ← M0b 최소 HTTPS 서버 (echo). M3+ 부터는 mac_proxy 사용
    │   └── assets/demo.mp4    ← SMPTE bars + 타임코드, 10s
    ├── mac_proxy.py           ← M3+ 정식 데몬 (§4.2 전체 구현)
    ├── fake_producer.py       ← M3 검증용 가짜 Home Server (WebRTC video publisher)
    └── requirements.txt       ← aiohttp, pyzmq, msgpack, aiortc, av, psutil
```

## 포트 / 프로토콜 맵 (§4.2)

| 포트 | 프로토콜 | 누가 쓰나 | 무엇 |
|---|---|---|---|
| `:8443` | HTTPS  | Quest 브라우저 | `GET /` → index.html · `GET /assets/*` · `GET /ws` WSS pose |
| `:8444` | HTTPS  | Quest 브라우저 + Home Server | `GET /signaling/quest` · `GET /signaling/server` (WSS WebRTC signaling) |
| `:7001` | ZMQ PUB tcp | VR bridge (Home Server) | topic `b"pose.<robot_id>"`, payload msgpack(dict) |
| `*`     | WebRTC media | Quest ↔ Mac ↔ Home Server | ICE 협상 후 동적 포트, 양 인터페이스(en0, utun) host candidate |

## 실행 (전체 시스템)

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

`https://<mac-lan-ip>:8443/?robot=1` 접속 → 인증서 경고 통과 → *Enter VR*.

- `?robot=N` 가 없으면 default 1. 페이지는 첫 WS 메시지로 `{"select_robot": N}` 를 보내고,
  mac_proxy 는 이후 들어오는 모든 pose 페이로드를 토픽 `b"pose.<N>"` 으로 publish.
- video plane 은 (a) WebRTC 트랙이 붙어 있으면 그것을, (b) 없으면 `assets/demo.mp4`
  를 흘려보낸다. HUD 의 오른쪽 작은 라인이 `video=WebRTC live` / `video=demo.mp4 fallback`
  으로 현재 상태를 명시.

### 3. Home Server (M8 전): 가짜 producer 로 loopback

Home Server 측 WebRTC 통합(M8) 전에 미디어 경로를 단독 검증하려면 같은 Mac
에서 `fake_producer.py` 를 한 번 더 띄운다:

```bash
python3 tools/fake_producer.py \
    --url wss://localhost:8444/signaling/server \
    --source tools/webxr/assets/demo.mp4
```

→ Quest 페이지의 video plane 이 `WebRTC live` 로 바뀌면서 같은 demo.mp4 가 흐른다
(파일 fallback 과 WebRTC 라이브가 시각적으로 같으므로, HUD 의 status 라인으로 구분).

### 4. Home Server (M8 이후): 진짜 NVENC producer

같은 `/signaling/server` 엔드포인트에 Home Server 의 NVENC encoder 가 offer 를
보내면 끝. 페이지는 새로고침 없이 자동으로 트랙이 갈아끼워진다 (`replaceTrack` on sender).

## VR bridge 가 받는 ZMQ 페이로드 (§4.2 ZMQ PUB)

mac_proxy 가 페이지의 raw `local-floor` payload 를 그대로 보내지는 않고, robot_id 와
mac 도착 시각을 메타로 붙인 msgpack dict 로 publish.

```python
# bridge 쪽 sub 예시
import zmq, msgpack
ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)
sub.connect("tcp://<mac-tailscale-ip>:7001")
sub.setsockopt(zmq.SUBSCRIBE, b"pose.2")          # robot_id=2 만
while True:
    topic, body = sub.recv_multipart()
    pose = msgpack.unpackb(body, raw=False)
    # pose: {
    #   "schema":    "xlerobot_v1.1.page",
    #   "stamp_ns":  <mac mono ns>,
    #   "robot_id":  2,
    #   "t_page_ms": <page perf.now() ms>,
    #   "hmd":   [x,y,z, qx,qy,qz,qw],            # local-floor frame (RH)
    #   "left":  {"pose":[...] | None, "grip":f, "trigger":f, "buttons":{...}},
    #   "right": { ... },
    #   "estop": bool,
    # }
```

bridge 는 `R_robot_from_xr` (§4.4) 적용 후 anchor/twist-swing/clamp 처리 → sim 으로 PUSH.
Page payload 는 의도적으로 robot 좌표계로 변환되어 있지 않다 — 그 변환은 bridge 책임.

## M3 통과 기준 (계획서 §8 row "3")

| # | 검증 항목 | 확인 방법 |
|---|---|---|
| (a) | Mac proxy 가 양 인터페이스 host candidate publish | `mac_proxy.py` 기동 로그에 en0 + utun 둘 다 enumerate. `fake_producer.py` 가 동일 호스트에서 붙고 connection state 가 `connected` |
| (b) | mac ↔ server end-to-end <10ms (가짜 producer) | `fake_producer.py` 가 mac_proxy 와 localhost 통신 시 트랙 도착까지 1초 미만, RTP 흐름 안정 |
| (c) | `?robot=N` 라우팅 | Quest 페이지에서 `?robot=2` 로 접속 후 mac_proxy 로그에 `pose ws[xxx] robot_id 1 -> 2`. `zmq` SUB 로 `b"pose.2"` 토픽 확인 |
| (d) | pose ZMQ PUB 90Hz | 별도 sub 클라이언트로 토픽 SUB 시 5초 평균 ~90Hz |

## M0b 통과 기준 (§8 row "0b")

(이미 통과되어 있다고 가정. mac_proxy 가 켜져 있어도 M0b 의 세 항목은 그대로 동작 —
WebRTC 안 붙으면 demo.mp4 가 fallback 으로 흐른다.)

| # | 검증 항목 | 확인 방법 |
|---|---|---|
| (i)   | immersive-vr 세션 진입 | Enter VR → 헤드셋이 grid + video plane 보임 |
| (ii)  | `<video>` 동시 렌더링 | video plane 에 SMPTE bars + 타임코드가 끊김 없이 흐름 |
| (iii) | B/Y/A/X 가 page JS 에 도달 | HUD 의 버튼 row 점등. B → 빨간 E-STOP. thumb click 으로 해제 |

## 페이지 → mac → bridge wire 흐름 (한 장)

[`M0b_dataflow.svg`](./M0b_dataflow.svg) 참조. M3+ 에서는 점선 → 실선으로 활성화되는
것들이 바로 이 문서의 §4.2 구현이다.

## 버튼 매핑 (Quest Touch Plus, §4.1)

| 입력 | Gamepad index | 동작 |
|---|---|---|
| Trigger | `buttons[0]` 아날로그 | gripper close (`right.trigger` / `left.trigger`) |
| Grip    | `buttons[1]` 아날로그 | clutch hold (right/left 독립) |
| A (R) / X (L) | `buttons[4]` digital | anchor 강제 리셋 |
| B (R) / Y (L) | `buttons[5]` digital | **E-STOP latched** |
| Thumbstick click | `buttons[3]` digital | M0b 단계 e-stop release fallback (§8 비고) |
| Menu (system) | OS 가로챔 | page JS 에 안 도달 |

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

두 스크립트는 같은 `tools/webxr/dev-cert.pem` 자기서명 인증서를 공유. mac_proxy
가 켜져 있으면 serve.py 는 띄울 필요 없음.

## 운영 환경(§10) 전환 체크리스트

- 인증서: 자기서명 → Caddy + Let's Encrypt DNS-01. mac_proxy 는 `--port 8443` 을
  localhost-bind 로 바꾼 뒤 Caddy reverse proxy 받는 형태로 변경.
- ZMQ PUB bind: `0.0.0.0` → Tailscale 인터페이스 IP 로 명시 (`tcp://100.x.y.z:7001`).
- 방화벽: Mac 의 8443/8444 인바운드는 LAN, 7001 인바운드는 Tailnet 만 허용.
- WebRTC ICE: STUN 추가 (`--stun stun:stun.l.google.com:19302`) 가 필요한지 §6.2
  의 latency budget 결과로 결정.

## 디버깅 팁

- 인터페이스 점검: `python3 tools/mac_proxy.py --log-level DEBUG` → 기동 시 ipv4 enumerate 출력.
- ICE candidate 보기: Quest Chrome inspect 의 `chrome://webrtc-internals` 또는 mac_proxy 의 `aioice` 로그 레벨 INFO 로 상승.
- pose 토픽 sniff: `python3 -c "import zmq,msgpack;c=zmq.Context();s=c.socket(zmq.SUB);s.connect('tcp://localhost:7001');s.setsockopt(zmq.SUBSCRIBE,b'');import sys;t,b=s.recv_multipart();print(t,msgpack.unpackb(b))"`
- 영상 codec: `ffprobe tools/webxr/assets/demo.mp4` — H.264 baseline 3.1 yuv420p.
- WebXR profile/버튼 매핑 검증: `https://immersive-web.github.io/webxr-samples/input-profiles.html` 같은 페이지를 Quest Browser 에서 별도로 띄워 확인.
