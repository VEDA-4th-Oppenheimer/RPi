# MQTT Interface Contract — v1.0

이 문서는 계약서다. Qt 쪽은 이 문서만 보고 구현할 수 있어야 하고, 데몬 쪽은 여기 적힌
것만 발행·구독한다. 바꾸려면 양쪽 합의 후 이 문서를 먼저 고친다.

데몬 담당: 이현우 · Qt 담당: 송영빈 · 브로커/인증서: 이광진

기준 코드: RPi `daemon/`, `shared/daemon_module.h`

## 1. 연결 정보

| 항목 | 값 | 비고 |
|---|---|---|
| 브로커 | Mosquitto (RPi 상주) | 킷 자체가 통신 허브 |
| 호스트 | RPi IP (예 10.144.31.125) | 고정 IP 또는 raspberrypi.local |
| 포트 | 8883 (MQTT over TLS) | 평문 1883 은 외부에 열지 않음 |
| 인증 | mTLS (클라이언트 인증서) | §6 참조 |
| MQTT 버전 | 3.1.1 | libmosquitto / QtMqtt 기본 |
| Keepalive | 30 초 | LWT 발동 시간에 영향 |
| Client ID | qt-console | 중복되면 서로 끊는다. 유일하게 |

## 2. 토픽 일람

구조: `adts/<kit_id>/<class>/<name>` — kit_id 는 현재 `kit1` 고정.

| 토픽 | 방향 | QoS | Retained | 내용 |
|---|---|---|---|---|
| `adts/kit1/cmd/scan` | Qt → 데몬 | 1 | 금지 | 스캔 시작 |
| `adts/kit1/cmd/stop` | Qt → 데몬 | 1 | 금지 | 스캔 중단 |
| `adts/kit1/cmd/home` | Qt → 데몬 | 1 | 금지 | 홈만 수행 |
| `adts/kit1/cmd/disarm` | Qt → 데몬 | 1 | 금지 | 안전정지 (모터 전류 차단) |
| `adts/kit1/state/daemon` | 데몬 → Qt | 1 | 예 | FSM·링크·홈·현재각 (LWT 대상) |
| `adts/kit1/state/scan` | 데몬 → Qt | 1 | 예 | 마지막 스캔 결과 (파일 경로) |
| `adts/kit1/event/progress` | 데몬 → Qt | 0 | 아니오 | 진행률 (스캔 중 2Hz) |
| `adts/kit1/event/error` | 데몬 → Qt | 1 | 아니오 | 오류 발생 |

Qt 가 구독할 것: `adts/kit1/state/#` 와 `adts/kit1/event/#` (두 줄이면 끝)

Qt 가 발행할 것: `adts/kit1/cmd/*` 만

⚠️ cmd 토픽에 retained 를 절대 걸지 말 것 (안전 문제)

retained 된 `cmd/scan` 은 데몬이 재접속할 때마다 다시 배달된다. 그러면 전원을 껐다 켜거나
데몬을 재시작할 때마다 킷이 혼자 스캔을 시작한다. 사람 손이 기구에 들어가 있을 수 있으므로
단순 버그가 아니라 안전 사고다.

QtMqtt: `publish(topic, payload, 1, false)` — 네 번째 인자가 retain. 반드시 false.

## 3. 페이로드 규격

전부 UTF-8 JSON. 모르는 필드는 무시할 것(전방 호환).

### 3.1 `cmd/scan` — 스캔 시작

```json
{
  "req_id": "a1b2c3d4",
  "pan_ddeg":  [0, 1790],
  "tilt_ddeg": [-900, 900],
  "step_ddeg": 10,
  "sensor_height_mm": 2400
}
```

| 필드 | 타입 | 범위 | 필수 | 설명 |
|---|---|---|---|---|
| req_id | string | 1~32자 | 예 | Qt 가 생성. 결과 대조용 (§4) |
| pan_ddeg | [int, int] | 0 ~ 3599 | 예 | 0.1도 단위. [시작, 끝] |
| tilt_ddeg | [int, int] | −900 ~ 900 | 예 | 0.1도 단위. 부호 있음 |
| step_ddeg | int | > 0 | 예 | 격자 간격. 10 (=1.0도) 권장 |
| sensor_height_mm | int | ≥ 0 | 아니오 | 지면→라이다 높이. 좌표엔 미적용, 메타데이터. 모르면 0 |

팬은 **0~1790** 으로 끊을 것 (0~1800 아님).

틸트 스윕이 바닥을 지나면서 한 줄이 방위 p 와 p+180 을 함께 훑는다. 그래서 팬을 0~180
양끝 포함으로 돌리면 첫 줄과 마지막 줄이 같은 수직 평면을 잡아 방위 0도와 180도만 두 번
측정된다(실측 중복 180건). 0~1790 이면 중복 0.

데몬이 이 조건을 감지하면 경고 로그를 남기지만 거부하지는 않는다.

UI 기본값 권장: pan [0,1790] / tilt [-900,900] / step 10 → 격자 91×360, 약 5분 소요.

### 3.2 `cmd/stop` / `cmd/home` / `cmd/disarm`

```json
{ "req_id": "e5f6g7h8" }
```

| 명령 | 동작 | 언제 쓰나 |
|---|---|---|
| stop | 스캔 중단. 여기까지 받은 점으로 파일을 마감한다 | 사용자가 중간에 끊고 싶을 때 |
| home | 홈만 수행 (구동 없음, 엔코더 판독 1회) | 보통 불필요 — 스캔 전에 데몬이 자동으로 한다 |
| disarm | 즉시 정지 + 모터 전류 차단 | 비상. UI 에 항상 보이는 버튼으로 |

### 3.3 `state/daemon` — 데몬 상태 (retained)

```json
{
  "state": "SCANNING",
  "online": true,
  "link_alive": true,
  "homed": true,
  "scanning": true,
  "cur_pan_ddeg": 450,
  "cur_tilt_ddeg": -230,
  "last_err": 0,
  "level": { "valid": false, "roll_deg": 0.0, "pitch_deg": 0.0 },
  "ts": 1785500123
}
```

| 필드 | 값 | UI 활용 |
|---|---|---|
| state | IDLE / SCANNING / EXPORT / DISARM / OFFLINE | 메인 상태 표시. §5 참조 |
| online | bool | false = LWT 로 브로커가 대신 보낸 것 = 데몬 죽음 |
| link_alive | bool | RPi↔STM32 링크. false 면 하드웨어 문제 |
| homed | bool | 홈 완료 여부 |
| cur_pan_ddeg / cur_tilt_ddeg | int (0.1도) | 현재 각도. 기구각이다(§7) |
| last_err | int | 0=정상. 코드표는 §3.5 |
| level | object | IMU 수평. valid:false 면 IMU 미구현 — 표시하지 말 것 |
| ts | int (unix sec) | 발행 시각 |

발행 시점: 상태가 바뀔 때마다 + 최소 5초에 한 번(heartbeat 겸용).

### 3.4 `state/scan` — 스캔 결과 (retained)

```json
{
  "req_id": "a1b2c3d4",
  "ok": true,
  "session_id": "calib-20260730-214014",
  "scan_id": "sweep-000001",
  "pcd":  "/var/lib/adts/scans/calib-20260730-214014_sweep-000001.pcd",
  "json": "/var/lib/adts/scans/calib-20260730-214014_sweep-000001_pan_tilt_lidar.json",
  "rows": 76,
  "columns": 360,
  "points": 27045,
  "expected": 27360,
  "duration_s": 304.7,
  "ts": 1785500428
}
```

포인트클라우드 파일 자체는 MQTT 로 오지 않는다. JSON 이 15MB, PCD 가 0.6MB 라 브로커에
부담이고 페이로드 한계에도 걸린다. 경로만 온다.

Qt 가 파일을 읽어야 하면 별도 경로로 가져가야 한다 — Samba 공유 / NFS / scp / 간단한
HTTP 서버 중 택1. 이 방식은 아직 미결이니 이광진·이현우와 협의할 것.

### 3.5 `event/error`

```json
{ "req_id": "a1b2c3d4", "code": 3, "name": "ERR_NOT_HOMED",
  "msg": "홈 전에 스캔 요청", "fatal": true, "ts": 1785500130 }
```

| code | name | 의미 | Qt 표시 권장 |
|---|---|---|---|
| 1 | ERR_BAD_CRC | 프레임 손상 | 경고 (일시적일 수 있음) |
| 2 | ERR_BAD_LEN | 길이 이상 | 경고 |
| 3 | ERR_NOT_HOMED | 홈 안 된 상태로 스캔 요청 | 오류 — 홈 실패 의심 |
| 4 | ERR_OUT_OF_RANGE | 스캔 파라미터 범위 밖 | 오류 — 입력값 확인 |
| 5 | ERR_STALL | 모터 탈조 (엔코더 대조 2.0도 초과) | 오류 — 기구 확인 필요 |
| 6 | ERR_LIDAR | 라이다 무응답 | 오류 |
| 100 | ERR_LINK_DEAD | RPi↔STM32 통신 두절 (데몬 판정) | 오류 — 배선/전원 확인 |
| 101 | ERR_HOME_TIMEOUT | 홈 3초 무응답 | 오류 |

### 3.6 `event/progress`

```json
{ "req_id": "a1b2c3d4", "points": 12345, "expected": 32580,
  "percent": 38, "ts": 1785500250 }
```

스캔 중 약 2Hz. QoS 0 이라 유실될 수 있다 — 진행바 갱신용으로만 쓰고, 완료 판정은
`state/daemon` 의 `state` 로 할 것.

## 4. req_id 규칙

- Qt 가 명령마다 새로 생성한다 (UUID 앞 8자 등, 재사용 금지)
- 데몬은 그 요청에서 파생된 모든 응답(progress / state/scan / error)에 같은 값을 되돌려준다
- Qt 는 자기가 보낸 req_id 가 아닌 응답은 무시할 것 — 다른 콘솔이 붙어 있을 수 있다

왜 필요한가: 스캔이 5분 걸린다. 그 사이 다른 클라이언트가 명령을 넣거나, QoS 1 특성상
같은 명령이 중복 배달될 수 있다. req_id 없이는 "지금 온 결과가 내가 시킨 그건가"를 알
방법이 없다.

데몬도 같은 req_id 를 연속으로 받으면 뒤엣것을 무시한다.

## 5. 상태 머신 — Qt UI 매핑

| state | 의미 | 버튼 활성 | 표시 |
|---|---|---|---|
| OFFLINE | 데몬 미접속 / 죽음 | 없음 | 회색 "연결 안 됨" |
| IDLE | 대기 — 스캔 가능 | 스캔 시작, 홈 | 초록 "준비" |
| SCANNING | 스캔 중 | 중단, 비상정지 | 진행바 + 남은 시간 |
| EXPORT | 파일 저장 중 (수 초) | 없음 | "저장 중..." |
| DISARM | 안전정지 — 모터 전류 차단 | 복구 | 빨강 "정지됨" |

### 5.2 데몬이 죽었을 때 (LWT)

데몬은 접속 시 Last Will 을 등록한다. 데몬 프로세스가 죽거나 RPi 전원이 나가면 브로커가
대신 발행한다:

```json
{ "state": "OFFLINE", "online": false }
```

retained + QoS 1 이므로 Qt 는 keepalive 시간(약 30초) 안에 이걸 받는다.

Qt 는 이걸 반드시 처리해야 한다. 없으면 데몬이 죽어도 화면에 IDLE 이 그대로 남아 있어,
조작자가 "준비됨" 으로 착각하고 버튼을 누르게 된다.

## 6. TLS / mTLS

포트 8883 은 MQTTS 이자 mTLS 다 — 브로커가 클라이언트 인증서를 요구한다
(`require_certificate true`).

### 6.1 Qt 에 필요한 파일 3개

| 파일 | 용도 |
|---|---|
| ca.crt | 브로커 인증서 검증용 (우리 CA) |
| qt-console.crt | Qt 자신의 인증서. CN = qt-console (ACL 신원) |
| qt-console.key | Qt 자신의 개인키. 절대 저장소에 커밋하지 말 것 |

ca.key 는 RPi 에만 있고 배포되지 않는다. 발급은 이광진.

### 6.3 함정 3개 (Windows/QtMqtt 기준. Paho MQTT C++ 로 이식 시에도 개념은 동일)

① OpenSSL 라이브러리 누락 — 가장 많이 걸린다. TLS 가 조용히 실패한다.

② 개인키 포맷 — OpenSSL 3.x 는 기본이 PKCS#8(`-----BEGIN PRIVATE KEY-----`)인데
전통 RSA 포맷(`-----BEGIN RSA PRIVATE KEY-----`)을 기대하는 라이브러리에서 조용히
실패할 수 있다. 필요하면 `openssl rsa -in qt-console.key -out qt-console-trad.key` 변환.

③ SAN / 호스트명 검증 — IP 로 접속하면 인증서에 IP SAN 이 있어야 한다. 없으면 검증
실패다. peer verify 를 꺼서 우회하면 mTLS 의 의미가 사라지므로, 발급자에게 SAN 을
요청할 것.

## 7. ⚠️ 각도 해석 주의

`cmd/scan` 과 `state/daemon` 의 각도는 **기구각**이다 — 모터가 실제로 어디 있는지.

산출물 파일(.json/.pcd) 안의 각도는 **계약각**으로, 둘은 1:1 이 아니다. 틸트 스윕이
바닥을 지나면서 한 줄이 방위 p 와 p+180 을 함께 훑기 때문이다.

```
기구 틸트 m ≤ 0 :  계약 pan = p        tilt = -90 - m
기구 틸트 m > 0 :  계약 pan = p + 180  tilt = -90 + m
```

Qt 는 기구각만 다루면 된다. 변환은 데몬이 하고, 계약각은 파일 안에만 존재한다. 현재
각도를 화면에 그릴 때 "팬 45도" 는 모터 위치지 빔이 보는 방위가 아니라는 점만 유의.

## 8. 구현 체크리스트 (Qt)

- [ ] TLS 라이브러리 지원 여부 먼저 확인
- [ ] 구독은 `state/#` + `event/#` 두 줄
- [ ] 발행 시 retain=false 확인 (cmd 토픽)
- [ ] req_id 생성 및 응답 대조 (남의 응답 무시)
- [ ] state:"OFFLINE" 처리 — 데몬 죽음을 화면에 반영
- [ ] DISARM 버튼은 항상 활성 (비상정지)
- [ ] 팬 기본값 [0, 1790] (1800 아님)
- [ ] 진행률 유실 가정 — 완료 판정은 state 로
- [ ] 개인키를 저장소에 커밋하지 않기 (.gitignore)

## 9. 미결 사항

- 파일 전달 방식 — 포인트클라우드를 Qt/카메라가 어떻게 가져갈지 (Samba / NFS / HTTP). 지금은 경로만 통지
- 카메라 단 토픽 — 캘리브 결과·객체 검출을 어디에 발행할지 (이영민 협의)
- 브로커 ACL — 인증서 CN 기반으로 카메라가 cmd/* 를 못 쏘게 제한 (이광진)
- RPi 시계 — RTC 가 없어 인터넷 없이 부팅하면 인증서가 "not yet valid" 로 거부될 수 있음. fake-hwclock 또는 DS3231 검토
- ~~mqtt_module 데몬측 구현~~ — `daemon/modules/mqtt/mqtt_module.c` 구현 완료(이 계약 기준). TLS는 데몬→브로커 loopback 구간은 평문 유지(§1 "외부에 열지 않음"의 해석), Qt→브로커 8883/mTLS 구간만 암호화 대상.
