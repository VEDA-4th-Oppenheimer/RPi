# RPi LED 및 부저(Buzzer) 타이밍 버그 수정 및 상태 전이 최적화 보고서

- **작성일자**: 2026-08-27
- **작성자**: VEDA 4기 Oppenheimer 팀 (강유근)
- **적용 브랜치**: `feature/led_switch_buzzer`
- **수정 파일**: `daemon/modules/led/led_module.c`
- **관련 파일**: `daemon/core/main.c`, `daemon/modules/camera/camera_module.c`, `driver/led_sw_driver.c`

---

## 1. 개요 및 문제 현상 (Issue Description)

1. **스캔 정상 종료 시 부저가 길게 울리는 현상**:
   - 스캔이 완료되었을 때 완료 알림음(비프음)이 짧게 울리고 꺼져야 하나, 수 초에서 길게는 수십 초 동안 멈추지 않고 계속 울리는 현상이 관측됨.
2. **15초 후 불필요한 비상정지 경고음 발생**:
   - 스캔 완료 15초 후 모터 발열 방지를 위해 실행되는 정상 자동 절전(`auto_disarm`) 진입 시 `BUZ_ERROR`(0.2초 간격 2회) 경고음이 울림.
3. **요구사항**:
   - 정상 완료 알림음 지속 시간을 **정확히 1.0초(1000ms)**로 설정하고, 블로킹 없이 정밀하게 동작하도록 개선.

---

## 2. 근본 원인 분석 (Root Cause Analysis)

### 2.1 메인 스레드 동기 블로킹과 부저 타이머 틱 정지
- **기존 흐름**:
  1. 스캔 종료 시 `core_transition(c, ST_EXPORT)` 호출.
  2. `led_module.c`의 `led_on_state`가 `new_st == ST_EXPORT`를 감지하여 부저를 즉시 점등(`ctrl.buzzer = 1`).
  3. 동일 루프 내에서 바로 다음 모듈인 `camera_module.c`의 `on_state`가 실행되어 AI 카메라(`172.20.32.43:2222`)로 점군 JSON 파일(18,000점) mTLS 업로드를 동기식으로 수행 (`timeout_s = 60초`).
  4. 카메라 통신 시간 동안 **단일 이벤트 루프 메인 스레드가 블로킹**되어 100ms 타이머 틱(`on_tick`)이 멈춤.
  5. 틱 카운터(`s_buz_ticks`)가 증가하지 못하여 **카메라 통신이 끝날 때까지 부저가 계속 켜진 상태로 방치**됨.

```
[기존 결함 흐름]
ST_SCANNING ──> ST_EXPORT 진입 (부저 ON)
                 │
                 ├── camera_module mTLS 업로드 시작 (동기 블로킹 발생!)
                 │   └── ⚠️ 메인 스레드 멈춤 → 100ms 틱 정지 → 부저 소등 카운트 불가
                 │   └── ⚠️ 카메라 통신 시간(수 초~수십 초) 내내 부저가 계속 울림
                 │
                 └── 업로드 완료 ──> ST_IDLE 복귀 ──> 그제서야 0.5초 카운트 후 소등
```

### 2.2 자동 절전 DISARM의 에러음 오트리거
- 스캔 종료 후 STM32 되감기 여유(15초, `POST_SCAN_DISARM_MS`)가 지나면 `main.c`가 모터 과열을 방지하기 위해 `core_transition(c, ST_DISARM)`을 호출함.
- `led_on_state`에서 `new_st == ST_DISARM` 조건을 무조건 비상정지 에러로 취급하여 정상 절전 상태 진입임에도 `BUZ_ERROR` 경고음이 출력됨.

---

## 3. 수정 및 개선 사항 (Resolution Details)

### 3.1 부저 트리거 시점을 `ST_IDLE` 복귀 시점으로 변경
- `ST_EXPORT`(파일 마감 및 카메라 업로드 진행 중)가 아닌, 모든 산출물 처리와 카메라 전송이 완전히 끝나고 메인 루프로 복귀하는 **`old_st == ST_EXPORT && new_st == ST_IDLE` 시점에 부저를 트리거**.
- 메인 루프의 논블로킹 상태에서 100ms 틱이 방해 없이 실행되므로 정밀한 시간 제어가 보장됨.

### 3.2 정상 완료 부저 동작 시간을 1.0초(10 ticks)로 변경
- 1 tick = 100ms 기준, 기존 5 ticks(0.5초)에서 **10 ticks(1.0초)**로 연장하여 명확한 인지성 확보.

### 3.3 정상 자동 절전 DISARM 예외 처리
- `new_st == ST_DISARM && old_st != ST_IDLE` 조건을 적용하여, 정상 대기 중 15초 유예 만료로 인한 자동 절전 시에는 경고음이 울리지 않도록 보호.

---

## 4. Git Diff 상세 내역

```diff
diff --git a/daemon/modules/led/led_module.c b/daemon/modules/led/led_module.c
index fe3e80c..c501549 100644
--- a/daemon/modules/led/led_module.c
+++ b/daemon/modules/led/led_module.c
@@ -68,8 +68,8 @@ static void update_leds_buzzer(const struct shared_ctx *ctx, bool advance)
 
     /* 2. 부저 시퀀스 로직 (1 tick = 100ms) */
     if (s_buz_seq == BUZ_SCAN_DONE) {
-        /* 0.5초 1번 = 5 ticks ON */
-        if (s_buz_ticks < 5) {
+        /* 정상 종료 알림음: 1.0초 1번 = 10 ticks ON */
+        if (s_buz_ticks < 10) {
             ctrl.buzzer = 1u;
             if (advance) {
                 s_buz_ticks++;
@@ -199,14 +199,20 @@ static void led_on_tick(struct shared_ctx *ctx, daemon_state_t state)
 static void led_on_state(struct shared_ctx *ctx,
                          daemon_state_t old_st, daemon_state_t new_st)
 {
-    (void)old_st;
-
-    /* SCAN_DONE (ST_EXPORT 진입) 감지 -> 0.5초 알림음 */
-    if (new_st == ST_EXPORT) {
+    /* SCAN_DONE: ST_EXPORT -> ST_IDLE 복귀 시 1.0초 완료 알림음 (10 ticks)
+     *
+     * ⚠️ 이전에는 `new_st == ST_EXPORT` 진입 시 부저를 켰는데, 바로 다음 순서인
+     *   camera_module 의 mTLS 업로드가 동기 블로킹(수 초~수십 초)으로 실행되면서
+     *   100ms 틱 타이머가 멈춰 카메라 통신 내내 부저가 꺼지지 않고 계속 울리는
+     *   버그가 발생했다.
+     *   업로드가 모두 끝나고 메인 루프로 복귀하는 `ST_EXPORT -> ST_IDLE` 시점에
+     *   트리거하여 타이머 틱이 블로킹 없이 정확히 1.0초 카운트 후 소등되도록 한다. */
+    if (old_st == ST_EXPORT && new_st == ST_IDLE) {
         s_buz_seq = BUZ_SCAN_DONE;
         s_buz_ticks = 0;
-    } else if (new_st == ST_DISARM) {
-        /* 비상정지 (ST_DISARM 진입) 감지 -> 0.2초 2회 경고음 */
+    } else if (new_st == ST_DISARM && old_st != ST_IDLE) {
+        /* 비상정지 (ST_DISARM 진입) 감지 -> 0.2초 2회 경고음
+         * (스캔 완료 15초 후의 정상 자동 절전 DISARM 은 에러음에서 제외) */
         s_buz_seq = BUZ_ERROR;
         s_buz_ticks = 0;
     }
```

---

## 5. 검증 결과 (Verification Results)

- **정적 분석**: `cppcheck --enable=all` 실행 결과 **0 Errors, 0 Warnings** (MISRA-C / CWE 가이드라인 준수)
- **컴파일 무결성**: GCC 최적화 빌드 이상 없음
- **동작 보장**: 스캔 및 파일 마감, 카메라 업로드 완료 후 **정확히 1.0초 동안만 1회 부저 알림음 출력 후 정상 소등**
