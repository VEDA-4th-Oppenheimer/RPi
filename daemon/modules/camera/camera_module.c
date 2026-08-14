/* ============================================================================
 *  camera_module.c  --  스캔 JSON 을 CCTV 앱(CV5 OpenSDK)으로 TCP 업로드
 *  담당: 이현우 (수신측 lidar_json_receiver = 이영민)
 *
 *  스캔이 끝나면(ST_EXPORT) 원시 측정 JSON 을 카메라의 TCP 서버로 보낸다.
 *  캘리브 연산은 카메라 단이 하고, 여기서는 파일을 건네주는 것까지만 한다.
 *
 *  ── 와이어 프로토콜 (lidar_json_receiver/README.md) ──
 *    한 연결에 파일 하나.
 *      [2바이트 파일명 길이, 네트워크 바이트오더]
 *      [파일명 ASCII]
 *      [8바이트 파일 길이, 네트워크 바이트오더]
 *      [JSON 바이트]
 *    → 응답 JSON 한 줄:  {"result":"ok", ...}  또는  {"result":"error", ...}
 *
 *  ⚠️ 수신측이 C++ 이지만 상관없다. 길이 접두 + 원시 바이트라 언어가 개입할
 *    여지가 없다. 직렬화 라이브러리도, ABI 도 끼지 않는다.
 *
 *  ── 왜 **동기**인가 (블로킹) ──
 *    보내는 시점이 스캔이 끝난 직후라, 그때 루프가 잠깐 멈춰도 잃는 것이 없다:
 *      · 스캔 점은 더 이상 오지 않는다 (SCAN_DONE 을 이미 받았다)
 *      · 모터는 STM32 가 알아서 파킹 중이고, STM32 는 PING 두절을 감시하지
 *        않는다(펌웨어 미구현) — 즉 데몬이 멈춰도 STM 쪽 부작용이 없다
 *      · 파일은 이미 디스크에 마감돼 있다
 *
 *    반대로 비동기로 하면 **--once 에서 전송 중에 프로세스가 죽는다**:
 *        ST_EXPORT -> (업로드 시작) -> IDLE -> exit_after_scan -> shutdown
 *    scan_batch.sh 가 --once 를 반복하므로 매 회차가 그렇게 된다. 동기가
 *    단순할 뿐 아니라 이 경우에 **더 옳다**.
 *
 *  ⚠️ 대신 두 가지를 반드시 지킨다:
 *    ① 소켓 타임아웃 — 카메라가 안 받으면 send() 가 영원히 안 돌아온다.
 *       무선이라 넉넉히 잡되 유한해야 한다.
 *    ② 끝나면 core_hb_reprime() — 안 하면 블로킹한 시간을 통신 두절로
 *       오판해 매번 DISARM 이 걸린다(그쪽 주석 참조).
 * ==========================================================================*/
#include "daemon_module.h"

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 접속 파라미터 — 환경변수로 덮어쓸 수 있다(mqtt_module 과 같은 방식) */
#define ENV_HOST      "ADTS_CAM_HOST"
#define ENV_PORT      "ADTS_CAM_PORT"
#define ENV_TIMEOUT   "ADTS_CAM_TIMEOUT_S"
#define ENV_DISABLE   "ADTS_CAM_DISABLE"     /* "1" 이면 업로드 자체를 끈다 */

/* 카메라(CV5 CCTV 앱) 주소. 망이 바뀌면 ADTS_CAM_HOST 로 덮어쓰면 되고,
 * systemd 유닛에서는 Environment= 로 준다. */
#define DEF_HOST      "172.20.32.29"
#define DEF_PORT      8081

/* ⚠️ 무선 기준. JSON 이 20~25MB 인데 Wi-Fi 실효 대역이 흔들리면 수십 초가
 *   걸린다. 이 값은 **한 번의 send/recv 가 멈춰 있을 수 있는 한도**이지 전체
 *   전송 시간이 아니다(진행 중이면 타이머가 갱신된다). 그래도 넉넉히 둔다. */
#define DEF_TIMEOUT_S  60

#define MAX_ATTEMPTS   3u        /* 사용자 요청: 최대 3회 */
#define RETRY_DELAY_US 2000000u  /* 재시도 사이 2초 — 순간적 무선 끊김 대비 */

#define CHUNK_BYTES    (256u * 1024u)
#define MAX_FILE_BYTES (64ull * 1024ull * 1024ull)   /* 수신측 상한과 동일 */
#define MAX_REPLY      2048u

static struct shared_ctx *s_ctx;
static char     s_host[128];
static int      s_port;
static int      s_timeout_s;
static bool     s_disabled;
static uint32_t s_last_result_seq;   /* 같은 스캔을 두 번 올리지 않기 위한 표식 */

/* ---------------------------------------------------------------------------
 *  보조
 * ------------------------------------------------------------------------- */
static const char *env_or(const char *name, const char *def)
{
    const char *v = getenv(name);
    return ((v != NULL) && (v[0] != '\0')) ? v : def;
}

/* 경로에서 파일명만. 수신측이 이 이름 그대로 저장한다. */
static const char *base_name(const char *path)
{
    const char *slash = strrchr(path, '/');
    return (slash != NULL) ? (slash + 1) : path;
}

/* 수신측 GetFileName() 이 거부하는 문자를 미리 거른다 — 보내고 나서 실패하는
 * 것보다 보내기 전에 아는 편이 낫다(25MB 를 헛되이 올리지 않는다). */
static bool name_is_acceptable(const char *n)
{
    size_t len = strlen(n);

    if ((len == 0u) || (len > 255u) || (strstr(n, "..") != NULL)) {
        return false;
    }
    for (size_t i = 0u; i < len; i++) {
        const char c = n[i];
        const bool ok = ((c >= 'A') && (c <= 'Z')) || ((c >= 'a') && (c <= 'z'))
                     || ((c >= '0') && (c <= '9'))
                     || (c == '.') || (c == '_') || (c == '-');
        if (!ok) {
            return false;
        }
    }
    return true;
}

/* 전부 보낼 때까지 반복. EINTR 은 재개, 그 외 오류는 실패.
 * ⚠️ MSG_NOSIGNAL — 상대가 끊긴 소켓에 쓰면 SIGPIPE 로 **프로세스가 죽는다**.
 *   데몬이 스캔 산출물을 들고 있는 상태라 절대 죽으면 안 된다. */
static bool send_all(int fd, const void *buf, size_t len)
{
    const unsigned char *p = buf;
    size_t sent = 0u;

    while (sent < len) {
        const ssize_t n = send(fd, p + sent, len - sent, MSG_NOSIGNAL);
        if (n > 0) {
            sent += (size_t)n;
        } else if ((n < 0) && (errno == EINTR)) {
            continue;
        } else {
            return false;
        }
    }
    return true;
}

/* getaddrinfo 로 접속. 호스트명도 IP 도 받는다. 실패 시 -1. */
static int connect_camera(void)
{
    struct addrinfo hints;
    struct addrinfo *list = NULL;
    char port_s[16];
    int fd = -1;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    (void)snprintf(port_s, sizeof(port_s), "%d", s_port);

    if (getaddrinfo(s_host, port_s, &hints, &list) != 0) {
        return -1;
    }
    for (struct addrinfo *a = list; a != NULL; a = a->ai_next) {
        fd = socket(a->ai_family, a->ai_socktype, a->ai_protocol);
        if (fd < 0) {
            continue;
        }
        if (connect(fd, a->ai_addr, a->ai_addrlen) == 0) {
            break;
        }
        (void)close(fd);
        fd = -1;
    }
    freeaddrinfo(list);

    if (fd >= 0) {
        struct timeval tv;
        memset(&tv, 0, sizeof(tv));
        tv.tv_sec = s_timeout_s;
        /* ⚠️ 이게 없으면 카메라가 응답을 멈췄을 때 데몬이 영원히 멈춘다.
         *   scan_batch.sh 의 회차 제한시간에 걸릴 때까지 아무 일도 안 한다. */
        (void)setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        (void)setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }
    return fd;
}

/* 한 번 시도. 성공 0 / 실패 -1. reason 에 실패 사유를 남긴다. */
static int upload_once(const char *path, char *reason, size_t reason_len)
{
    struct stat st;
    FILE *f = NULL;
    int fd = -1;
    int rc = -1;

    if ((stat(path, &st) != 0) || !S_ISREG(st.st_mode)) {
        (void)snprintf(reason, reason_len, "파일 없음: %s", path);
        return -1;
    }
    if ((st.st_size <= 0) || ((uint64_t)st.st_size > MAX_FILE_BYTES)) {
        (void)snprintf(reason, reason_len, "크기 범위 밖: %lld B",
                       (long long)st.st_size);
        return -1;
    }

    const char *name = base_name(path);
    if (!name_is_acceptable(name)) {
        (void)snprintf(reason, reason_len, "수신측이 거부할 파일명: %s", name);
        return -1;
    }

    f = fopen(path, "rb");
    if (f == NULL) {
        (void)snprintf(reason, reason_len, "열기 실패: %s", strerror(errno));
        return -1;
    }

    fd = connect_camera();
    if (fd < 0) {
        (void)snprintf(reason, reason_len, "접속 실패 %s:%d", s_host, s_port);
        (void)fclose(f);
        return -1;
    }

    /* ── 헤더 ── 길이는 전부 네트워크 바이트오더(big endian) ── */
    const uint16_t nlen_be = htons((uint16_t)strlen(name));
    unsigned char size_be[8];
    uint64_t sz = (uint64_t)st.st_size;
    for (size_t i = 0u; i < 8u; i++) {
        size_be[7u - i] = (unsigned char)(sz & 0xFFu);
        sz >>= 8u;
    }

    if (!send_all(fd, &nlen_be, sizeof(nlen_be))
     || !send_all(fd, name, strlen(name))
     || !send_all(fd, size_be, sizeof(size_be))) {
        (void)snprintf(reason, reason_len, "헤더 전송 실패: %s", strerror(errno));
        goto out;
    }

    /* ── 본문 ── */
    {
        static unsigned char buf[CHUNK_BYTES];   /* 스택 256KB 회피 */
        uint64_t total = 0u;

        for (;;) {
            const size_t got = fread(buf, 1u, sizeof(buf), f);
            if (got == 0u) {
                break;
            }
            if (!send_all(fd, buf, got)) {
                (void)snprintf(reason, reason_len,
                               "본문 전송 실패 (%llu/%lld B): %s",
                               (unsigned long long)total,
                               (long long)st.st_size, strerror(errno));
                goto out;
            }
            total += got;
        }
        if (total != (uint64_t)st.st_size) {
            (void)snprintf(reason, reason_len, "읽기 중단 %llu/%lld B",
                           (unsigned long long)total, (long long)st.st_size);
            goto out;
        }
    }

    /* ── 응답 ── 한 줄 JSON. 개행이 오거나 상대가 닫으면 끝. ── */
    {
        char reply[MAX_REPLY];
        size_t used = 0u;

        for (;;) {
            const ssize_t n = recv(fd, reply + used, (sizeof(reply) - 1u) - used, 0);
            if (n > 0) {
                used += (size_t)n;
                reply[used] = '\0';
                if (strchr(reply, '\n') != NULL) {
                    break;
                }
                if (used >= (sizeof(reply) - 1u)) {
                    break;
                }
            } else if ((n < 0) && (errno == EINTR)) {
                continue;
            } else if ((n == 0) && (used > 0u)) {
                break;                     /* 상대가 닫음 — 받은 만큼으로 판정 */
            } else {
                (void)snprintf(reason, reason_len, "응답 없음: %s",
                               (n == 0) ? "연결 종료" : strerror(errno));
                goto out;
            }
        }
        reply[used] = '\0';

        /* 수신측 IsSuccessReply() 와 같은 기준. 공백 변형까지 보진 않는다 —
         * 같은 코드베이스가 생성하는 응답이라 형식이 고정이다. */
        if (strstr(reply, "\"result\":\"ok\"") == NULL) {
            (void)snprintf(reason, reason_len, "카메라 거부: %.160s", reply);
            goto out;
        }
        rc = 0;
    }

out:
    if (fd >= 0) {
        (void)close(fd);
    }
    (void)fclose(f);
    return rc;
}

/* ---------------------------------------------------------------------------
 *  모듈 인터페이스
 * ------------------------------------------------------------------------- */
static int cam_init(struct shared_ctx *ctx)
{
    s_ctx = ctx;
    (void)snprintf(s_host, sizeof(s_host), "%s", env_or(ENV_HOST, DEF_HOST));
    s_port      = atoi(env_or(ENV_PORT, "0"));
    s_timeout_s = atoi(env_or(ENV_TIMEOUT, "0"));
    if ((s_port <= 0) || (s_port > 65535)) {
        s_port = DEF_PORT;
    }
    if (s_timeout_s <= 0) {
        s_timeout_s = DEF_TIMEOUT_S;
    }
    s_disabled = (strcmp(env_or(ENV_DISABLE, "0"), "1") == 0);

    (void)fprintf(stderr, "[camera  ] %s  %s:%d (타임아웃 %ds, 최대 %u회)\n",
                  s_disabled ? "업로드 비활성" : "업로드 대상",
                  s_host, s_port, s_timeout_s, MAX_ATTEMPTS);
    return 0;   /* 카메라가 없어도 데몬은 계속 구동한다 */
}

/* 파일 전송은 fd 이벤트가 아니라 상태 전이에서 시작한다 */
static int cam_get_fd(void) { return -1; }

/* cppcheck-suppress constParameterCallback ; on_state 는 daemon_module 콜백 ABI */
static void cam_on_state(struct shared_ctx *ctx,
                         daemon_state_t old_st, daemon_state_t new_st)
{
    (void)old_st;

    if ((new_st != ST_EXPORT) || s_disabled) {
        return;
    }
    /* ⚠️ result.valid 를 반드시 본다. 산출이 실패한 스캔(권한·디스크)에서도
     *   EXPORT 전이는 일어나므로, 안 보면 없는 파일을 올리려 든다. */
    if ((ctx->result.valid == 0u) || (ctx->result.json_path[0] == '\0')) {
        core_log(ctx->core, "CAM", "산출물이 없어 업로드를 건너뛴다");
        return;
    }
    /* 같은 스캔을 두 번 올리지 않는다 — EXPORT 재진입이 생겨도 안전하게. */
    if (ctx->result.point_count == s_last_result_seq) {
        return;
    }

    char reason[192] = {0};
    bool ok = false;

    for (uint32_t attempt = 1u; attempt <= MAX_ATTEMPTS; attempt++) {
        core_log(ctx->core, "CAM", "업로드 시도 %u/%u → %s:%d  %s",
                 attempt, MAX_ATTEMPTS, s_host, s_port,
                 base_name(ctx->result.json_path));

        if (upload_once(ctx->result.json_path, reason, sizeof(reason)) == 0) {
            ok = true;
            break;
        }
        core_log(ctx->core, "CAM", "  실패: %s", reason);
        if (attempt < MAX_ATTEMPTS) {
            (void)usleep(RETRY_DELAY_US);
        }
    }

    if (ok) {
        s_last_result_seq = ctx->result.point_count;
        core_log(ctx->core, "CAM", "✅ 업로드 완료");
    } else {
        /* 파일은 로컬에 남아 있으므로 데이터를 잃지는 않는다. 사람이 나중에
         * 손으로 올릴 수 있도록 경로를 로그에 남긴다. */
        core_log(ctx->core, "CAM",
                 "★ 업로드 실패 (%u회 시도) — 파일은 남아 있다: %s",
                 MAX_ATTEMPTS, ctx->result.json_path);
        notice_post(ctx, NOTICE_UPLOAD_FAIL, 0u, "ERR_UPLOAD", reason);
    }

    /* ★ 반드시 마지막에. 위 루프가 수십 초를 블로킹했으므로, 그 간격을
     *   heartbeat 두절로 오판하지 않게 기준선을 되맞춘다. 안 하면 성공한
     *   스캔마다 link_dead -> DISARM 이 뜬다. */
    core_hb_reprime(ctx->core);
}

static void cam_deinit(struct shared_ctx *ctx)
{
    (void)ctx;
    s_ctx = NULL;
}

static const struct daemon_module k_camera = {
    "camera",
    cam_init,
    cam_get_fd,
    NULL,           /* on_event  — fd 가 없다 */
    NULL,           /* on_tick   — 상태 전이에서만 일한다 */
    cam_on_state,
    cam_deinit,
};

const struct daemon_module *camera_module_get(void)
{
    return &k_camera;
}
