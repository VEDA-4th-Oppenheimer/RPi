/* ============================================================================
 *  camera_module.c  --  스캔 JSON 을 CCTV 앱(CV5 OpenSDK)으로 mTLS 업로드
 *  담당: 이현우 (수신측 lidar_json_receiver = 이영민)
 *
 *  스캔이 끝나면(ST_EXPORT) 원시 측정 JSON 을 카메라의 TLS 서버로 보낸다.
 *  캘리브 연산은 카메라 단이 하고, 여기서는 파일을 건네주는 것까지만 한다.
 *
 *  ── 와이어 프로토콜 (lidar_json_receiver/README.md) ──
 *    한 연결에 파일 하나. TLS 안에 그대로 흐른다 — 프레이밍은 안 바뀐다.
 *      [2바이트 파일명 길이, 네트워크 바이트오더]
 *      [파일명 ASCII]
 *      [8바이트 파일 길이, 네트워크 바이트오더]
 *      [JSON 바이트]
 *    → 응답 JSON 한 줄:  {"result":"ok", ...}  또는  {"result":"error", ...}
 *
 *  ============================================================================
 *  핵심: 주소와 신원을 분리한다 — 이 모듈의 핵심 설계
 *  ============================================================================
 *  카메라도 RPi 도 DHCP 라 IP 가 계속 바뀌고, 둘이 서로 다른 서브넷에 있어
 *  라우터를 거친다. 그래서 mDNS·브로드캐스트 같은 링크로컬 탐색이 전부 안 되고,
 *  사내 DNS 도 없어 이름 등록도 못 한다. 남는 방법은 하나뿐이다:
 *
 *      주소는 **설정 파일**이 알려주고,  신원은 **인증서**가 증명한다.
 *
 *    · 주소 : /etc/adts/camera.conf 의 host. 업로드할 때마다 다시 읽으므로
 *             데몬을 재시작하지 않아도 바뀐 IP 가 즉시 반영된다.
 *    · 신원 : 인증서 SAN 의 고정된 이름(기본 adts-camera)을 SSL_set1_host 로
 *             검증한다. IP 가 매일 바뀌어도 인증서는 다시 발급할 필요가 없다.
 *
 *  주의: 이 분리가 없으면 mTLS 를 붙여도 실익이 없다. IP 로만 상대를 특정하면
 *    DHCP 가 그 주소를 다른 장비에 재할당했을 때 "주소는 맞는데 다른 놈"이
 *    되는데, 그게 정확히 지금 상황에서 일어날 수 있는 일이다.
 *
 *  주의: 평문 폴백은 없다. TLS 를 준비 못 하면 업로드를 **안 한다**. 보안 기능이
 *    조용히 꺼진 채로 도는 것이 실패보다 나쁘다 — 파일은 로컬에 남아 있으므로
 *    잃는 것은 없고, 나중에 손으로 올릴 수 있다.
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
 *  주의: 대신 세 가지를 반드시 지킨다:
 *    ① 소켓 타임아웃 — 카메라가 안 받으면 쓰기가 영원히 안 돌아온다.
 *    ② 끝나면 core_hb_reprime() — 안 하면 블로킹한 시간을 통신 두절로
 *       오판해 매번 DISARM 이 걸린다(그쪽 주석 참조).
 *    ③ SIGPIPE 무시 — 아래 cam_init 주석 참조.
 * ==========================================================================*/
#include "daemon_module.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <signal.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef ADTS_NO_TLS
#include <openssl/err.h>
#include <openssl/ssl.h>
#include <openssl/x509v3.h>
#endif

/* ── 설정 ────────────────────────────────────────────────────────────────────
 *  우선순위:  설정 파일  >  환경변수  >  내장 기본값
 *
 *  주의: 파일이 환경변수를 **이긴다**. 반대로 두면 systemd 유닛의 Environment=
 *    가 항상 이겨서, 파일을 고쳐도 아무 일이 안 일어난다("고쳤는데 왜 그대로지").
 *    환경변수는 파일이 없을 때의 기본값 역할만 한다.
 */
#define ENV_CONF      "ADTS_CAM_CONF"
#define ENV_HOST      "ADTS_CAM_HOST"
#define ENV_PORT      "ADTS_CAM_PORT"
#define ENV_NAME      "ADTS_CAM_NAME"
#define ENV_TIMEOUT   "ADTS_CAM_TIMEOUT_S"
#define ENV_DISABLE   "ADTS_CAM_DISABLE"     /* "1" 이면 업로드 자체를 끈다 */
#define ENV_CA        "ADTS_CAM_CA"
#define ENV_CERT      "ADTS_CAM_CERT"
#define ENV_KEY       "ADTS_CAM_KEY"

#define DEF_CONF      "/etc/adts/camera.conf"
#define DEF_HOST      "172.20.32.43"        /* DHCP 라 바뀐다 — conf 로 고친다 */
#define DEF_PORT      2222                  /* 수신 앱이 고정으로 여는 포트 */
#define DEF_NAME      "adts-camera"          /* 카메라 인증서 SAN 의 DNS 이름 */

/* MQTT 와 같은 CA·같은 클라이언트 인증서를 쓴다. 신뢰 뿌리가 하나여야
 * "누가 누구를 믿는가"가 한 장으로 설명된다. */
#define DEF_CA        "/etc/adts/certs/ca.crt"
#define DEF_CERT      "/etc/adts/certs/daemon.crt"
#define DEF_KEY       "/etc/adts/certs/daemon.key"

/* 주의: 무선 기준. JSON 이 20~25MB 인데 Wi-Fi 실효 대역이 흔들리면 수십 초가
 *   걸린다. 이 값은 **한 번의 읽기/쓰기가 멈춰 있을 수 있는 한도**이지 전체
 *   전송 시간이 아니다(진행 중이면 타이머가 갱신된다). 그래도 넉넉히 둔다. */
#define DEF_TIMEOUT_S  60

#define MAX_ATTEMPTS   3u        /* 사용자 요청: 최대 3회 */
#define RETRY_DELAY_US 2000000u  /* 재시도 사이 2초 — 순간적 무선 끊김 대비 */

#define CHUNK_BYTES    (256u * 1024u)
#define MAX_FILE_BYTES (64ull * 1024ull * 1024ull)   /* 수신측 상한과 동일 */
#define MAX_REPLY      2048u
#define PATH_MAX_LEN   256u
#define HOST_MAX_LEN   128u

static struct shared_ctx *s_ctx;
static char     s_conf[PATH_MAX_LEN];
static char     s_host[HOST_MAX_LEN];
static char     s_name[HOST_MAX_LEN];    /* 인증서로 검증할 이름 */
static char     s_ca[PATH_MAX_LEN];
static char     s_cert[PATH_MAX_LEN];
static char     s_key[PATH_MAX_LEN];
static int      s_port;
static int      s_timeout_s;
static bool     s_disabled;
/* 같은 스캔을 두 번 올리지 않기 위한 표식 = **직전에 올린 JSON 경로**.
 *
 * 주의: 예전엔 point_count 를 썼는데 그게 틀렸다. 고정 격자 스캔은 유효 셀 수가
 *   회차마다 거의 같으므로, 두 번째 스캔부터 "이미 올렸다" 로 오인해 업로드가
 *   조용히 사라졌다(로그도 안 남는다). 파일명에는 시작 시각이 들어 있어 회차
 *   구분이 실제로 되고, 재전송을 막아야 할 대상 자체가 "이 파일" 이라 의미도
 *   정확하다. */
static char s_last_uploaded[PATH_MAX_LEN];

#ifndef ADTS_NO_TLS
static SSL_CTX *s_tls;
#endif

/* ---------------------------------------------------------------------------
 *  보조
 * ------------------------------------------------------------------------- */
static const char *env_or(const char *name, const char *def)
{
    const char *v = getenv(name);
    return ((v != NULL) && (v[0] != '\0')) ? v : def;
}

static void copy_str(char *dst, size_t cap, const char *src)
{
    (void)snprintf(dst, cap, "%s", src);
}

/* 경로에서 파일명만. 수신측이 이 이름 그대로 저장한다. */
static const char *base_name(const char *path)
{
    const char *slash = strrchr(path, '/');
    return (slash != NULL) ? (slash + 1) : path;
}

#ifndef ADTS_NO_TLS
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
#endif /* !ADTS_NO_TLS */

/* ---------------------------------------------------------------------------
 *  설정 파일
 *
 *    # 주석
 *    host      = 172.20.32.43
 *    port      = 2222
 *    name      = adts-camera
 *    timeout_s = 60
 *    disable   = 0
 *    ca / cert / key = <경로>
 *
 *  주의: JSON 이 아니라 key = value 다. cJSON 은 이 빌드에서 **선택 의존성**이라
 *    (맥 크로스 sysroot 에 없다) 설정 읽기가 거기 묶이면 안 된다. 그리고
 *    현장에서 vi 로 한 줄 고치는 파일이라 콤마·따옴표가 없는 편이 안전하다.
 * ------------------------------------------------------------------------- */
static char *trim(char *s)
{
    char *end;

    while ((*s == ' ') || (*s == '\t')) {
        s++;
    }
    end = s + strlen(s);
    while (end > s) {
        const char c = *(end - 1);
        if ((c == ' ') || (c == '\t') || (c == '\r') || (c == '\n')) {
            end--;
        } else {
            break;
        }
    }
    *end = '\0';
    return s;
}

static void apply_kv(const char *k, const char *v)
{
    if (strcmp(k, "host") == 0) {
        copy_str(s_host, sizeof(s_host), v);
    } else if (strcmp(k, "name") == 0) {
        copy_str(s_name, sizeof(s_name), v);
    } else if (strcmp(k, "ca") == 0) {
        copy_str(s_ca, sizeof(s_ca), v);
    } else if (strcmp(k, "cert") == 0) {
        copy_str(s_cert, sizeof(s_cert), v);
    } else if (strcmp(k, "key") == 0) {
        copy_str(s_key, sizeof(s_key), v);
    } else if (strcmp(k, "port") == 0) {
        const int p = atoi(v);
        if ((p > 0) && (p <= 65535)) {
            s_port = p;
        }
    } else if (strcmp(k, "timeout_s") == 0) {
        const int t = atoi(v);
        if (t > 0) {
            s_timeout_s = t;
        }
    } else if (strcmp(k, "disable") == 0) {
        s_disabled = (atoi(v) != 0);
    } else {
        /* 모르는 키는 조용히 무시한다 — 설정 파일에 메모를 남길 수 있게. */
    }
}

/* 환경변수/기본값으로 초기화한 뒤 설정 파일로 덮어쓴다.
 * 반환: 직전 호출과 비교해 주소·포트·이름이 달라졌으면 true. */
static bool load_config(void)
{
    char prev_host[HOST_MAX_LEN];
    char prev_name[HOST_MAX_LEN];
    const int prev_port = s_port;
    FILE *f;
    char line[512];

    copy_str(prev_host, sizeof(prev_host), s_host);
    copy_str(prev_name, sizeof(prev_name), s_name);

    copy_str(s_host, sizeof(s_host), env_or(ENV_HOST, DEF_HOST));
    copy_str(s_name, sizeof(s_name), env_or(ENV_NAME, DEF_NAME));
    copy_str(s_ca,   sizeof(s_ca),   env_or(ENV_CA,   DEF_CA));
    copy_str(s_cert, sizeof(s_cert), env_or(ENV_CERT, DEF_CERT));
    copy_str(s_key,  sizeof(s_key),  env_or(ENV_KEY,  DEF_KEY));
    s_port      = atoi(env_or(ENV_PORT, "0"));
    s_timeout_s = atoi(env_or(ENV_TIMEOUT, "0"));
    s_disabled  = (strcmp(env_or(ENV_DISABLE, "0"), "1") == 0);
    if ((s_port <= 0) || (s_port > 65535)) {
        s_port = DEF_PORT;
    }
    if (s_timeout_s <= 0) {
        s_timeout_s = DEF_TIMEOUT_S;
    }

    f = fopen(s_conf, "r");
    if (f != NULL) {
        while (fgets(line, (int)sizeof(line), f) != NULL) {
            char *p = trim(line);
            char *eq;

            if ((*p == '\0') || (*p == '#')) {
                continue;
            }
            eq = strchr(p, '=');
            if (eq == NULL) {
                continue;
            }
            *eq = '\0';
            apply_kv(trim(p), trim(eq + 1));
        }
        (void)fclose(f);
    }

    return (strcmp(prev_host, s_host) != 0)
        || (strcmp(prev_name, s_name) != 0)
        || (prev_port != s_port);
}

/* ---------------------------------------------------------------------------
 *  TLS
 * ------------------------------------------------------------------------- */
#ifndef ADTS_NO_TLS
static void ssl_err_str(char *out, size_t cap)
{
    const unsigned long e = ERR_peek_last_error();

    if (e == 0uL) {
        (void)snprintf(out, cap, "%s", strerror(errno));
    } else {
        char buf[112];
        ERR_error_string_n(e, buf, sizeof(buf));
        (void)snprintf(out, cap, "%s", buf);
    }
    ERR_clear_error();
}

/* SSL_CTX 는 한 번 만들어 재사용한다. 실패하면 NULL 을 남기고, 다음 업로드
 * 때 다시 시도한다(그 사이에 인증서를 설치했을 수 있다). */
static bool tls_ctx_ready(char *reason, size_t reason_len)
{
    SSL_CTX *c;

    if (s_tls != NULL) {
        return true;
    }

    c = SSL_CTX_new(TLS_client_method());
    if (c == NULL) {
        (void)snprintf(reason, reason_len, "SSL_CTX_new 실패");
        return false;
    }
    /* TLS 1.2 미만은 받지 않는다. */
    (void)SSL_CTX_set_min_proto_version(c, TLS1_2_VERSION);
    (void)SSL_CTX_set_mode(c, SSL_MODE_AUTO_RETRY);

    if (SSL_CTX_load_verify_locations(c, s_ca, NULL) != 1) {
        (void)snprintf(reason, reason_len, "CA 읽기 실패: %.180s", s_ca);
        SSL_CTX_free(c);
        return false;
    }
    if (SSL_CTX_use_certificate_chain_file(c, s_cert) != 1) {
        (void)snprintf(reason, reason_len, "인증서 읽기 실패: %.180s", s_cert);
        SSL_CTX_free(c);
        return false;
    }
    /* 주의: 키는 0640 root:adts 다. 데몬이 그 그룹에 없으면 여기서 걸린다 —
     *   "권한" 이라고 분명히 말해주지 않으면 원인을 찾는 데 한참 걸린다. */
    if (SSL_CTX_use_PrivateKey_file(c, s_key, SSL_FILETYPE_PEM) != 1) {
        (void)snprintf(reason, reason_len, "키 읽기 실패(권한 확인): %.180s", s_key);
        SSL_CTX_free(c);
        return false;
    }
    if (SSL_CTX_check_private_key(c) != 1) {
        (void)snprintf(reason, reason_len, "인증서와 키가 짝이 아니다");
        SSL_CTX_free(c);
        return false;
    }
    /* 상대(카메라) 인증서를 반드시 검증한다. 이름 검증은 연결마다 건다. */
    SSL_CTX_set_verify(c, SSL_VERIFY_PEER, NULL);
    SSL_CTX_set_verify_depth(c, 4);

    s_tls = c;
    return true;
}

/* connect 에 우리 타임아웃을 건다. 성공하면 true, 그 자리에서 fd 는 다시
 * 블로킹 모드로 돌아간다.
 *
 * 주의: SO_SNDTIMEO/SO_RCVTIMEO 는 **connect 에 안 걸린다.** 그 둘은 연결이
 *   성립된 소켓의 읽기/쓰기에만 적용된다. 그래서 블로킹 connect 는 커널
 *   기본 재시도가 끝날 때까지 돌아오지 않는데, 리눅스 기본 tcp_syn_retries=6
 *   이면 응답 없는 주소에서 약 127초다. 재시도 3회면 6분 넘게 걸리고, 데몬은
 *   단일 스레드 epoll 이라 그동안 MQTT keepalive·EMS 스위치·STM heartbeat 가
 *   전부 멈춘다. 상대가 RST 를 주면(앱만 안 떠 있는 경우) 즉시 실패하므로
 *   평소엔 안 드러나고, 카메라 전원이 빠지거나 IP 가 옮겨간 날 터진다.
 *
 * 주의: select 가 쓰기 가능으로 깨워도 성공이 아니다 — 실패한 소켓도 쓰기
 *   가능으로 깨어난다. 실제 결과는 SO_ERROR 에만 있다. 이걸 빼면 죽은
 *   연결에 TLS 핸드셰이크를 시도하게 된다.
 *
 * 주의: 연결 뒤 반드시 블로킹으로 되돌린다. 논블로킹인 채로 두면 OpenSSL 이
 *   WANT_READ/WANT_WRITE 를 뱉어서 전송 루프를 통째로 다시 써야 한다. */
static bool connect_with_deadline(int fd, const struct sockaddr *sa, socklen_t len)
{
    const int flags = fcntl(fd, F_GETFL, 0);
    fd_set  wset;
    struct timeval tv;
    int     err  = 0;
    socklen_t elen = sizeof(err);
    bool    ok   = false;

    if ((flags < 0) || (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0)) {
        return false;
    }

    if (connect(fd, sa, len) == 0) {
        ok = true;
    } else if (errno == EINPROGRESS) {
        /* select 는 fd 가 FD_SETSIZE(1024) 미만이어야 한다. 데몬이 여는 fd 는
         * 십여 개뿐이라 넘을 일이 없지만, 넘으면 스택을 밟으므로 막아둔다. */
        if (fd < FD_SETSIZE) {
            FD_ZERO(&wset);
            FD_SET(fd, &wset);
            memset(&tv, 0, sizeof(tv));
            tv.tv_sec = s_timeout_s;

            if (select(fd + 1, NULL, &wset, NULL, &tv) > 0) {
                if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &elen) == 0) {
                    ok = (err == 0);
                }
            }
        }
    } else {
        ok = false;
    }

    if (fcntl(fd, F_SETFL, flags) < 0) {
        ok = false;
    }
    return ok;
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
        if (connect_with_deadline(fd, a->ai_addr, a->ai_addrlen)) {
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
        /* 주의: 이게 없으면 카메라가 응답을 멈췄을 때 데몬이 영원히 멈춘다.
         *   scan_batch.sh 의 회차 제한시간에 걸릴 때까지 아무 일도 안 한다. */
        (void)setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        (void)setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }
    return fd;
}

/* 전부 보낼 때까지 반복. */
static bool ssl_send_all(SSL *ssl, const void *buf, size_t len)
{
    const unsigned char *p = buf;
    size_t sent = 0u;

    while (sent < len) {
        const int want = (int)((len - sent) > 0x40000000u ? 0x40000000u
                                                          : (len - sent));
        const int n = SSL_write(ssl, p + sent, want);
        if (n > 0) {
            sent += (size_t)n;
        } else {
            return false;
        }
    }
    return true;
}

/* 핸드셰이크까지 끝난 SSL* 를 돌려준다. 실패 시 NULL + reason.
 *
 * 핵심: 여기가 이 모듈의 요점이다. connect 는 s_host(그때그때의 IP)로 하지만
 *   검증은 s_name(고정된 이름)으로 한다. */
static SSL *tls_handshake(int fd, char *reason, size_t reason_len)
{
    SSL *ssl = SSL_new(s_tls);
    X509_VERIFY_PARAM *param;

    if (ssl == NULL) {
        (void)snprintf(reason, reason_len, "SSL_new 실패");
        return NULL;
    }

    param = SSL_get0_param(ssl);
    X509_VERIFY_PARAM_set_hostflags(param, X509_CHECK_FLAG_NO_PARTIAL_WILDCARDS);
    if (SSL_set1_host(ssl, s_name) != 1) {
        (void)snprintf(reason, reason_len, "검증 이름 설정 실패: %s", s_name);
        SSL_free(ssl);
        return NULL;
    }
    /* SNI 도 이름으로 보낸다 — 카메라가 여러 인증서를 갖게 되어도 맞는 것을
     * 고를 수 있다. IP 를 SNI 로 보내는 것은 규격상 틀리다. */
    (void)SSL_set_tlsext_host_name(ssl, s_name);

    if (SSL_set_fd(ssl, fd) != 1) {
        (void)snprintf(reason, reason_len, "SSL_set_fd 실패");
        SSL_free(ssl);
        return NULL;
    }

    if (SSL_connect(ssl) != 1) {
        /* 주의: 접속 실패와 신원 불일치를 반드시 구분해서 알린다. 전자는
         *   "주소가 틀렸다"(설정 파일을 고쳐라), 후자는 "다른 장비다"
         *   (DHCP 가 그 IP 를 남에게 줬거나 인증서가 안 맞다)라서 대응이
         *   완전히 다르다. 뭉뚱그리면 엉뚱한 데를 고치게 된다. */
        const long vr = SSL_get_verify_result(ssl);
        if (vr != X509_V_OK) {
            (void)snprintf(reason, reason_len,
                           "신원 불일치 — %s (기대한 이름 '%s')",
                           X509_verify_cert_error_string(vr), s_name);
        } else {
            char e[128];
            ssl_err_str(e, sizeof(e));
            (void)snprintf(reason, reason_len, "TLS 핸드셰이크 실패: %s", e);
        }
        SSL_free(ssl);
        return NULL;
    }
    return ssl;
}
#endif /* !ADTS_NO_TLS */

/* 한 번 시도. reason 에 실패 사유를 남긴다.
 *
 *   UP_OK      0   성공
 *   UP_RETRY  -1   일시적일 수 있다 — 다시 해볼 가치가 있다
 *   UP_FATAL  -2   다시 해도 같다 — 즉시 그만두고 이 사유를 보고한다
 *
 * 주의: 이 구분이 없으면 **엉뚱한 사유가 보고된다.** 신원 불일치로 실패한 뒤
 *   2초 있다 또 붙으면 상대가 그새 사라져 "Connection refused" 가 나고,
 *   마지막 사유만 남으니 Qt 에는 "접속 실패" 로 뜬다. 실제로는 인증서
 *   문제인데 사람은 주소를 고치러 간다(시험에서 그대로 재현됐다).
 *
 *   인증서·이름·파일 문제는 2초 뒤에도 똑같다. 재시도는 무선이 잠깐 끊기는
 *   경우에만 의미가 있다.
 */
#define UP_OK     0
#define UP_RETRY  (-1)
#define UP_FATAL  (-2)

#ifdef ADTS_NO_TLS
static int upload_once(const char *path, char *reason, size_t reason_len)
{
    (void)path;
    (void)snprintf(reason, reason_len,
                   "이 바이너리는 OpenSSL 없이 빌드됐다 — 업로드 불가");
    return UP_FATAL;
}
#else
static int upload_once(const char *path, char *reason, size_t reason_len)
{
    struct stat st;
    FILE *f = NULL;
    SSL *ssl = NULL;
    int fd = -1;
    int rc = UP_RETRY;

    if (!tls_ctx_ready(reason, reason_len)) {
        return UP_FATAL;   /* 인증서 파일 문제 — 2초 뒤에도 같다 */
    }

    if ((stat(path, &st) != 0) || !S_ISREG(st.st_mode)) {
        (void)snprintf(reason, reason_len, "파일 없음: %s", path);
        return UP_FATAL;
    }
    if ((st.st_size <= 0) || ((uint64_t)st.st_size > MAX_FILE_BYTES)) {
        (void)snprintf(reason, reason_len, "크기 범위 밖: %lld B",
                       (long long)st.st_size);
        return UP_FATAL;
    }

    const char *name = base_name(path);
    if (!name_is_acceptable(name)) {
        (void)snprintf(reason, reason_len, "수신측이 거부할 파일명: %s", name);
        return UP_FATAL;
    }

    f = fopen(path, "rb");
    if (f == NULL) {
        (void)snprintf(reason, reason_len, "열기 실패: %s", strerror(errno));
        return UP_FATAL;
    }

    fd = connect_camera();
    if (fd < 0) {
        (void)snprintf(reason, reason_len, "접속 실패 %s:%d (%s)",
                       s_host, s_port, strerror(errno));
        (void)fclose(f);
        return UP_RETRY;   /* 주소가 살아날 수도, 무선이 돌아올 수도 */
    }

    ssl = tls_handshake(fd, reason, reason_len);
    if (ssl == NULL) {
        /* 신원이 안 맞거나 인증서가 안 맞는다 — 재시도해도 같다. */
        (void)close(fd);
        (void)fclose(f);
        return UP_FATAL;
    }

    /* ── 헤더 ── 길이는 전부 네트워크 바이트오더(big endian) ── */
    const uint16_t nlen_be = htons((uint16_t)strlen(name));
    unsigned char size_be[8];
    uint64_t sz = (uint64_t)st.st_size;
    for (size_t i = 0u; i < 8u; i++) {
        size_be[7u - i] = (unsigned char)(sz & 0xFFu);
        sz >>= 8u;
    }

    if (!ssl_send_all(ssl, &nlen_be, sizeof(nlen_be))
     || !ssl_send_all(ssl, name, strlen(name))
     || !ssl_send_all(ssl, size_be, sizeof(size_be))) {
        char e[128];
        ssl_err_str(e, sizeof(e));
        (void)snprintf(reason, reason_len, "헤더 전송 실패: %s", e);
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
            if (!ssl_send_all(ssl, buf, got)) {
                char e[128];
                ssl_err_str(e, sizeof(e));
                (void)snprintf(reason, reason_len,
                               "본문 전송 실패 (%llu/%lld B): %s",
                               (unsigned long long)total,
                               (long long)st.st_size, e);
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
            const int n = SSL_read(ssl, reply + used,
                                   (int)((sizeof(reply) - 1u) - used));
            if (n > 0) {
                used += (size_t)n;
                reply[used] = '\0';
                if (strchr(reply, '\n') != NULL) {
                    break;
                }
                if (used >= (sizeof(reply) - 1u)) {
                    break;
                }
            } else if (used > 0u) {
                break;                     /* 상대가 닫음 — 받은 만큼으로 판정 */
            } else {
                char e[128];
                ssl_err_str(e, sizeof(e));
                (void)snprintf(reason, reason_len, "응답 없음: %s", e);
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
        rc = UP_OK;
    }

out:
    if (ssl != NULL) {
        (void)SSL_shutdown(ssl);
        SSL_free(ssl);
    }
    if (fd >= 0) {
        (void)close(fd);
    }
    (void)fclose(f);
    return rc;
}
#endif /* ADTS_NO_TLS */

/* ---------------------------------------------------------------------------
 *  모듈 인터페이스
 * ------------------------------------------------------------------------- */
static int cam_init(struct shared_ctx *ctx)
{
    s_ctx = ctx;
    copy_str(s_conf, sizeof(s_conf), env_or(ENV_CONF, DEF_CONF));
    (void)load_config();

    /* 주의: SIGPIPE 를 끈다. 예전엔 send(MSG_NOSIGNAL) 로 막았지만 OpenSSL 은
     *   내부에서 write(2) 를 부르므로 그 플래그를 넘길 방법이 없다. 상대가
     *   먼저 끊은 소켓에 쓰면 **데몬이 그 자리에서 죽는다** — 스캔 산출물을
     *   들고 있는 상태라 절대 죽으면 안 된다. 프로세스 전역 설정이지만
     *   데몬 어디에도 SIGPIPE 로 죽어야 할 코드가 없다. */
    (void)signal(SIGPIPE, SIG_IGN);

#ifdef ADTS_NO_TLS
    /* 평문으로 되돌리지 않는다(파일 머리말 참조). 여기서 끝낸다. */
    (void)fprintf(stderr,
                  "[camera  ] 주의: OpenSSL 없이 빌드됨 — 업로드 비활성\n");
    s_disabled = true;
    return 0;
#else
    {
        char reason[224] = {0};
        if (!tls_ctx_ready(reason, sizeof(reason))) {
            /* 여기서 실패해도 데몬은 계속 뜬다. 인증서를 나중에 설치하고
             * 재시작 없이 이어갈 수 있게, 업로드 때 다시 시도한다. */
            (void)fprintf(stderr, "[camera  ] 주의: TLS 준비 실패: %s\n", reason);
        }
    }
#endif

    (void)fprintf(stderr,
                  "[camera  ] %s  %s:%d  (검증 이름 '%s', 타임아웃 %ds, 최대 %u회)\n",
                  s_disabled ? "업로드 비활성" : "업로드 대상",
                  s_host, s_port, s_name, s_timeout_s, MAX_ATTEMPTS);
    (void)fprintf(stderr, "[camera  ] 설정 파일: %s (업로드마다 다시 읽는다)\n",
                  s_conf);
    return 0;   /* 카메라가 없어도 데몬은 계속 구동한다 */
}

/* 파일 전송은 fd 이벤트가 아니라 상태 전이에서 시작한다 */
static int cam_get_fd(void) { return -1; }

/* cppcheck-suppress constParameterCallback ; on_state 는 daemon_module 콜백 ABI */
static void cam_on_state(struct shared_ctx *ctx,
                         daemon_state_t old_st, daemon_state_t new_st)
{
    (void)old_st;

    if (new_st != ST_EXPORT) {
        return;
    }
    /* 주의: result.valid 를 반드시 본다. 산출이 실패한 스캔(권한·디스크)에서도
     *   EXPORT 전이는 일어나므로, 안 보면 없는 파일을 올리려 든다. */
    if ((ctx->result.valid == 0u) || (ctx->result.json_path[0] == '\0')) {
        core_log(ctx->core, "CAM", "산출물이 없어 업로드를 건너뛴다");
        return;
    }
    /* 같은 파일을 두 번 올리지 않는다 — EXPORT 재진입이 생겨도 안전하게. */
    if (strcmp(ctx->result.json_path, s_last_uploaded) == 0) {
        core_log(ctx->core, "CAM", "이미 올린 파일이라 건너뛴다");
        return;
    }

    char reason[224] = {0};
    bool ok = false;

    for (uint32_t attempt = 1u; attempt <= MAX_ATTEMPTS; attempt++) {
        /* 핵심: 시도할 때마다 설정을 다시 읽는다. 데몬을 재시작하지 않아도 되고,
         *   재시도 사이(최대 수십 초)에 사람이 주소를 고치면 바로 반영된다.
         *   같은 죽은 주소를 세 번 두드리는 것보다 훨씬 낫다. */
        if (load_config() && (attempt > 1u)) {
            core_log(ctx->core, "CAM", "설정이 바뀌었다 → %s:%d ('%s')",
                     s_host, s_port, s_name);
        }
        if (s_disabled) {
            core_log(ctx->core, "CAM", "업로드가 비활성이라 건너뛴다");
            return;
        }

        core_log(ctx->core, "CAM", "업로드 시도 %u/%u → %s:%d  %s",
                 attempt, MAX_ATTEMPTS, s_host, s_port,
                 base_name(ctx->result.json_path));

        const int rc = upload_once(ctx->result.json_path, reason, sizeof(reason));
        if (rc == UP_OK) {
            ok = true;
            break;
        }
        core_log(ctx->core, "CAM", "  실패: %s", reason);
        if (rc == UP_FATAL) {
            /* 재시도해도 같은 결과다. 여기서 멈춰야 **이 사유가** 보고된다 —
             * 더 두드리면 뒤늦은 "접속 실패" 가 진짜 원인을 덮어쓴다. */
            core_log(ctx->core, "CAM", "  (재시도해도 같은 결과라 중단한다)");
            break;
        }
        if (attempt < MAX_ATTEMPTS) {
            (void)usleep(RETRY_DELAY_US);
        }
    }

    if (ok) {
        copy_str(s_last_uploaded, sizeof(s_last_uploaded), ctx->result.json_path);
        core_log(ctx->core, "CAM", " 업로드 완료 (mTLS)");
    } else {
        /* 파일은 로컬에 남아 있으므로 데이터를 잃지는 않는다. 사람이 나중에
         * 손으로 올릴 수 있도록 경로를 로그에 남긴다. */
        core_log(ctx->core, "CAM",
                 "핵심: 업로드 실패 — 파일은 남아 있다: %s",
                 ctx->result.json_path);
        core_log(ctx->core, "CAM",
                 "  주소가 바뀌었으면 %s 의 host 를 고치면 된다(재시작 불필요)",
                 s_conf);
        notice_post(ctx, NOTICE_UPLOAD_FAIL, 0u, "ERR_UPLOAD", reason);
    }

    /* 핵심: 반드시 마지막에. 위 루프가 수십 초를 블로킹했으므로, 그 간격을
     *   heartbeat 두절로 오판하지 않게 기준선을 되맞춘다. 안 하면 성공한
     *   스캔마다 link_dead -> DISARM 이 뜬다. */
    core_hb_reprime(ctx->core);
}

static void cam_deinit(struct shared_ctx *ctx)
{
    (void)ctx;
#ifndef ADTS_NO_TLS
    if (s_tls != NULL) {
        SSL_CTX_free(s_tls);
        s_tls = NULL;
    }
#endif
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
