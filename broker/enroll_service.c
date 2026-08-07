/* ============================================================================
 *  enroll_service.c — Qt 관제 콘솔 인증서 발급 서비스 (HTTPS POST /enroll)
 * ----------------------------------------------------------------------------
 *  Qt 배포본에는 인증서도 카메라 설정도 담지 않는다. 인증서에는 adts/cmd/# 쓰기
 *  권한이 있어 장비를 움직일 수 있고, 카메라 설정에는 카메라 admin 비밀번호가
 *  RTSP URL 에 박혀 있다. 배포물에 넣으면 받은 사람 전원이 그 권한을 갖는다.
 *
 *  대신 사용자가 1회용 토큰을 입력하면 이 서비스가 인증서·설정을 한 번에 내려준다.
 *
 *  ── 설계 메모 ────────────────────────────────────────────────────────────
 *  · 언어: C. 프로젝트 전체가 C/C++ 로 통일돼 있다.
 *  · TLS: OpenSSL. 데몬의 MQTT-over-TLS 와 같은 스택을 쓴다(OpenSSL 고정 요건).
 *  · JSON: cJSON. 데몬(mqtt_module.c)이 이미 쓰는 것과 동일.
 *  · 서명: gen-certs.sh --client 를 fork/exec 로 호출한다. 서명 로직을 여기서
 *    다시 구현하면 확장(v3_client)이나 키 포맷(전통 RSA) 같은 세부가 갈라진다.
 *  · 동시성: 단일 스레드로 한 연결씩 처리한다. 발급은 사람이 하루 몇 번 하는
 *    작업이라 성능이 문제되지 않고, 토큰 파일·ACL 파일을 동시에 건드릴 일이
 *    없어 잠금 없이 안전하다.
 *  · 권한: CA 개인키를 읽어야 하므로 root 로 돈다(adts-enroll.service 참고).
 *    CA 키는 이 장비 밖으로 나가지 않는다 — 서명은 전부 여기서 이뤄진다.
 *
 *  클라이언트 인증서는 요구하지 않는다. 아직 인증서가 없는 사람이 받으러 오는
 *  곳이기 때문이다. 신원 확인은 1회용 토큰이 하고, 반대로 Qt 는 실행파일에
 *  박아둔 ca.crt 로 이 서버를 검증한다.
 * ==========================================================================*/
/* strcasestr 는 GNU 확장이다 — 헤더보다 먼저 정의해야 한다. */
#define _GNU_SOURCE

#include <arpa/inet.h>
#include <errno.h>
#include <dirent.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include <cjson/cJSON.h>
#include <openssl/crypto.h>
#include <openssl/err.h>
#include <openssl/ssl.h>

/* ── 기본 설정 (환경변수로 덮는다) ──────────────────────────────────────── */
#define DEF_CERT_DIR    "/etc/adts/certs"
#define DEF_TOKEN_FILE  "/etc/adts/enroll_tokens"
#define DEF_CAMERA_FILE "/etc/adts/cameras.json"
#define DEF_ACL_FILE    "/etc/mosquitto/conf.d/adts.acl"
#define DEF_GEN_CERTS   "/opt/adts/gen-certs.sh"
#define DEF_SYSTEMCTL   "/bin/systemctl"
#define DEF_SCAN_DIR    "/home/pi/final_project/scans"
#define DEF_BIND_PORT   8443
#define DEF_MQTT_PORT   8883

#define MAX_HEADER      8192u
#define MAX_BODY        8192u
#define MAX_PEM         (64u * 1024u)
#define MAX_LABEL       64u
#define MAX_CN          128u

static const char *g_cert_dir;
static const char *g_token_file;
static const char *g_camera_file;
static const char *g_acl_file;
static const char *g_scan_dir;
static const char *g_gen_certs;
static const char *g_systemctl;   /* 경로를 빼둔 이유: 배포판마다 위치가 다르고, 테스트에서 대체할 수 있어야 한다 */
static const char *g_mqtt_host;      /* 비면 요청의 Host 헤더를 쓴다 */
static int         g_mqtt_port;

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int sig) { (void)sig; g_stop = 1; }

static void logmsg(const char *level, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

static void logmsg(const char *level, const char *fmt, ...)
{
    char      ts[32];
    time_t    now = time(NULL);
    struct tm tmv;
    va_list   ap;

    (void)localtime_r(&now, &tmv);
    (void)strftime(ts, sizeof ts, "%Y-%m-%d %H:%M:%S", &tmv);
    (void)fprintf(stderr, "%s %-7s ", ts, level);

    va_start(ap, fmt);
    (void)vfprintf(stderr, fmt, ap);
    va_end(ap);
    (void)fputc('\n', stderr);
}

static const char *env_or(const char *name, const char *fallback)
{
    const char *v = getenv(name);
    return (v != NULL && v[0] != '\0') ? v : fallback;
}

/* CN 은 파일명과 ACL 한 줄에 그대로 들어간다. 문자 집합을 좁혀 경로 조작이나
 * ACL 구문 훼손을 막는다. */
static bool cn_is_safe(const char *s)
{
    if (s == NULL || s[0] == '\0') { return false; }
    for (const char *p = s; *p != '\0'; ++p) {
        const bool ok = (*p >= 'A' && *p <= 'Z') || (*p >= 'a' && *p <= 'z') ||
                        (*p >= '0' && *p <= '9') || *p == '.' || *p == '_' || *p == '-';
        if (!ok) { return false; }
    }
    return true;
}

/* 파일 전체를 NUL 종료 문자열로 읽는다. 실패하면 NULL. */
static char *read_file(const char *path, size_t limit)
{
    FILE *f = fopen(path, "rb");
    if (f == NULL) { return NULL; }

    if (fseek(f, 0, SEEK_END) != 0) { (void)fclose(f); return NULL; }
    const long sz = ftell(f);
    if (sz < 0 || (size_t)sz > limit) { (void)fclose(f); return NULL; }
    rewind(f);

    char *buf = malloc((size_t)sz + 1u);
    if (buf == NULL) { (void)fclose(f); return NULL; }

    const size_t got = fread(buf, 1, (size_t)sz, f);
    (void)fclose(f);
    buf[got] = '\0';
    return buf;
}

/* ── 토큰 ────────────────────────────────────────────────────────────────
 * 파일 형식: '<토큰> <라벨>' 한 줄에 하나. '#' 주석과 빈 줄은 무시.
 * 라벨이 CN 접미사가 된다 — 관리자가 누구에게 준 토큰인지 통제할 수 있게.
 *
 * 검증에 성공하면 그 줄을 **지우고** 파일을 원자적으로 교체한다(1회용).
 * 비교는 CRYPTO_memcmp 로 한다 — 비교에 걸리는 시간으로 토큰을 알아내는 것을
 * 막기 위해서다.
 * ---------------------------------------------------------------------- */
static bool consume_token(const char *token, char *label_out, size_t label_sz)
{
    char *body = read_file(g_token_file, 1u << 20);
    if (body == NULL) {
        logmsg("ERROR", "토큰 파일을 읽지 못했습니다: %s", g_token_file);
        return false;
    }

    char  tmp_path[512];
    (void)snprintf(tmp_path, sizeof tmp_path, "%s.tmp", g_token_file);
    FILE *out = fopen(tmp_path, "w");
    if (out == NULL) {
        logmsg("ERROR", "임시 토큰 파일을 열지 못했습니다: %s", tmp_path);
        free(body);
        return false;
    }
    (void)fprintf(out, "# 1회용 발급 토큰 — '<토큰> <라벨>' 한 줄에 하나."
                       " 사용되면 자동으로 지워진다.\n");

    const size_t tok_len = strlen(token);
    bool         matched = false;
    char        *save1   = NULL;

    for (const char *line = strtok_r(body, "\n", &save1); line != NULL;
         line = strtok_r(NULL, "\n", &save1)) {

        if (line[0] == '#' || line[0] == '\0' || line[0] == '\r') { continue; }

        char *save2 = NULL;
        char  copy[512];
        (void)snprintf(copy, sizeof copy, "%s", line);

        const char *tok   = strtok_r(copy, " \t\r", &save2);
        const char *label = strtok_r(NULL, " \t\r", &save2);
        if (tok == NULL || label == NULL) {
            logmsg("WARN", "토큰 파일에 형식이 잘못된 줄이 있어 건너뜁니다");
            continue;
        }

        const bool same = !matched && strlen(tok) == tok_len &&
                          CRYPTO_memcmp(tok, token, tok_len) == 0;
        if (same) {
            matched = true;
            (void)snprintf(label_out, label_sz, "%s", label);
            continue;                       /* 소비 — 새 파일에 남기지 않는다 */
        }
        (void)fprintf(out, "%s %s\n", tok, label);
    }

    free(body);
    if (fclose(out) != 0) {
        logmsg("ERROR", "임시 토큰 파일을 닫지 못했습니다");
        (void)unlink(tmp_path);
        return false;
    }

    if (!matched) { (void)unlink(tmp_path); return false; }

    (void)chmod(tmp_path, S_IRUSR | S_IWUSR);
    if (rename(tmp_path, g_token_file) != 0) {   /* 원자적 교체 */
        logmsg("ERROR", "토큰 파일 교체 실패: %s", strerror(errno));
        (void)unlink(tmp_path);
        return false;
    }
    return true;
}

/* 외부 명령 실행. 셸을 거치지 않는다(인젝션 여지 제거). 성공 시 true. */
static bool run_cmd(char *const argv[])
{
    const pid_t pid = fork();
    if (pid < 0) { logmsg("ERROR", "fork 실패: %s", strerror(errno)); return false; }

    if (pid == 0) {
        /* 자식: 표준출력은 버리고 표준에러만 남긴다 */
        const int devnull = open("/dev/null", O_WRONLY);
        if (devnull >= 0) { (void)dup2(devnull, STDOUT_FILENO); (void)close(devnull); }
        (void)execv(argv[0], argv);
        _exit(127);
    }

    int status = 0;
    if (waitpid(pid, &status, 0) < 0) { return false; }
    return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

/* ── 인증서 발급 ─────────────────────────────────────────────────────── */
static bool issue_cert(const char *cn, char **ca_pem, char **crt_pem, char **key_pem)
{
    char crt_path[512];
    char key_path[512];
    char pkcs8_path[512];
    char ca_path[512];

    (void)snprintf(crt_path,   sizeof crt_path,   "%s/%s.crt",       g_cert_dir, cn);
    (void)snprintf(key_path,   sizeof key_path,   "%s/%s-trad.key",  g_cert_dir, cn);
    (void)snprintf(pkcs8_path, sizeof pkcs8_path, "%s/%s.key",       g_cert_dir, cn);
    (void)snprintf(ca_path,    sizeof ca_path,    "%s/ca.crt",       g_cert_dir);

    /* 재발급(로그아웃 후 재등록 등)이면 기존 파일을 지우고 새로 만든다.
     * ⚠️ 이전 인증서는 파일을 지운다고 폐기되지 않는다 — 암호학적으로 여전히
     *    유효하다. 실제로 무효화하려면 브로커에 CRL 을 걸어야 한다. */
    if (access(crt_path, F_OK) == 0) {
        logmsg("WARN", "CN=%s 재발급 — 이전 인증서는 CRL 없이는 여전히 유효합니다", cn);
        (void)unlink(crt_path);
        (void)unlink(key_path);
        (void)unlink(pkcs8_path);
    }

    /* gen-certs.sh 는 bash 스크립트다. execv 로 직접 띄우려면 실행권한과
     * shebang 에 의존하게 되므로 bash 를 명시적으로 호출한다. */
    char *bash_argv[] = { (char *)"/bin/bash", (char *)g_gen_certs, (char *)"--client",
                          (char *)cn, (char *)g_cert_dir, NULL };

    if (!run_cmd(bash_argv)) {
        logmsg("ERROR", "gen-certs.sh --client %s 실패", cn);
        return false;
    }

    *ca_pem  = read_file(ca_path,  MAX_PEM);
    *crt_pem = read_file(crt_path, MAX_PEM);
    *key_pem = read_file(key_path, MAX_PEM);
    if (*ca_pem == NULL || *crt_pem == NULL || *key_pem == NULL) {
        logmsg("ERROR", "발급된 파일을 읽지 못했습니다 (CN=%s)", cn);
        free(*ca_pem);  free(*crt_pem);  free(*key_pem);
        *ca_pem = NULL; *crt_pem = NULL; *key_pem = NULL;
        return false;
    }
    return true;
}

/* ── ACL ─────────────────────────────────────────────────────────────────
 * mosquitto 의 `user` 는 정확 매칭이라 와일드카드가 없다. 발급할 때마다 CN
 * 블록을 추가하고 reload 해야 한다. 빠뜨리면 TLS 핸드셰이크는 성공하는데
 * 구독·발행만 조용히 막혀서 원인을 찾기 어렵다 — 가장 놓치기 쉬운 곳.
 * ---------------------------------------------------------------------- */
static bool ensure_acl(const char *cn)
{
    char *body = read_file(g_acl_file, 1u << 20);
    if (body == NULL) {
        logmsg("ERROR", "ACL 파일을 읽지 못했습니다: %s", g_acl_file);
        return false;
    }

    char marker[MAX_CN + 8];
    (void)snprintf(marker, sizeof marker, "\nuser %s\n", cn);
    const bool present = (strstr(body, marker) != NULL);
    free(body);

    if (present) {
        logmsg("INFO", "ACL 에 %s 이미 있음 — 추가하지 않습니다", cn);
    } else {
        FILE *f = fopen(g_acl_file, "a");
        if (f == NULL) {
            logmsg("ERROR", "ACL 파일을 열지 못했습니다: %s", g_acl_file);
            return false;
        }
        (void)fprintf(f, "\nuser %s\n"
                         "topic write adts/cmd/#\n"
                         "topic read  adts/state/#\n"
                         "topic read  adts/event/#\n", cn);
        if (fclose(f) != 0) {
            logmsg("ERROR", "ACL 파일 쓰기를 마치지 못했습니다");
            return false;
        }
        logmsg("INFO", "ACL 에 %s 추가", cn);
    }

    /* reload(SIGHUP)는 ACL 만 다시 읽는다. restart 를 쓰면 붙어 있던
     * 클라이언트가 전부 끊기므로 쓰지 않는다. */
    char *sysctl_argv[] = { (char *)g_systemctl, (char *)"reload",
                            (char *)"mosquitto", NULL };
    if (!run_cmd(sysctl_argv)) {
        logmsg("ERROR", "mosquitto reload 실패 — ACL 이 반영되지 않았습니다");
        return false;
    }
    return true;
}

/* ── HTTP 응답 ───────────────────────────────────────────────────────── */
static void send_json(SSL *ssl, int code, const char *reason, const char *body)
{
    char head[256];
    const int n = snprintf(head, sizeof head,
                           "HTTP/1.1 %d %s\r\n"
                           "Content-Type: application/json; charset=utf-8\r\n"
                           "Content-Length: %zu\r\n"
                           "Connection: close\r\n\r\n",
                           code, reason, strlen(body));
    if (n > 0) { (void)SSL_write(ssl, head, n); }
    (void)SSL_write(ssl, body, (int)strlen(body));
}

static void send_error(SSL *ssl, int code, const char *reason, const char *msg)
{
    cJSON *o = cJSON_CreateObject();
    if (o == NULL) { return; }
    (void)cJSON_AddStringToObject(o, "error", msg);
    char *txt = cJSON_PrintUnformatted(o);
    cJSON_Delete(o);
    if (txt != NULL) { send_json(ssl, code, reason, txt); free(txt); }
}


/* ── GET /scan/<파일명> ──────────────────────────────────────────────────
 *  Qt 관제가 state/scan 으로 받은 .pcd 를 실제로 가져가는 경로.
 *  이 서비스는 인증서를 발급하는 곳이라 파일 경로를 다는 것 자체가 위험하다.
 *  그래서 세 겹으로 막는다.
 *    1) 검증된 클라이언트 인증서를 요구한다 (발급받은 콘솔만 접근).
 *    2) 파일명만 받는다 — '/' 나 '%' 가 하나라도 있으면 거부. 디렉터리는
 *       g_scan_dir 로 고정이라 경로 탈출이 성립할 수 없다.
 *    3) 확장자는 .pcd 만.
 *  '%' 를 디코드하지 않고 그냥 거부하는 이유: 스캔 파일명은 영숫자와 -_. 뿐이라
 *  인코딩이 필요 없고, 디코더를 두면 그 자체가 새로운 우회 표면이 된다.
 */
static int scan_name_ok(const char *name)
{
    size_t i;
    const size_t n = strlen(name);

    if ((n == 0u) || (n > 160u)) { return 0; }
    if (strstr(name, "..") != NULL) { return 0; }
    if ((n < 5u) || (strcmp(name + (n - 4u), ".pcd") != 0)) { return 0; }

    for (i = 0u; i < n; ++i) {
        const char c = name[i];
        const int alnum = (((c >= 'a') && (c <= 'z')) || ((c >= 'A') && (c <= 'Z')) ||
                           ((c >= '0') && (c <= '9')));
        if ((alnum == 0) && (c != '-') && (c != '_') && (c != '.')) { return 0; }
    }
    return 1;
}


/* ── GET /scans ──────────────────────────────────────────────────────────
 *  스캔 파일 목록. /scan 과 같은 조건(검증된 클라이언트 인증서)을 요구한다.
 *  파일명·크기·수정시각만 준다 — 경로는 주지 않는다. 클라이언트는 파일명만
 *  알면 되고, 서버가 디렉터리를 고정하고 있어서 경로를 노출할 이유가 없다.
 */
static void send_scan_list(SSL *ssl, const char *peer)
{
    DIR           *d;
    struct dirent *de;
    cJSON         *root;
    cJSON         *arr;
    char          *txt;
    int            n = 0;

    {
        X509      *cert = SSL_get1_peer_certificate(ssl);
        const long vr   = SSL_get_verify_result(ssl);
        if ((cert == NULL) || (vr != X509_V_OK)) {
            if (cert != NULL) { X509_free(cert); }
            send_error(ssl, 401, "Unauthorized", "클라이언트 인증서가 필요합니다");
            return;
        }
        X509_free(cert);
    }

    d = opendir(g_scan_dir);
    if (d == NULL) {
        send_error(ssl, 404, "Not Found", "스캔 디렉터리가 없습니다");
        return;
    }

    root = cJSON_CreateObject();
    arr  = cJSON_CreateArray();
    if ((root == NULL) || (arr == NULL)) {
        (void)closedir(d);
        if (root != NULL) { cJSON_Delete(root); }
        if (arr != NULL)  { cJSON_Delete(arr); }
        send_error(ssl, 500, "Internal Server Error", "메모리 부족");
        return;
    }

    while (((de = readdir(d)) != NULL) && (n < 500)) {
        char        path[640];
        struct stat st;
        cJSON      *item;

        if (scan_name_ok(de->d_name) == 0) { continue; }
        (void)snprintf(path, sizeof path, "%s/%s", g_scan_dir, de->d_name);
        if ((stat(path, &st) != 0) || (S_ISREG(st.st_mode) == 0)) { continue; }

        item = cJSON_CreateObject();
        if (item == NULL) { break; }
        (void)cJSON_AddStringToObject(item, "name", de->d_name);
        (void)cJSON_AddNumberToObject(item, "size", (double)st.st_size);
        (void)cJSON_AddNumberToObject(item, "mtime", (double)st.st_mtime);
        (void)cJSON_AddItemToArray(arr, item);
        ++n;
    }
    (void)closedir(d);

    (void)cJSON_AddItemToObject(root, "scans", arr);
    txt = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (txt == NULL) { send_error(ssl, 500, "Internal Server Error", "직렬화 실패"); return; }
    send_json(ssl, 200, "OK", txt);
    free(txt);
    logmsg("INFO", "%s: /scans %d건", peer, n);
}

static void send_scan_file(SSL *ssl, const char *name, const char *peer)
{
    char        path[640];
    struct stat st;
    int         fd;
    char        head[256];
    int         hn;

    /* 1) 클라이언트 인증서 검증 */
    {
        X509 *cert = SSL_get1_peer_certificate(ssl);
        const long vr = SSL_get_verify_result(ssl);
        if ((cert == NULL) || (vr != X509_V_OK)) {
            if (cert != NULL) { X509_free(cert); }
            logmsg("WARN", "%s: /scan 거부 — 클라이언트 인증서 없음/검증 실패", peer);
            send_error(ssl, 401, "Unauthorized",
                       "클라이언트 인증서가 필요합니다 (발급받은 콘솔만 내려받을 수 있습니다)");
            return;
        }
        X509_free(cert);
    }

    /* 2) 파일명 검사 */
    if (scan_name_ok(name) == 0) {
        logmsg("WARN", "%s: /scan 거부 — 파일명 규칙 위반", peer);
        send_error(ssl, 400, "Bad Request", "파일명이 올바르지 않습니다 (.pcd, 경로 불가)");
        return;
    }

    (void)snprintf(path, sizeof path, "%s/%s", g_scan_dir, name);
    if ((stat(path, &st) != 0) || (S_ISREG(st.st_mode) == 0)) {
        send_error(ssl, 404, "Not Found", "그런 스캔 파일이 없습니다");
        return;
    }

    fd = open(path, O_RDONLY);
    if (fd < 0) {
        send_error(ssl, 500, "Internal Server Error", "파일을 열 수 없습니다");
        return;
    }

    hn = snprintf(head, sizeof head,
                  "HTTP/1.1 200 OK\r\n"
                  "Content-Type: application/octet-stream\r\n"
                  "Content-Length: %lld\r\n"
                  "Connection: close\r\n\r\n",
                  (long long)st.st_size);
    if (hn > 0) { (void)SSL_write(ssl, head, hn); }

    for (;;) {
        char    chunk[16384];
        ssize_t r = read(fd, chunk, sizeof chunk);
        if (r <= 0) { break; }
        if (SSL_write(ssl, chunk, (int)r) <= 0) { break; }
    }
    (void)close(fd);
    logmsg("INFO", "%s: /scan %s (%lld B) 전송", peer, name, (long long)st.st_size);
}

/* ── 요청 처리 ───────────────────────────────────────────────────────── */
static void handle_request(SSL *ssl, const char *peer)
{
    char   buf[MAX_HEADER + MAX_BODY + 1u];
    size_t used = 0;
    char  *hdr_end = NULL;

    /* 헤더가 끝날 때까지 읽는다 */
    while (used < MAX_HEADER) {
        const int r = SSL_read(ssl, buf + used, (int)(sizeof buf - used - 1u));
        if (r <= 0) { return; }
        used += (size_t)r;
        buf[used] = '\0';
        hdr_end = strstr(buf, "\r\n\r\n");
        if (hdr_end != NULL) { break; }
    }
    if (hdr_end == NULL) { send_error(ssl, 400, "Bad Request", "헤더가 너무 큽니다"); return; }

    if (strncmp(buf, "GET /healthz ", 13) == 0) {
        send_json(ssl, 200, "OK", "{\"ok\":true}");
        return;
    }
    if (strncmp(buf, "GET /scans ", 11) == 0) {
        send_scan_list(ssl, peer);
        return;
    }
    if (strncmp(buf, "GET /scan/", 10) == 0) {
        char name[192];
        const char *p = buf + 10;
        size_t i = 0u;
        while ((i + 1u < sizeof name) && (*p != ' ') && (*p != '\r') && (*p != '\0')) {
            name[i++] = *p++;
        }
        name[i] = '\0';
        send_scan_file(ssl, name, peer);
        return;
    }
    if (strncmp(buf, "POST /enroll ", 13) != 0) {
        send_error(ssl, 404, "Not Found", "없는 경로입니다");
        return;
    }

    /* Host 헤더 (mqtt.host 기본값으로 쓴다) */
    char host_hdr[128] = {0};
    {
        const char *h = strcasestr(buf, "\r\nHost:");
        if (h != NULL) {
            h += 7;
            while (*h == ' ') { ++h; }
            size_t i = 0;
            while (i + 1u < sizeof host_hdr && *h != '\r' && *h != ':' && *h != '\0') {
                host_hdr[i++] = *h++;
            }
            host_hdr[i] = '\0';
        }
    }

    /* Content-Length */
    long clen = 0;
    {
        const char *c = strcasestr(buf, "\r\nContent-Length:");
        if (c == NULL) { send_error(ssl, 400, "Bad Request", "Content-Length 가 없습니다"); return; }
        clen = strtol(c + 17, NULL, 10);
    }
    if (clen <= 0 || (unsigned long)clen > MAX_BODY) {
        send_error(ssl, 400, "Bad Request", "요청 본문 크기가 잘못되었습니다");
        return;
    }

    /* 본문이 아직 덜 왔으면 마저 읽는다 */
    char        *body      = hdr_end + 4;
    const size_t have_body = used - (size_t)(body - buf);
    size_t       need      = (size_t)clen;
    if (have_body < need) {
        size_t got = have_body;
        while (got < need && used < sizeof buf - 1u) {
            const int r = SSL_read(ssl, buf + used, (int)(sizeof buf - used - 1u));
            if (r <= 0) { return; }
            used += (size_t)r;
            got  += (size_t)r;
            buf[used] = '\0';
        }
        if (got < need) { send_error(ssl, 400, "Bad Request", "본문이 잘렸습니다"); return; }
    }
    body[need] = '\0';

    /* ── JSON 파싱 ── */
    cJSON *req = cJSON_Parse(body);
    if (req == NULL) { send_error(ssl, 400, "Bad Request", "JSON 을 해석하지 못했습니다"); return; }

    const cJSON *jtok = cJSON_GetObjectItemCaseSensitive(req, "token");
    const cJSON *jdev = cJSON_GetObjectItemCaseSensitive(req, "device_name");
    if (!cJSON_IsString(jtok) || jtok->valuestring[0] == '\0') {
        send_error(ssl, 400, "Bad Request", "token 이 없습니다");
        cJSON_Delete(req);
        return;
    }
    const char *device = cJSON_IsString(jdev) ? jdev->valuestring : "-";

    /* ── 토큰 검증 및 소비 ── */
    char label[MAX_LABEL] = {0};
    if (!consume_token(jtok->valuestring, label, sizeof label)) {
        /* 토큰이 틀렸는지 이미 썼는지 구분해 알려주지 않는다 — 추측을 돕지 않기 위해 */
        logmsg("WARN", "발급 거부: 토큰 불일치 또는 이미 사용됨 (device=%s, from=%s)",
               device, peer);
        send_error(ssl, 401, "Unauthorized", "토큰이 유효하지 않거나 이미 사용되었습니다");
        cJSON_Delete(req);
        return;
    }

    char cn[MAX_CN];
    (void)snprintf(cn, sizeof cn, "qt-console-%s", label);
    if (!cn_is_safe(cn)) {
        logmsg("ERROR", "토큰 라벨이 CN 으로 쓸 수 없는 문자를 담고 있습니다: %s", label);
        send_error(ssl, 500, "Internal Server Error", "서버 토큰 설정이 잘못되었습니다");
        cJSON_Delete(req);
        return;
    }

    /* ── 발급 + ACL ── */
    char *ca_pem = NULL, *crt_pem = NULL, *key_pem = NULL;
    if (!issue_cert(cn, &ca_pem, &crt_pem, &key_pem)) {
        send_error(ssl, 500, "Internal Server Error", "인증서 발급에 실패했습니다");
        cJSON_Delete(req);
        return;
    }
    if (!ensure_acl(cn)) {
        free(ca_pem); free(crt_pem); free(key_pem);
        send_error(ssl, 500, "Internal Server Error",
                   "ACL 갱신에 실패했습니다 — 발급은 됐으나 권한이 없습니다");
        cJSON_Delete(req);
        return;
    }

    /* ── 응답 조립 ── */
    cJSON *res = cJSON_CreateObject();
    if (res == NULL) {
        free(ca_pem); free(crt_pem); free(key_pem);
        cJSON_Delete(req);
        return;
    }
    (void)cJSON_AddStringToObject(res, "cn",         cn);
    (void)cJSON_AddStringToObject(res, "ca_crt",     ca_pem);
    (void)cJSON_AddStringToObject(res, "client_crt", crt_pem);
    (void)cJSON_AddStringToObject(res, "client_key", key_pem);

    cJSON *mq = cJSON_AddObjectToObject(res, "mqtt");
    if (mq != NULL) {
        const char *h = (g_mqtt_host[0] != '\0') ? g_mqtt_host : host_hdr;
        (void)cJSON_AddStringToObject(mq, "host", h);
        (void)cJSON_AddNumberToObject(mq, "port", g_mqtt_port);
    }

    /* 카메라 설정은 없어도 진행한다 — MQTT 는 되고 영상만 안 나오는 상태가 된다. */
    char *cam_txt = read_file(g_camera_file, MAX_PEM);
    if (cam_txt != NULL) {
        cJSON *cams = cJSON_Parse(cam_txt);
        free(cam_txt);
        if (cams != NULL) {
            (void)cJSON_AddItemToObject(res, "cameras", cams);
        } else {
            logmsg("WARN", "카메라 설정 JSON 을 해석하지 못했습니다: %s", g_camera_file);
        }
    } else {
        logmsg("WARN", "카메라 설정이 없습니다: %s — cameras 없이 응답합니다", g_camera_file);
    }

    char *txt = cJSON_PrintUnformatted(res);
    if (txt != NULL) {
        send_json(ssl, 200, "OK", txt);
        free(txt);
        logmsg("INFO", "발급 완료: CN=%s device=%s from=%s", cn, device, peer);
    }

    cJSON_Delete(res);
    cJSON_Delete(req);
    free(ca_pem); free(crt_pem); free(key_pem);
}

/* ── main ────────────────────────────────────────────────────────────── */
int main(void)
{
    g_cert_dir    = env_or("ADTS_CERT_DIR",    DEF_CERT_DIR);
    g_scan_dir    = env_or("ADTS_SCAN_DIR",    DEF_SCAN_DIR);
    g_token_file  = env_or("ADTS_TOKEN_FILE",  DEF_TOKEN_FILE);
    g_camera_file = env_or("ADTS_CAMERA_FILE", DEF_CAMERA_FILE);
    g_acl_file    = env_or("ADTS_ACL_FILE",    DEF_ACL_FILE);
    g_gen_certs   = env_or("ADTS_GEN_CERTS",   DEF_GEN_CERTS);
    g_systemctl   = env_or("ADTS_SYSTEMCTL",   DEF_SYSTEMCTL);
    g_mqtt_host   = env_or("ADTS_MQTT_HOST",   "");
    g_mqtt_port   = atoi(env_or("ADTS_MQTT_PORT", "8883"));
    if (g_mqtt_port <= 0) { g_mqtt_port = DEF_MQTT_PORT; }

    const int bind_port = atoi(env_or("ADTS_BIND_PORT", "8443"));

    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    (void)sigaction(SIGINT,  &sa, NULL);
    (void)sigaction(SIGTERM, &sa, NULL);
    (void)signal(SIGPIPE, SIG_IGN);     /* 클라이언트가 먼저 끊어도 죽지 않게 */

    char server_crt[512], server_key[512];
    (void)snprintf(server_crt, sizeof server_crt, "%s/server.crt", g_cert_dir);
    (void)snprintf(server_key, sizeof server_key, "%s/server.key", g_cert_dir);

    SSL_CTX *ctx = SSL_CTX_new(TLS_server_method());
    if (ctx == NULL) { logmsg("ERROR", "SSL_CTX_new 실패"); return 1; }
    (void)SSL_CTX_set_min_proto_version(ctx, TLS1_2_VERSION);

    /* 클라이언트 인증서를 "받기는 하되 없어도 통과"시킨다.
     *   - /enroll 은 인증서를 발급받기 전에 부르는 것이라 인증서를 요구할 수 없다.
     *   - /scan 은 파일을 내주므로 검증된 인증서를 요구한다(handle_request 참고).
     * SSL_VERIFY_PEER 만 켜고 FAIL_IF_NO_PEER_CERT 는 켜지 않는 이유가 이것이다. */
    {
        char ca_path[512];
        (void)snprintf(ca_path, sizeof ca_path, "%s/ca.crt", g_cert_dir);
        if (SSL_CTX_load_verify_locations(ctx, ca_path, NULL) != 1) {
            logmsg("WARN", "클라이언트 CA(%s)를 읽지 못했습니다 — /scan 은 항상 거부됩니다", ca_path);
        } else {
            SSL_CTX_set_verify(ctx, SSL_VERIFY_PEER, NULL);
            (void)SSL_CTX_set_client_CA_list(ctx, SSL_load_client_CA_file(ca_path));
        }
    }

    if (SSL_CTX_use_certificate_file(ctx, server_crt, SSL_FILETYPE_PEM) != 1 ||
        SSL_CTX_use_PrivateKey_file(ctx, server_key, SSL_FILETYPE_PEM) != 1) {
        logmsg("ERROR", "서버 인증서/키를 읽지 못했습니다 (%s, %s)", server_crt, server_key);
        ERR_print_errors_fp(stderr);
        SSL_CTX_free(ctx);
        return 1;
    }

    const int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) { logmsg("ERROR", "socket 실패: %s", strerror(errno)); SSL_CTX_free(ctx); return 1; }

    const int one = 1;
    (void)setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof addr);
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons((uint16_t)bind_port);

    if (bind(sock, (struct sockaddr *)&addr, sizeof addr) < 0 || listen(sock, 8) < 0) {
        logmsg("ERROR", "bind/listen 실패 (포트 %d): %s", bind_port, strerror(errno));
        (void)close(sock);
        SSL_CTX_free(ctx);
        return 1;
    }

    logmsg("INFO", "발급 서비스 시작 — https://0.0.0.0:%d/enroll", bind_port);
    logmsg("INFO", "  인증서 %s / 토큰 %s / ACL %s", g_cert_dir, g_token_file, g_acl_file);
    logmsg("INFO", "  스캔 %s (GET /scan/<파일명>, 클라이언트 인증서 필요)", g_scan_dir);

    while (g_stop == 0) {
        struct sockaddr_in peer;
        socklen_t          plen = sizeof peer;
        const int          fd   = accept(sock, (struct sockaddr *)&peer, &plen);
        if (fd < 0) {
            if (errno == EINTR) { continue; }
            logmsg("WARN", "accept 실패: %s", strerror(errno));
            continue;
        }

        char peer_ip[INET_ADDRSTRLEN] = "?";
        (void)inet_ntop(AF_INET, &peer.sin_addr, peer_ip, sizeof peer_ip);

        SSL *ssl = SSL_new(ctx);
        if (ssl != NULL) {
            (void)SSL_set_fd(ssl, fd);
            if (SSL_accept(ssl) == 1) {
                handle_request(ssl, peer_ip);
                (void)SSL_shutdown(ssl);
            } else {
                logmsg("WARN", "TLS 핸드셰이크 실패 (from=%s)", peer_ip);
            }
            SSL_free(ssl);
        }
        (void)close(fd);
    }

    logmsg("INFO", "종료");
    (void)close(sock);
    SSL_CTX_free(ctx);
    return 0;
}
