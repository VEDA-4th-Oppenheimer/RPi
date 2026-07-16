/* ============================================================================
 *  main.cpp  --  통합 데몬 코어 (A.D.T.S)
 *  담당: 이현우
 *
 *  - 단일 스레드 epoll 이벤트 루프 (락 불필요)
 *  - 상태머신 IDLE -> TRACK -> LOCK_ON -> DISARM (daemon_module.h 계약)
 *  - shared_ctx 소유. 모듈(tracking/meta/tls)을 등록·구동
 *  - /dev/turret 은 코어가 직접 ioctl (protocol.h). 없으면 degraded 모드로 구동
 *  - 100ms timerfd tick 에서 STM 링크상태 폴링 + 모듈 on_tick + 상태 평가
 *  - core_aim / core_request_state / core_log 구현 (모듈이 호출)
 * ==========================================================================*/
#define PROTO_WANT_IOCTL 1          /* protocol.h 의 ioctl 인터페이스 노출 */

#include "daemon_module.h"
#include "protocol.h"
#include "fd.hpp"

#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <sys/signalfd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cerrno>
#include <ctime>
#include <vector>
#include <unordered_map>

namespace {

constexpr int kTickMs      = 100;   /* heartbeat/tick 주기 (protocol.h HB_PING_PERIOD_MS) */
constexpr int kMaxEvents   = 16;
constexpr const char *kTurretDev = "/dev/turret";

/* 주기 tick 용 timerfd 생성 (period_ms 간격 반복) */
Fd make_timerfd(int period_ms)
{
    Fd fd(::timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC));
    if (!fd.valid()) {
        return fd;
    }
    itimerspec its{};
    its.it_interval.tv_sec  = period_ms / 1000;
    its.it_interval.tv_nsec = static_cast<long>(period_ms % 1000) * 1000000L;
    its.it_value = its.it_interval;
    if (::timerfd_settime(fd.get(), 0, &its, nullptr) < 0) {
        return Fd();
    }
    return fd;
}

/* SIGINT/SIGTERM 를 signalfd 로 받아 epoll 에서 처리 (graceful shutdown) */
Fd make_signalfd()
{
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGTERM);
    if (::sigprocmask(SIG_BLOCK, &mask, nullptr) < 0) {
        return Fd();
    }
    return Fd(::signalfd(-1, &mask, SFD_NONBLOCK | SFD_CLOEXEC));
}

/* 단조 증가 시각(ms). heartbeat 300ms 판정은 이 시계 기준(데몬 소유). */
uint64_t mono_ms()
{
    timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000u
         + static_cast<uint64_t>(ts.tv_nsec) / 1000000u;
}

bool epoll_add(int epfd, int fd, uint32_t events)
{
    epoll_event ev{};
    ev.events  = events;
    ev.data.fd = fd;
    return ::epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev) == 0;
}

}  /* namespace */

/* ---------------------------------------------------------------------------
 *  코어 구조체
 * ------------------------------------------------------------------------- */
struct Core {
    shared_ctx ctx{};
    Fd  epoll_fd;
    Fd  turret_fd;                                  /* 없을 수 있음(개발 VM) */
    Fd  timer_fd;
    Fd  signal_fd;
    std::vector<const daemon_module *>            modules;
    std::unordered_map<int, const daemon_module *> fd_modules;
    bool running = true;

    /* heartbeat 상태(데몬 소유). 드라이버 pong_seq 증가를 자기 시계로 스탬프 */
    uint32_t hb_last_seq  = 0;      /* 마지막으로 관측한 pong_seq        */
    uint64_t hb_last_pong = 0;      /* 마지막 PONG 관측 시각(mono ms)    */
    bool     hb_primed    = false;  /* 최초 GET_STATE 로 기준선 설정 여부 */

    bool setup();
    void run();
    void tick();
    void on_turret_event();
    void poll_link();
    void eval_state();
    void transition(daemon_state_t want);
    void shutdown();

    int  do_aim(int16_t theta_ddeg, int16_t phi_ddeg);
};

/* /dev/turret 로 목표각 전송 */
int Core::do_aim(int16_t theta_ddeg, int16_t phi_ddeg)
{
    if (!turret_fd.valid()) {
        return -ENODEV;
    }
    proto_target t{};
    t.theta_ddeg = theta_ddeg;
    t.phi_ddeg   = phi_ddeg;
    if (::ioctl(turret_fd.get(), TURRET_SET_TARGET, &t) < 0) {
        return -errno;
    }
    return 0;
}

/* heartbeat + STM 상태 캐시 갱신.
 *   - PING 송신(fire-and-forget): 주기는 tick(100ms) = HB_PING_PERIOD_MS
 *   - PONG 도착은 드라이버 pong_seq 증가로 감지 → 자기 CLOCK_MONOTONIC 스탬프
 *   - HB_TIMEOUT_MS(300ms) 무응답 시 link_dead 판정 → DISARM (정책=데몬 소유) */
void Core::poll_link()
{
    if (!turret_fd.valid()) {
        return;                          /* degraded: STM 링크 없음 */
    }

    /* ① PING 송신 */
    (void)::ioctl(turret_fd.get(), TURRET_PING);   /* 실패는 아래 타임아웃이 흡수 */

    /* ② 상태 조회 */
    turret_link_state st{};
    if (::ioctl(turret_fd.get(), TURRET_GET_STATE, &st) < 0) {
        return;
    }
    ctx.aim.cur_theta_ddeg = st.cur_theta_ddeg;
    ctx.aim.cur_phi_ddeg   = st.cur_phi_ddeg;
    ctx.aim.homed          = (st.flags & STF_HOMED)   ? 1u : 0u;
    ctx.aim.aligned        = (st.flags & STF_ALIGNED) ? 1u : 0u;

    /* ③ PONG 도착 감지 → 자기 시계로 타임스탬프 */
    const uint64_t now = mono_ms();
    if (!hb_primed) {
        hb_primed    = true;             /* 시작 시점부터 grace 부여 */
        hb_last_seq  = st.pong_seq;
        hb_last_pong = now;
    } else if (st.pong_seq != hb_last_seq) {
        hb_last_seq  = st.pong_seq;
        hb_last_pong = now;
    }

    /* ④ 300ms 무응답 → link_dead (데몬이 link_alive 를 권위있게 계산) */
    const bool alive = (now - hb_last_pong) <= HB_TIMEOUT_MS;
    ctx.aim.link_alive = alive ? 1u : 0u;

    if (!alive && ctx.state != ST_DISARM) {
        core_log(this, "LINK", "link_dead (PONG > %ums) -> DISARM", HB_TIMEOUT_MS);
        transition(ST_DISARM);
    }
}

/* 상태 자동 평가 (센서 입력 기반). 실제 임계/히스테리시스는 추후 정교화 */
void Core::eval_state()
{
    switch (ctx.state) {
    case ST_IDLE:
        if (ctx.pixerr.valid != 0u) {           /* 카메라가 표적 포착 */
            transition(ST_TRACK);
        }
        break;
    case ST_TRACK:
        if (ctx.pixerr.valid == 0u) {           /* 표적 상실 */
            transition(ST_IDLE);
        } else if (ctx.aim.aligned != 0u) {     /* 정렬 완료 */
            transition(ST_LOCK_ON);
        }
        break;
    case ST_LOCK_ON:
        if (ctx.pixerr.valid == 0u) {
            transition(ST_TRACK);
        }
        break;
    case ST_DISARM:
    default:
        break;
    }
}

void Core::transition(daemon_state_t want)
{
    const daemon_state_t cur = ctx.state;
    if (want == cur) {
        return;
    }
    bool ok = false;
    switch (cur) {
    case ST_IDLE:    ok = (want == ST_TRACK)   || (want == ST_DISARM); break;
    case ST_TRACK:   ok = (want == ST_LOCK_ON) || (want == ST_IDLE) || (want == ST_DISARM); break;
    case ST_LOCK_ON: ok = (want == ST_TRACK)   || (want == ST_IDLE) || (want == ST_DISARM); break;
    case ST_DISARM:  ok = (want == ST_IDLE); break;   /* rearm */
    default:         ok = false; break;
    }
    if (!ok) {
        core_log(this, "FSM", "reject %s -> %s",
                 daemon_state_str(cur), daemon_state_str(want));
        return;
    }
    ctx.state = want;
    core_log(this, "FSM", "%s -> %s", daemon_state_str(cur), daemon_state_str(want));
    for (const daemon_module *m : modules) {
        if (m->on_state != nullptr) {
            m->on_state(&ctx, cur, want);
        }
    }
}

bool Core::setup()
{
    ctx.core  = this;
    ctx.state = ST_IDLE;

    /* /dev/turret (없으면 degraded 로 계속 — 개발 VM 대응) */
    turret_fd = Fd(::open(kTurretDev, O_RDWR | O_CLOEXEC | O_NONBLOCK));
    if (!turret_fd.valid()) {
        core_log(this, "TURRET", "open %s 실패 (%s) — degraded 모드로 구동",
                 kTurretDev, std::strerror(errno));
    }

    timer_fd  = make_timerfd(kTickMs);
    signal_fd = make_signalfd();
    epoll_fd  = Fd(::epoll_create1(EPOLL_CLOEXEC));
    if (!timer_fd.valid() || !signal_fd.valid() || !epoll_fd.valid()) {
        core_log(this, "SETUP", "timer/signal/epoll fd 생성 실패");
        return false;
    }

    if (!epoll_add(epoll_fd.get(), timer_fd.get(),  EPOLLIN) ||
        !epoll_add(epoll_fd.get(), signal_fd.get(), EPOLLIN)) {
        core_log(this, "SETUP", "epoll 등록 실패");
        return false;
    }
    if (turret_fd.valid()) {
        (void)epoll_add(epoll_fd.get(), turret_fd.get(), EPOLLIN | EPOLLERR);
    }

    /* 모듈 등록 (v2: tracking / meta / tls) */
    modules = { tracking_module_get(), meta_module_get(), tls_module_get() };
    for (const daemon_module *m : modules) {
        if (m->init != nullptr && m->init(&ctx) < 0) {
            core_log(this, "SETUP", "module '%s' init 실패", m->name);
            return false;
        }
        const int mfd = (m->get_fd != nullptr) ? m->get_fd() : -1;
        if (mfd >= 0) {
            if (!epoll_add(epoll_fd.get(), mfd, EPOLLIN)) {
                core_log(this, "SETUP", "module '%s' fd epoll 등록 실패", m->name);
                return false;
            }
            fd_modules[mfd] = m;
        }
        core_log(this, "SETUP", "module '%s' 등록 (fd=%d)", m->name, mfd);
    }

    core_log(this, "SETUP", "코어 준비 완료 (turret=%s)",
             turret_fd.valid() ? "on" : "off");
    return true;
}

void Core::tick()
{
    uint64_t expirations = 0;
    if (::read(timer_fd.get(), &expirations, sizeof(expirations)) < 0) {
        /* EAGAIN 등은 무시 */
    }

    poll_link();                 /* STM 링크/조준 캐시 + heartbeat 판정 */

    if (ctx.req_disarm != 0u) {  /* 모듈이 정지 요청 */
        ctx.req_disarm = 0u;
        transition(ST_DISARM);
    }

    for (const daemon_module *m : modules) {
        if (m->on_tick != nullptr) {
            m->on_tick(&ctx, ctx.state);
        }
    }

    eval_state();                /* 센서 입력 기반 자동 전이 */
}

void Core::on_turret_event()
{
    /* POLLIN(STM 통지 도착) / POLLERR(link_dead) 모두 상태 재확인으로 처리 */
    poll_link();
}

void Core::run()
{
    epoll_event events[kMaxEvents];
    while (running) {
        const int nfds = ::epoll_wait(epoll_fd.get(), events, kMaxEvents, -1);
        if (nfds < 0) {
            if (errno == EINTR) {
                continue;
            }
            core_log(this, "LOOP", "epoll_wait 오류: %s", std::strerror(errno));
            break;
        }
        for (int i = 0; i < nfds; ++i) {
            const int fd = events[i].data.fd;
            if (fd == timer_fd.get()) {
                tick();
            } else if (fd == signal_fd.get()) {
                signalfd_siginfo si{};
                if (::read(signal_fd.get(), &si, sizeof(si)) > 0) {
                    core_log(this, "SIGNAL", "signal %u 수신 -> 종료",
                             si.ssi_signo);
                }
                running = false;
            } else if (turret_fd.valid() && fd == turret_fd.get()) {
                on_turret_event();
            } else {
                auto it = fd_modules.find(fd);
                if (it != fd_modules.end() && it->second->on_event != nullptr) {
                    it->second->on_event(&ctx);
                }
            }
        }
    }
}

void Core::shutdown()
{
    core_log(this, "SHUTDOWN", "정리 시작");
    if (turret_fd.valid()) {
        transition(ST_DISARM);            /* 안전 정지 */
    }
    for (const daemon_module *m : modules) {
        if (m->deinit != nullptr) {
            m->deinit(&ctx);
        }
    }
    core_log(this, "SHUTDOWN", "완료");
}

/* ---------------------------------------------------------------------------
 *  코어 API 구현 (daemon_module.h 선언 — 모듈이 ctx->core 로 호출)
 * ------------------------------------------------------------------------- */
extern "C" {

int core_aim(void *core, int16_t theta_ddeg, int16_t phi_ddeg)
{
    return static_cast<Core *>(core)->do_aim(theta_ddeg, phi_ddeg);
}

int core_request_state(void *core, daemon_state_t want)
{
    static_cast<Core *>(core)->transition(want);
    return 0;
}

void core_log(void *core, const char *event, const char *fmt, ...)
{
    (void)core;
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    (void)::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    (void)std::fprintf(stderr, "[%-8s] %s\n", event, buf);
}

}  /* extern "C" */

/* ---------------------------------------------------------------------------
 *  진입점
 * ------------------------------------------------------------------------- */
int main()
{
    (void)std::fprintf(stderr, "=== A.D.T.S Daemon (proto v%u, module v%u) ===\n",
                       PROTO_VERSION, DAEMON_MODULE_VERSION);
    Core core;
    if (!core.setup()) {
        return 1;
    }
    core.run();
    core.shutdown();
    return 0;
}
