/* ============================================================================
 *  fd.hpp  --  POSIX 파일 디스크립터 RAII 래퍼 (move-only)
 *  담당: 이현우 (데몬 코어)
 *
 *  데몬은 fd 가 많다(epoll/timerfd/signalfd/socket/ /dev/turret). 소멸자에서
 *  자동 close 하여 누수·이중해제(C 의 goto cleanup 실수)를 원천 차단한다.
 * ==========================================================================*/
#ifndef ADTS_FD_HPP
#define ADTS_FD_HPP

#include <unistd.h>

class Fd {
public:
    Fd() noexcept = default;
    explicit Fd(int fd) noexcept : fd_(fd) {}

    ~Fd() { reset(); }

    Fd(const Fd &) = delete;
    Fd &operator=(const Fd &) = delete;

    Fd(Fd &&other) noexcept : fd_(other.fd_) { other.fd_ = -1; }
    Fd &operator=(Fd &&other) noexcept {
        if (this != &other) {
            reset();
            fd_ = other.fd_;
            other.fd_ = -1;
        }
        return *this;
    }

    [[nodiscard]] bool valid() const noexcept { return fd_ >= 0; }
    [[nodiscard]] int  get()   const noexcept { return fd_; }

    void reset() noexcept {
        if (fd_ >= 0) {
            (void)::close(fd_);
            fd_ = -1;
        }
    }

    [[nodiscard]] int release() noexcept {
        int f = fd_;
        fd_ = -1;
        return f;
    }

private:
    int fd_ = -1;
};

#endif /* ADTS_FD_HPP */
