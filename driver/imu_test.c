/*
 * imu_test.c — /dev/imu ICM-20948 실시간 수평 측정 테스트
 *
 * ⚠️ MPU-6050 에서 ICM-20948 로 교체했지만 이 파일의 파싱 코드는 안 바뀌었다.
 *   드라이버가 /dev/imu 계약(6바이트 가속도, 빅엔디안, ±2g=16384 LSB/g)을
 *   그대로 유지하기 때문이다.
 *
 * ★ 교체 후 **가장 먼저 이걸로 축 방향을 확인할 것.** 칩이 바뀌면 패키지
 *   축이나 브레이크아웃 실장 방향이 달라져 roll/pitch 의 부호가 뒤집히거나
 *   두 축이 서로 바뀔 수 있다. 'z'(영점)는 오프셋만 없애 주지 부호는 못 고친다.
 *     · 앞쪽을 들어올렸을 때 pitch 가 +로 가는가
 *     · 오른쪽을 들어올렸을 때 roll 이 +로 가는가
 *   어긋나면 여기와 daemon/modules/imu/imu_module.c 의 식을 **둘 다** 고칠 것.
 *
 * 수정 사항 (원본 대비):
 *  1. 드라이버가 offset을 무시(no_llseek)하므로 무의미한 lseek() 제거
 *  2. SIGINT(Ctrl+C) 핸들러로 fd 정리 후 정상 종료
 *  3. system("clear") 대신 ANSI escape 로 화면 지우기 (fork 오버헤드/깜빡임 감소)
 *  4. 파일 기반 캘리브레이션(turret_imu.conf) 제거.
 *     대신 실행 중 'z' 키로 즉석 영점 잡기(tare), 'r' 키로 오프셋 리셋.
 *     오프셋은 메모리에만 존재하며 프로그램 종료 시 사라짐 (디스크에 안 남음).
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>
#include <signal.h>
#include <termios.h>
#include <sys/select.h>

#define DEV_IMU         "/dev/imu"
#define LEVEL_THRESHOLD 1.5f
#define ZERO_SAMPLES    20   /* 'z' 입력 시 평균 낼 샘플 개수 (노이즈 완화) */

static volatile sig_atomic_t g_running = 1;
static int g_fd = -1;
static struct termios g_orig_termios;
static int g_termios_saved = 0;

static void handle_sigint(int signo)
{
    (void)signo;
    g_running = 0;
}

/* stdin을 raw 모드로 전환: 캐노니컬/에코 끄고, read()가 즉시 리턴하도록
 * non-blocking으로 설정. Enter 없이 즉시 키 입력을 감지하기 위함. */
static void enable_raw_mode(void)
{
    struct termios raw;

    if (tcgetattr(STDIN_FILENO, &g_orig_termios) == -1)
        return; /* 터미널이 아니면(파이프 등) 조용히 무시 */
    g_termios_saved = 1;

    raw = g_orig_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

static void restore_terminal(void)
{
    if (g_termios_saved)
        tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios);
}

/* 키 입력이 대기 중이면 그 문자를, 없으면 0을 반환 (non-blocking) */
static char poll_keypress(void)
{
    fd_set fds;
    struct timeval tv = {0, 0};
    char c;

    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);

    if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0) {
        if (read(STDIN_FILENO, &c, 1) == 1)
            return c;
    }
    return 0;
}

static int read_sensor_angles(int fd, float *roll, float *pitch)
{
    uint8_t buf[6] = {0};

    ssize_t ret = read(fd, buf, 6);
    if (ret != 6) {
        return -1; /* 센서 데이터 읽기 실패 */
    }

    int16_t raw_ax = (int16_t)((uint16_t)buf[0] << 8 | buf[1]);
    int16_t raw_ay = (int16_t)((uint16_t)buf[2] << 8 | buf[3]);
    int16_t raw_az = (int16_t)((uint16_t)buf[4] << 8 | buf[5]);

    float ax = raw_ax / 16384.0f;
    float ay = raw_ay / 16384.0f;
    float az = raw_az / 16384.0f;

    *roll  = atan2f(ay, az) * 180.0f / (float)M_PI;
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / (float)M_PI;
    return 0;
}

/* 'z' 입력 시 호출: ZERO_SAMPLES개 샘플을 짧게 모아 평균 낸 뒤
 * 그 값을 새 오프셋으로 반환. 화면에 진행 상황을 보여준다. */
static void do_zero_calibration(int fd, float *offset_r, float *offset_p)
{
    float sum_r = 0.0f, sum_p = 0.0f;
    int good = 0;

    printf("\033[H\033[J");
    printf(" [영점 캘리브레이션] 센서를 수평으로 고정한 채 잠시 기다려주세요...\n");
    fflush(stdout);

    for (int i = 0; i < ZERO_SAMPLES; i++) {
        float r, p;
        if (read_sensor_angles(fd, &r, &p) == 0) {
            sum_r += r;
            sum_p += p;
            good++;
        }
        usleep(50000); /* 50ms 간격 */
    }

    if (good > 0) {
        *offset_r = sum_r / good;
        *offset_p = sum_p / good;
        printf(" [완료] 샘플 %d개 평균으로 영점 설정됨 (Roll: %+.3f° / Pitch: %+.3f°)\n",
               good, *offset_r, *offset_p);
    } else {
        printf(" [실패] 유효한 샘플을 하나도 못 읽었습니다. 오프셋 변경 안 함.\n");
    }
    fflush(stdout);
    usleep(800000); /* 결과를 잠깐 보여준 뒤 메인 화면으로 복귀 */
}

int main(void)
{
    signal(SIGINT, handle_sigint);
    enable_raw_mode();

    g_fd = open(DEV_IMU, O_RDONLY);
    if (g_fd < 0) {
        perror("장치 오픈 실패 (" DEV_IMU ")");
        restore_terminal();
        return -1;
    }

    /* 오프셋은 항상 0에서 시작 — 파일에서 안 불러옴.
     * 실행 중 'z' 키로 그때그때 잡는다. */
    float offset_r = 0.0f, offset_p = 0.0f;

    int count = 0;
    int err_cnt = 0;

    while (g_running) {
        char key = poll_keypress();
        if (key == 'z' || key == 'Z') {
            do_zero_calibration(g_fd, &offset_r, &offset_p);
            continue; /* 캘리브레이션 직후 화면은 다음 루프에서 바로 갱신 */
        } else if (key == 'r' || key == 'R') {
            offset_r = 0.0f;
            offset_p = 0.0f;
        }

        float raw_r = 0.0f, raw_p = 0.0f;
        int ret = read_sensor_angles(g_fd, &raw_r, &raw_p);

        /* 화면 지우기: system("clear") 대신 ANSI escape 사용
         * (fork/exec 오버헤드 및 깜빡임 감소) */
        printf("\033[H\033[J");

        printf("==================================================\n");
        printf("     /dev/imu ICM-20948 실시간 테스트 (0.1s)      \n");
        printf("==================================================\n");

        if (ret == 0) {
            float calib_r = raw_r - offset_r;
            float calib_p = raw_p - offset_p;

            printf(" [영점 오프셋] Roll: %+.3f°  | Pitch: %+.3f°\n", offset_r, offset_p);
            printf(" [실측 Raw 각] Roll: %+.3f°  | Pitch: %+.3f°\n", raw_r, raw_p);
            printf("--------------------------------------------------\n");
            printf(" [보정 설치각] Roll: %+.3f°  | Pitch: %+.3f°\n", calib_r, calib_p);
            printf("--------------------------------------------------\n");

            if (fabsf(calib_r) <= LEVEL_THRESHOLD && fabsf(calib_p) <= LEVEL_THRESHOLD) {
                printf(" [상태]  \033[1;32m[ OK ] 수평 유지 중 (Scan & Operable)\033[0m   (#%d)\n", ++count);
            } else {
                printf(" [상태]  \033[1;31m[WARN] 수평 이탈! (Turret Blocked)\033[0m   (#%d)\n", ++count);
            }
        } else {
            printf(" [오류]  \033[1;31m /dev/imu 읽기 실패 (누적 에러: %d회)\033[0m\n", ++err_cnt);
            printf("--------------------------------------------------\n");
            printf(" [상태]  \033[1;31m[ FAIL ] 드라이버 응답 없음\033[0m\n");
        }

        printf("==================================================\n");
        printf(" [z] 영점 잡기   [r] 오프셋 리셋   [Ctrl+C] 종료\n");
        fflush(stdout);

        usleep(100000);
    }

    printf("\n종료 중... fd 정리\n");
    close(g_fd);
    restore_terminal();
    return 0;
}