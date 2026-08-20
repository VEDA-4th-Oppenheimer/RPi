/*
 * encoder_jitter_test.c — MT6701 홀 센서(자기식 절대 엔코더) 정지상태 지터링 분석 테스트
 *
 * [목적]
 *   정지 상태에서 MT6701 자기식 엔코더(14비트, 16384 CPR)의 원시 데이터(Raw) 노이즈와
 *   지터(Jitter: Peak-to-Peak 변동폭, RMS/표준편차)를 측정하여,
 *   추가 소프트웨어 필터(LPF, Moving Average, EMA 등) 적용 필요성을 통계적으로 판정합니다.
 *
 * [빌드 방법]
 *   gcc -O2 -Wall -Wextra -DPROTO_WANT_IOCTL -I../shared -o encoder_jitter_test encoder_jitter_test.c -lm
 *
 * [사용법]
 *   ./encoder_jitter_test [샘플수] [샘플링간격_ms]
 *   예) ./encoder_jitter_test 100 30    (100개 샘플, 30ms 주기 수집 - 기본값)
 *       ./encoder_jitter_test 300 20    (300개 샘플, 20ms 주기 수집)
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>
#include <math.h>
#include <sys/ioctl.h>

#ifndef PROTO_WANT_IOCTL
#define PROTO_WANT_IOCTL
#endif
#include "protocol.h"

#define DEV_TURRET "/dev/turret"

/* 14비트 엔코더 상수 (MT6701: 0 ~ 16383) */
#define ENCODER_RESOLUTION  16384.0
#define DEG_PER_LSB         (360.0 / ENCODER_RESOLUTION) /* 약 0.02197265625도 */

typedef struct {
    uint16_t min_raw;
    uint16_t max_raw;
    uint16_t pp_lsb;        /* Peak-to-Peak 지터 (LSB) */
    double   mean_raw;
    double   mean_deg;
    double   pp_deg;        /* Peak-to-Peak 변동각 (deg) */
    double   std_dev_lsb;   /* 1-sigma RMS 노이즈 (LSB) */
    double   std_dev_deg;   /* 1-sigma RMS 노이즈 (deg) */
} jitter_stat_t;

static void compute_statistics(const uint16_t *raw_buf, int count, jitter_stat_t *stat)
{
    uint16_t min_val = 0xFFFF;
    uint16_t max_val = 0;
    double sum = 0.0;

    for (int i = 0; i < count; i++) {
        if (raw_buf[i] < min_val) min_val = raw_buf[i];
        if (raw_buf[i] > max_val) max_val = raw_buf[i];
        sum += (double)raw_buf[i];
    }

    stat->min_raw = min_val;
    stat->max_raw = max_val;
    stat->pp_lsb = max_val - min_val;
    stat->mean_raw = sum / (double)count;
    stat->mean_deg = stat->mean_raw * DEG_PER_LSB;
    stat->pp_deg = (double)stat->pp_lsb * DEG_PER_LSB;

    double var_sum = 0.0;
    for (int i = 0; i < count; i++) {
        double diff = (double)raw_buf[i] - stat->mean_raw;
        var_sum += diff * diff;
    }
    stat->std_dev_lsb = sqrt(var_sum / (double)count);
    stat->std_dev_deg = stat->std_dev_lsb * DEG_PER_LSB;
}

static void print_report(int count, int interval_ms,
                         const jitter_stat_t *pan_stat,
                         const jitter_stat_t *tilt_stat)
{
    printf("\n");
    printf("=========================================================================================\n");
    printf("                     MT6701 홀 센서 정지상태 지터링(Jitter) 측정 보고서                  \n");
    printf("=========================================================================================\n");
    printf(" - 수집 샘플 수  : %d 개\n", count);
    printf(" - 샘플링 주기   : %d ms (약 %.1f Hz)\n", interval_ms, 1000.0 / interval_ms);
    printf(" - 엔코더 분해능 : 14-bit (16,384 CPR, 1 LSB = %.5f°)\n", DEG_PER_LSB);
    printf("-----------------------------------------------------------------------------------------\n");
    printf("  항목 (Metric)                  │  Pan 축 (방위각)             │  Tilt 축 (고각)         \n");
    printf("--------------------------------─┼──────────────────────────────┼────────────────────────\n");
    printf("  평균 원시값 (Mean Raw)         │  %8.2f LSB (%7.3f°)       │  %8.2f LSB (%7.3f°) \n",
           pan_stat->mean_raw, pan_stat->mean_deg, tilt_stat->mean_raw, tilt_stat->mean_deg);
    printf("  최소 ~ 최대값 (Min ~ Max)      │  %5u ~ %5u LSB             │  %5u ~ %5u LSB       \n",
           pan_stat->min_raw, pan_stat->max_raw, tilt_stat->min_raw, tilt_stat->max_raw);
    printf("  Peak-to-Peak 지터 (LSB)        │  %5u LSB                     │  %5u LSB             \n",
           pan_stat->pp_lsb, tilt_stat->pp_lsb);
    printf("  Peak-to-Peak 변동각 (Δ deg)    │  ±%6.4f° (전체 Δ %6.4f°)    │  ±%6.4f° (전체 Δ %6.4f°)\n",
           pan_stat->pp_deg / 2.0, pan_stat->pp_deg, tilt_stat->pp_deg / 2.0, tilt_stat->pp_deg);
    printf("  표준편차 (RMS Noise, 1-σ)      │   %5.2f LSB ( %6.4f°)        │   %5.2f LSB ( %6.4f°) \n",
           pan_stat->std_dev_lsb, pan_stat->std_dev_deg, tilt_stat->std_dev_lsb, tilt_stat->std_dev_deg);
    printf("=========================================================================================\n\n");

    /* 필터링 필요성 자동 진단 */
    uint16_t max_pp = (pan_stat->pp_lsb > tilt_stat->pp_lsb) ? pan_stat->pp_lsb : tilt_stat->pp_lsb;
    double max_std = (pan_stat->std_dev_lsb > tilt_stat->std_dev_lsb) ? pan_stat->std_dev_lsb : tilt_stat->std_dev_lsb;

    printf("[ 필터(Filter) 적용 필요성 분석 및 가이드 ]\n");
    if (max_pp <= 2 && max_std <= 0.8) {
        printf(" [상태: 매우 우수 (Very Clean)]\n");
        printf("  - 최대 지터 변동폭이 %u LSB (약 %.4f°) 이내로 매우 안정적입니다.\n", max_pp, max_pp * DEG_PER_LSB);
        printf("  - 결론: 추가적인 소프트웨어 필터(LPF)가 전혀 필요하지 않으며 원시값을 그대로 사용 가능합니다.\n");
    } else if (max_pp <= 6) {
        printf("! [상태: 보통 노이즈 (Moderate Noise)]\n");
        printf("  - 최대 지터 변동폭이 %u LSB (약 %.4f°, RMS %.2f LSB) 수준입니다.\n", max_pp, max_pp * DEG_PER_LSB, max_std);
        printf("  - 결론: 정지 상태 지터를 잡기 위해 가벼운 필터 적용을 권장합니다.\n");
        printf("    * 권장 1: 4~8-Tap 이동평균 필터 (Moving Average Filter)\n");
        printf("    * 권장 2: 지수이동평균 필터 (EMA: y[k] = 0.25*x[k] + 0.75*y[k-1])\n");
    } else {
        printf(" [상태: 지터 높음 (High Noise Warning)]\n");
        printf("  - 최대 지터 변동폭이 %u LSB (약 %.4f°)로 노이즈가 큽니다.\n", max_pp, max_pp * DEG_PER_LSB);
        printf("  - 하드웨어 점검: MT6701 센서와 자석 간의 거리(Air Gap 1.0~2.0mm) 및 자석 편심 확인 필요.\n");
        printf("  - 결론: 소프트웨어 필터 적용이 필수적입니다.\n");
        printf("    * 권장: 5-Tap 중간값(Median) 필터 + 8-Tap 저주파 통과 필터(LPF)\n");
    }
    printf("=========================================================================================\n");
}

int main(int argc, char *argv[])
{
    int sample_count = 100;
    int interval_ms = 30;

    if (argc >= 2) {
        if (!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help")) {
            printf("Usage: %s [samples] [interval_ms]\n", argv[0]);
            printf("  samples     : Number of samples to collect (default: 100, min: 10)\n");
            printf("  interval_ms : Sampling interval in milliseconds (default: 30, min: 10)\n");
            return 0;
        }
        sample_count = atoi(argv[1]);
    }
    if (argc >= 3) {
        interval_ms = atoi(argv[2]);
    }

    if (sample_count < 10) sample_count = 10;
    if (interval_ms < 10) interval_ms = 10;

    int fd = open(DEV_TURRET, O_RDWR);
    if (fd < 0) {
        perror("open " DEV_TURRET);
        fprintf(stderr, "-> 드라이버 로드 확인: sudo insmod turret_driver.ko\n");
        return 1;
    }

    uint16_t *pan_raws = malloc(sizeof(uint16_t) * (size_t)sample_count);
    uint16_t *tilt_raws = malloc(sizeof(uint16_t) * (size_t)sample_count);
    if (!pan_raws || !tilt_raws) {
        fprintf(stderr, "Memory allocation error\n");
        free(pan_raws);
        free(tilt_raws);
        close(fd);
        return 1;
    }

    printf("===================================================================\n");
    printf(" MT6701 홀 센서 정지상태 지터링 측정을 시작합니다...\n");
    printf(" 대상 장치: %s (samples=%d, interval=%dms)\n", DEV_TURRET, sample_count, interval_ms);
    printf("===================================================================\n");

    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    int collected = 0;

    for (int i = 0; i < sample_count; i++) {
        /* CMD_HOME ioctl 요청 */
        if (ioctl(fd, TURRET_HOME) < 0) {
            perror("ioctl TURRET_HOME");
            break;
        }

        /* STM32 응답 대기 (최대 100ms) */
        int pr = poll(&pfd, 1, 100);
        if (pr < 0) {
            if (errno == EINTR) {
                i--;
                continue;
            }
            perror("poll");
            break;
        }

        struct turret_link_state s;
        if (ioctl(fd, TURRET_GET_STATE, &s) < 0) {
            perror("ioctl TURRET_GET_STATE");
            break;
        }

        pan_raws[collected] = s.home_pan_encoder_raw;
        tilt_raws[collected] = s.home_tilt_encoder_raw;
        collected++;

        printf("\r[샘플링 진행 %3d/%3d] Pan Raw=%5u (%6.2f°) | Tilt Raw=%5u (%6.2f°)",
               collected, sample_count,
               s.home_pan_encoder_raw,
               (double)s.home_pan_encoder_raw * DEG_PER_LSB,
               s.home_tilt_encoder_raw,
               (double)s.home_tilt_encoder_raw * DEG_PER_LSB);
        fflush(stdout);

        usleep((useconds_t)interval_ms * 1000);
    }
    printf("\n");

    if (collected < 5) {
        fprintf(stderr, "샘플 수집 실패 (수집된 샘플: %d개)\n", collected);
        free(pan_raws);
        free(tilt_raws);
        close(fd);
        return 1;
    }

    jitter_stat_t pan_stat, tilt_stat;
    compute_statistics(pan_raws, collected, &pan_stat);
    compute_statistics(tilt_raws, collected, &tilt_stat);

    print_report(collected, interval_ms, &pan_stat, &tilt_stat);

    free(pan_raws);
    free(tilt_raws);
    close(fd);
    return 0;
}
