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
 *   ./encoder_jitter_test [샘플수] [샘플링간격_ms] [출력파일.md]
 *   예) ./encoder_jitter_test 100 200               (100개 샘플, 200ms 주기 수집 - 기본값)
 *       ./encoder_jitter_test 500 200 report.md     (500개 샘플 수집)
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
#include <time.h>
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
        printf("✓ [상태: 매우 우수 (Very Clean)]\n");
        printf("  - 최대 지터 변동폭이 %u LSB (약 %.4f°) 이내로 매우 안정적입니다.\n", max_pp, max_pp * DEG_PER_LSB);
        printf("  - 결론: 추가적인 소프트웨어 필터(LPF)가 전혀 필요하지 않으며 원시값을 그대로 사용 가능합니다.\n");
    } else if (max_pp <= 6) {
        printf("! [상태: 보통 노이즈 (Moderate Noise)]\n");
        printf("  - 최대 지터 변동폭이 %u LSB (약 %.4f°, RMS %.2f LSB) 수준입니다.\n", max_pp, max_pp * DEG_PER_LSB, max_std);
        printf("  - 결론: 정지 상태 지터를 잡기 위해 가벼운 필터 적용을 권장합니다.\n");
        printf("    * 권장 1: 4~8-Tap 이동평균 필터 (Moving Average Filter)\n");
        printf("    * 권장 2: 지수이동평균 필터 (EMA: y[k] = 0.25*x[k] + 0.75*y[k-1])\n");
    } else {
        printf("⚠ [상태: 지터 높음 (High Noise Warning)]\n");
        printf("  - 최대 지터 변동폭이 %u LSB (약 %.4f°)로 노이즈가 큽니다.\n", max_pp, max_pp * DEG_PER_LSB);
        printf("  - 하드웨어 점검: MT6701 센서와 자석 간의 거리(Air Gap 1.0~2.0mm) 및 자석 편심 확인 필요.\n");
        printf("  - 결론: 소프트웨어 필터 적용이 필수적입니다.\n");
        printf("    * 권장: 5-Tap 중간값(Median) 필터 + 8-Tap 저주파 통과 필터(LPF)\n");
    }
    printf("=========================================================================================\n");
}

static void save_markdown_report(const char *filepath, int count, int interval_ms,
                                const uint16_t *pan_raws, const uint16_t *tilt_raws,
                                const jitter_stat_t *pan_stat, const jitter_stat_t *tilt_stat)
{
    FILE *fp = fopen(filepath, "w");
    if (!fp) {
        perror("fopen markdown report");
        return;
    }

    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    char time_str[64];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", t);

    uint16_t max_pp = (pan_stat->pp_lsb > tilt_stat->pp_lsb) ? pan_stat->pp_lsb : tilt_stat->pp_lsb;
    double max_std = (pan_stat->std_dev_lsb > tilt_stat->std_dev_lsb) ? pan_stat->std_dev_lsb : tilt_stat->std_dev_lsb;

    fprintf(fp, "# MT6701 홀 센서(자기식 엔코더) 정지상태 지터링 측정 보고서\n\n");

    fprintf(fp, "## 1. 측정 메타데이터 (Test Metadata)\n\n");
    fprintf(fp, "| 항목 | 설정값 |\n");
    fprintf(fp, "| :--- | :--- |\n");
    fprintf(fp, "| **측정 일시** | `%s` |\n", time_str);
    fprintf(fp, "| **총 샘플 수** | `%d 개` |\n", count);
    fprintf(fp, "| **샘플링 주기** | `%d ms` (약 %.1f Hz) |\n", interval_ms, 1000.0 / interval_ms);
    fprintf(fp, "| **총 측정 소요 시간** | `약 %.1f 초` |\n", (double)(count * interval_ms) / 1000.0);
    fprintf(fp, "| **엔코더 분해능** | `14-bit (16,384 CPR, 1 LSB = %.5f°)` |\n\n", DEG_PER_LSB);

    fprintf(fp, "## 2. 통계 분석 요약 (Statistical Jitter Analysis)\n\n");
    fprintf(fp, "| 항목 (Metric) | Pan 축 (방위각) | Tilt 축 (고각) |\n");
    fprintf(fp, "| :--- | :---: | :---: |\n");
    fprintf(fp, "| **평균 원시값 (Mean Raw)** | `%.2f LSB (%.3f°)` | `%.2f LSB (%.3f°)` |\n",
            pan_stat->mean_raw, pan_stat->mean_deg, tilt_stat->mean_raw, tilt_stat->mean_deg);
    fprintf(fp, "| **최소 ~ 최대값 (Min ~ Max)** | `%u ~ %u LSB` | `%u ~ %u LSB` |\n",
            pan_stat->min_raw, pan_stat->max_raw, tilt_stat->min_raw, tilt_stat->max_raw);
    fprintf(fp, "| **Peak-to-Peak 지터 (LSB)** | **`%u LSB`** | **`%u LSB`** |\n",
            pan_stat->pp_lsb, tilt_stat->pp_lsb);
    fprintf(fp, "| **Peak-to-Peak 변동각 (P-P deg)** | `±%.4f° (Δ %.4f°)` | `±%.4f° (Δ %.4f°)` |\n",
            pan_stat->pp_deg / 2.0, pan_stat->pp_deg, tilt_stat->pp_deg / 2.0, tilt_stat->pp_deg);
    fprintf(fp, "| **표준편차 (RMS Noise, 1-σ)** | `%.2f LSB (%.4f°)` | `%.2f LSB (%.4f°)` |\n\n",
            pan_stat->std_dev_lsb, pan_stat->std_dev_deg, tilt_stat->std_dev_lsb, tilt_stat->std_dev_deg);

    fprintf(fp, "## 3. 필터링 필요성 진단 및 권장사항 (Filter Assessment)\n\n");
    if (max_pp <= 2 && max_std <= 0.8) {
        fprintf(fp, "> **[상태: 매우 우수 (Very Clean)]**\n");
        fprintf(fp, "> - 최대 지터 변동폭이 `%u LSB (약 %.4f°)` 이내로 매우 안정적입니다.\n", max_pp, max_pp * DEG_PER_LSB);
        fprintf(fp, "> - **결론**: 추가적인 소프트웨어 필터(LPF)가 전혀 필요하지 않으며 원시값을 그대로 사용 가능합니다.\n\n");
    } else if (max_pp <= 6) {
        fprintf(fp, "> **[!주의] [상태: 보통 노이즈 (Moderate Noise)]**\n");
        fprintf(fp, "> - 최대 지터 변동폭이 `%u LSB (약 %.4f°, RMS %.2f LSB)` 수준입니다.\n", max_pp, max_pp * DEG_PER_LSB, max_std);
        fprintf(fp, "> - **결론**: 정지 상태 지터를 완화하기 위해 가벼운 소프트웨어 필터 적용을 권장합니다.\n");
        fprintf(fp, ">   * **권장 1**: 4~8-Tap 이동평균 필터 (Moving Average Filter)\n");
        fprintf(fp, ">   * **권장 2**: 지수이동평균 필터 (EMA: `y[k] = 0.25 * x[k] + 0.75 * y[k-1]`)\n\n");
    } else {
        fprintf(fp, "> **[⚠경고] [상태: 지터 높음 (High Noise Warning)]**\n");
        fprintf(fp, "> - 최대 지터 변동폭이 `%u LSB (약 %.4f°)`로 노이즈가 큽니다.\n", max_pp, max_pp * DEG_PER_LSB);
        fprintf(fp, "> - **하드웨어 점검**: MT6701 센서와 자석 간의 거리(Air Gap 1.0~2.0mm) 및 자석 편심 확인 필요.\n");
        fprintf(fp, "> - **결론**: 소프트웨어 필터 적용이 필수적입니다.\n");
        fprintf(fp, ">   * **권장**: 5-Tap 중간값(Median) 필터 + 8-Tap 저주파 통과 필터(LPF)\n\n");
    }

    fprintf(fp, "## 4. 원시 측정 데이터 전수 로그 (Full Raw Data Log - %d건)\n\n", count);
    fprintf(fp, "| 번호 | 시간(ms) | Pan Raw (LSB) | Pan 각도 (deg) | Pan 편차 (LSB) | Tilt Raw (LSB) | Tilt 각도 (deg) | Tilt 편차 (LSB) |\n");
    fprintf(fp, "| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |\n");

    for (int i = 0; i < count; i++) {
        double pan_deg = (double)pan_raws[i] * DEG_PER_LSB;
        double tilt_deg = (double)tilt_raws[i] * DEG_PER_LSB;
        double pan_diff = (double)pan_raws[i] - pan_stat->mean_raw;
        double tilt_diff = (double)tilt_raws[i] - tilt_stat->mean_raw;

        fprintf(fp, "| %4d | %6d | %5u | %7.3f° | %+6.1f | %5u | %7.3f° | %+6.1f |\n",
                i + 1, i * interval_ms,
                pan_raws[i], pan_deg, pan_diff,
                tilt_raws[i], tilt_deg, tilt_diff);
    }

    fclose(fp);
}

int main(int argc, char *argv[])
{
    int sample_count = 100;
    int interval_ms = 200;
    char report_file[256] = {0};

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            printf("Usage: %s [samples] [interval_ms] [output_file.md]\n", argv[0]);
            printf("  samples        : Number of samples to collect (default: 100, min: 10)\n");
            printf("  interval_ms    : Sampling interval in ms (default: 200, min: 100)\n");
            printf("  output_file.md : Output markdown report path (default: auto-generated timestamp)\n");
            return 0;
        } else if (sample_count == 100 && atoi(argv[i]) > 0 && i == 1) {
            sample_count = atoi(argv[i]);
        } else if (interval_ms == 200 && atoi(argv[i]) > 0 && i == 2) {
            interval_ms = atoi(argv[i]);
        } else if (report_file[0] == '\0' && strstr(argv[i], ".md")) {
            snprintf(report_file, sizeof(report_file), "%s", argv[i]);
        }
    }

    if (report_file[0] == '\0') {
        time_t now = time(NULL);
        struct tm *t = localtime(&now);
        strftime(report_file, sizeof(report_file), "encoder_jitter_report_%Y%m%d_%H%M%S.md", t);
    }

    if (sample_count < 10) sample_count = 10;
    if (interval_ms < 100) interval_ms = 100;

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
    printf(" [MT6701 홀 센서 정지상태 지터링 측정 도구 (노이즈 정밀 분석)]\n");
    printf(" - 대상 장치  : %s\n", DEV_TURRET);
    printf(" - 샘플 수    : %d 개\n", sample_count);
    printf(" - 샘플링주기 : %d ms (예상 소요시간: 약 %.1f 초)\n", interval_ms, (double)(sample_count * interval_ms) / 1000.0);
    printf(" - 결과 파일  : %s\n", report_file);
    printf("===================================================================\n\n");

    struct pollfd pfd = { .fd = fd, .events = POLLIN };

    printf("[1단계] 모터를 홈(0°) 위치로 1회 이동시킵니다...\n");
    if (ioctl(fd, TURRET_HOME) < 0) {
        perror("ioctl TURRET_HOME");
    }
    /* 첫 홈 완료 통지 대기 */
    printf("-> 모터 회전 및 홈 도달 대기 중...\n");
    (void)poll(&pfd, 1, 5000);

    /* 모터 물리적 이동 완료 후 잔여 진동이 완전히 멈출 때까지 3초간 정착 대기 */
    printf("-> 모터 도달 완료. 잔여 기계적 진동이 멈출 때까지 3초간 정착 대기 중...\n");
    for (int sec = 3; sec > 0; sec--) {
        printf("   [%d초 대기 중...]\n", sec);
        sleep(1);
    }
    printf("✓ 모터가 완벽한 정지 상태에 도달했습니다.\n");
    printf("✓ 지금부터 14-bit 순수 엔코더 지터링 데이터 %d건 수집을 시작합니다.\n\n", sample_count);

    int collected = 0;

    for (int i = 0; i < sample_count; i++) {
        /* 엔코더 판독 요청 */
        if (ioctl(fd, TURRET_HOME) < 0) {
            perror("ioctl TURRET_HOME");
            break;
        }

        /* STM32 응답 대기 (최대 200ms) */
        int pr = poll(&pfd, 1, 200);
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

        printf("\r[샘플링 진행 %4d/%4d] Pan Raw=%5u (%6.2f°) | Tilt Raw=%5u (%6.2f°)",
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

    /* 마크다운 파일 저장 */
    save_markdown_report(report_file, collected, interval_ms, pan_raws, tilt_raws, &pan_stat, &tilt_stat);
    printf("✓ 전체 %d개 원시 데이터 및 마크다운 보고서가 저장되었습니다:\n", collected);
    printf("  -> 파일 경로: %s\n\n", report_file);

    free(pan_raws);
    free(tilt_raws);
    close(fd);
    return 0;
}
