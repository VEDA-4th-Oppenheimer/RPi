#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <math.h>
#include <time.h>
#include "visual_servo.h"

#define PORT 8080
#define DATA_COUNT (PHONE_WIDTH * PHONE_HEIGHT)
#define PACKET_SIZE (DATA_COUNT * sizeof(float))

int main() {
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1, addrlen = sizeof(address);
    float depth_frame[DATA_COUNT];

    struct point3d prev_pos = {0}, curr_pos = {0};
    struct timespec last_time, current_time;

    // 1. TCP 수신 서버 구성
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) return -1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0 || listen(server_fd, 3) < 0) return -1;
    printf("📡 [Oppenheimer] CCTV 연동형 1D LiDAR 추적 서버 대기 중 (Port: %d)...\n", PORT);

    while (1) {
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) continue;
        printf("\n✅ 아이폰(라이다 에뮬레이터) 및 CCTV 에이전트 연결 성공!\n");

        clock_gettime(CLOCK_MONOTONIC, &last_time);

        // 가상의 실시간 CCTV 픽셀 스트리밍 상태 변수 (드론이 CCTV 화면 중앙에서 우상단으로 이동하는 시나리오)
        double cctv_drone_x = 960.0;
        double cctv_drone_y = 540.0;

        while (1) {
            // [시뮬레이션] CCTV가 실시간으로 드론의 x, y 좌표를 주는 상황을 가상 업데이트
            // 매 루프마다 드론이 우상단으로 슬금슬금 움직인다고 가정
            cctv_drone_x += 1.5;
            cctv_drone_y -= 0.8; // 픽셀 좌표계 상 y가 작아져야 실제 하늘 위로 날아가는 것
            if (cctv_drone_x > 1800.0) cctv_drone_x = 960.0;
            if (cctv_drone_y < 100.0)  cctv_drone_y = 540.0;

            // 1단계: 실시간 CCTV 픽셀 기반 터렛 방위각/고각 산출 (1차 광역 유도)
            struct proto_target coarse_cmd = Tracking_ConvertCCTVToAngle(cctv_drone_x, cctv_drone_y);
            struct proto_status mock_status = {
                .cur_theta_ddeg = coarse_cmd.theta_ddeg,
                .cur_phi_ddeg = coarse_cmd.phi_ddeg
            };

            // 2단계: 아이폰 TCP 소켓 데이터 수신 (실시간 Depth 데이터 스트림)
            size_t total_received = 0;
            char *buf_ptr = (char *)depth_frame;

            while (total_received < PACKET_SIZE) {
                ssize_t r = read(new_socket, buf_ptr + total_received, PACKET_SIZE - total_received);
                if (r <= 0) goto client_disconnected;
                total_received += r;
            }

            // 3단계: 1D 라이다 가상 필터링 적용 (중앙 4x4 영역 평균 거리 측정)
            int start_x = (PHONE_WIDTH / 2) - 2;
            int start_y = (PHONE_HEIGHT / 2) - 2;
            double sum_dist = 0.0;
            int valid_count = 0;

            for (int y = start_y; y < start_y + 4; y++) {
                for (int x = start_x; x < start_x + 4; x++) {
                    float val = depth_frame[y * PHONE_WIDTH + x];
                    if (val > 0.1f && val < 9.0f) {
                        sum_dist += val;
                        valid_count++;
                    }
                }
            }
            double mock_1d_dist = (valid_count > 0) ? (sum_dist / valid_count) : depth_frame[(PHONE_HEIGHT/2)*PHONE_WIDTH + (PHONE_WIDTH/2)];

            // 4단계: 주기 dt 계측
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            double dt = (current_time.tv_sec - last_time.tv_sec) + (current_time.tv_nsec - last_time.tv_nsec) / 1000000000.0;
            last_time = current_time;

            // 5단계: CCTV 타겟팅 각도와 라이다 정밀 거리를 물리 결합하여 정밀 3D (X, Y, Z) 좌표 도출
            curr_pos = Tracking_Calc3D(mock_1d_dist, mock_status.cur_theta_ddeg, mock_status.cur_phi_ddeg);

            // 6단계: 실시간 미분 속도 산출
            struct velocity3d vel = Tracking_CalcVelocity(curr_pos, prev_pos, dt);
            double speed = sqrt(vel.vx*vel.vx + vel.vy*vel.vy + vel.vz*vel.vz) * 3.6; // km/h

            prev_pos = curr_pos;

            // 최종 CCTV 입력 좌표값과 3D 물리 공간 합성 좌표 출력
            printf("\r📺 CCTV Cam: (%7.1f, %7.1f)px | 🎯 Dist: %.2fm | Pos: (X: %6.2f, Y: %6.2f, Z: %6.2f)m | Speed: %5.1fkm/h",
                   cctv_drone_x, cctv_drone_y, mock_1d_dist, curr_pos.x, curr_pos.y, curr_pos.z, speed);
            fflush(stdout);
        }

    client_disconnected:
        close(new_socket);
        printf("\n🔄 연결 종료. 다음 세션 대기...\n");
    }

    close(server_fd);
    return 0;
}