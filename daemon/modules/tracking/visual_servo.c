#include "visual_servo.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief CCTV의 2D 픽셀 좌표(x, y)를 터렛 기준의 절대 방위각/고각 패킷으로 변환합니다. (1차 조준 유도용)
 */
struct proto_target Tracking_ConvertCCTVToAngle(double x, double y) {
    struct proto_target packet = {0};

    // 1. CCTV 화면 중심 대비 픽셀 오차 계산
    double dx = x - CCTV_CENTER_X;
    double dy = CCTV_CENTER_Y - y;

    // 2. 픽셀 오차를 기하학적 각도 오차(Degree)로 변환
    double d_theta = dx * (CCTV_FOV_X / CCTV_WIDTH);
    double d_phi = dy * (CCTV_FOV_Y / CCTV_HEIGHT);

    // 3. CCTV의 설치 방위 오리진에 오차 각도를 더해 터렛이 조준할 절대각 산출
    double target_th = CCTV_ORIGIN_THETA + d_theta;
    double target_ph = CCTV_ORIGIN_PHI + d_phi;

    // 4. 모터 구동 범위 보호를 위한 안전 리미트 (Soft Limit)
    if (target_th < 0.0)   target_th = 0.0;
    if (target_th > 360.0) target_th = 360.0;
    if (target_ph < -30.0) target_ph = -30.0;
    if (target_ph > 90.0)  target_ph = 90.0;

    // 5. 정수형 ddeg 형태로 패키징
    packet.theta_ddeg = (int16_t)round(target_th * ANGLE_SCALE);
    packet.phi_ddeg = (int16_t)round(target_ph * ANGLE_SCALE);

    return packet;
}

/**
 * @brief 아이폰 라이다 카메라상의 픽셀 오차와 실측 거리(dist)를 기반으로 정밀 3D 비주얼 서보잉 각도를 연산합니다.
 */
struct proto_target Tracking_CalculateTargetPacket(double target_x, double target_y, double dist, struct proto_status current_status) {
    struct proto_target packet = {0};

    // 1. 아이폰 라이다 화면 중심 대비 픽셀 오차 계산
    double delta_x = target_x - PHONE_CENTER_X;
    double delta_y = PHONE_CENTER_Y - target_y;

    // 2. 원근법 오차 극복을 위한 거리 가중치(Dynamic Gain Scaling) 적용
    double distance_weight = 1.0;
    if (dist > 0.1) {
        distance_weight = 3.0 / dist;
        if (distance_weight > 2.0) distance_weight = 2.0;
        if (distance_weight < 0.5) distance_weight = 0.5;
    }

    // 3. 거리 가중치가 결합된 정밀 각도 편차 연산
    double delta_theta = (delta_x * (PHONE_FOV_X / PHONE_WIDTH)) * distance_weight;
    double delta_phi = (delta_y * (PHONE_FOV_Y / PHONE_HEIGHT)) * distance_weight;

    // 4. STM32의 현재 실시간 반환 각도에 보정 각도를 누적하여 절대 목표각 연산
    double target_theta = ((double)current_status.cur_theta_ddeg / ANGLE_SCALE) + delta_theta;
    double target_phi = ((double)current_status.cur_phi_ddeg / ANGLE_SCALE) + delta_phi;

    // 안전 리미트 (Soft Limit)
    if (target_theta < 0.0)   target_theta = 0.0;
    if (target_theta > 360.0) target_theta = 360.0;
    if (target_phi < -30.0)   target_phi = -30.0;
    if (target_phi > 90.0)    target_phi = 90.0;

    packet.theta_ddeg = (int16_t)round(target_theta * ANGLE_SCALE);
    packet.phi_ddeg = (int16_t)round(target_phi * ANGLE_SCALE);

    return packet;
}

/**
 * @brief 스텝 카운트 값을 실제 매칭되는 피드백 각도(ddeg) 단위로 환산합니다.
 */
void Tracking_StepsToAngle(int32_t pan_steps, int32_t tilt_steps, int16_t *out_theta_ddeg, int16_t *out_phi_ddeg) {
    double theta_deg = (double)pan_steps * STEP_TO_DEG_PAN;
    double phi_deg   = (double)tilt_steps * STEP_TO_DEG_TILT;

    *out_theta_ddeg = (int16_t)round(theta_deg * ANGLE_SCALE);
    *out_phi_ddeg   = (int16_t)round(phi_deg * ANGLE_SCALE);
}

/**
 * @brief 라이다 실측 거리와 조준 각도를 활용해 터렛 원점 기준의 3D 물리 공간 좌표(X, Y, Z)를 추출합니다.
 */
struct point3d Tracking_Calc3D(double dist, int16_t theta_ddeg, int16_t phi_ddeg) {
    struct point3d pt = {0};

    double rad_th = ((double)theta_ddeg / ANGLE_SCALE) * (M_PI / 180.0);
    double rad_ph = ((double)phi_ddeg / ANGLE_SCALE) * (M_PI / 180.0);

    pt.x = dist * cos(rad_ph) * cos(rad_th);
    pt.y = dist * cos(rad_ph) * sin(rad_th);
    pt.z = dist * sin(rad_ph);

    return pt;
}

/**
 * @brief 시간 변위량(dt) 동안의 3D 위치 오차를 기반으로 이동 속도를 수치 미분합니다.
 */
struct velocity3d Tracking_CalcVelocity(struct point3d curr, struct point3d prev, double dt) {
    struct velocity3d vel = {0};
    if (dt <= 1e-6) return vel;

    vel.vx = (curr.x - prev.x) / dt;
    vel.vy = (curr.y - prev.y) / dt;
    vel.vz = (curr.z - prev.z) / dt;

    return vel;
}