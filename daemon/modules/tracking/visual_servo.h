#ifndef VISUAL_SERVO_H
#define VISUAL_SERVO_H

#include <stdint.h>

#define PROTO_VERSION     3
#define ANGLE_SCALE       10.0f  // 1도 = 10 ddeg

// [아이폰 카메라 및 스텝 모터 규격]
#define PHONE_WIDTH       64
#define PHONE_HEIGHT      48
#define PHONE_CENTER_X    (PHONE_WIDTH / 2.0)
#define PHONE_CENTER_Y    (PHONE_HEIGHT / 2.0)
#define PHONE_FOV_X       60.0
#define PHONE_FOV_Y       40.0

#define PAN_STEPS_PER_REV  12800.0
#define TILT_STEPS_PER_REV 12800.0
#define STEP_TO_DEG_PAN   (360.0 / PAN_STEPS_PER_REV)
#define STEP_TO_DEG_TILT  (360.0 / TILT_STEPS_PER_REV)

// [CCTV 카메라 사양]
#define CCTV_WIDTH        1920
#define CCTV_HEIGHT       1080
#define CCTV_CENTER_X     (CCTV_WIDTH / 2.0)
#define CCTV_CENTER_Y     (CCTV_HEIGHT / 2.0)
#define CCTV_FOV_X        90.0
#define CCTV_FOV_Y        60.0
#define CCTV_ORIGIN_THETA 180.0
#define CCTV_ORIGIN_PHI   15.0

// 구조체 정의
struct proto_status {
    int16_t cur_theta_ddeg;
    int16_t cur_phi_ddeg;
};

struct proto_target {
    int16_t theta_ddeg;
    int16_t phi_ddeg;
};

struct point3d {
    double x;
    double y;
    double z;
};

struct velocity3d {
    double vx;
    double vy;
    double vz;
};

// --- 함수 선언 ---
struct proto_target Tracking_ConvertCCTVToAngle(double x, double y);
struct proto_target Tracking_CalculateTargetPacket(double target_x, double target_y, double dist, struct proto_status current_status);

void Tracking_StepsToAngle(int32_t pan_steps, int32_t tilt_steps, int16_t *out_theta_ddeg, int16_t *out_phi_ddeg);
struct point3d Tracking_Calc3D(double dist, int16_t theta_ddeg, int16_t phi_ddeg);
struct velocity3d Tracking_CalcVelocity(struct point3d curr, struct point3d prev, double dt);

#endif // VISUAL_SERVO_H