int main(void) {
    carrinho_state_t s = {0};
    s.pose.x_m = 1.24f; s.pose.y_m = 0.58f; s.pose.yaw_rad = 0.31f;
    s.pose.yaw_odom_rad = 0.29f;
    s.wheel.v_linear_mps = 0.18f; s.wheel.v_left_mps = 0.16f; s.wheel.v_right_mps = 0.20f;
    float d[4] = {0.87f, 1.45f, 0.63f, 0.71f};
    int64_t age_ms[4] = {300, 200, 100, 0};
    for (int i=0;i<4;i++){ s.sonar[i].valid=true; s.sonar[i].dist_m=d[i];
                           s.sonar[i].stamp_us = s_now - age_ms[i]*1000; }
    s.imu.valid = true; s.imu.stamp_us = s_now;
    s.imu.accel_m_s2[0]=0.02f; s.imu.accel_m_s2[1]=0.01f; s.imu.accel_m_s2[2]=9.81f;
    s.imu.gyro_rad_s[2]=0.16f;
    char buf[1024];
    int n = build_payload(buf, sizeof(buf), &s);
    fprintf(stderr, "bytes amostra completa: %d\n", n);
    printf("%s\n", buf);
    carrinho_state_t z = {0};
    z.safety.timeout_active = true;
    n = build_payload(buf, sizeof(buf), &z);
    fprintf(stderr, "bytes amostra com nulos: %d\n", n);
    printf("%s\n", buf);
    return 0;
}
