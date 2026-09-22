static void straight_line(void) {
    odom_pose_t p = {0};
    const float step = 0.001f;
    for (int i = 0; i < 2000; i++) {
        odom_step(&p, step, step, 0.0f, 0.02f, false);
    }
    printf("straight 2 m  -> x=%+.4f  y=%+.4f  yaw=%+.2f deg\n",
           p.x_m, p.y_m, p.yaw_rad * 180.0f / (float)M_PI);
}

static void spin_in_place(void) {
    odom_pose_t p = {0};
    /* dtheta = (dsR - dsL)/b with dsR = -dsL = s, so 90 deg needs s = pi*b/4. */
    float s_total = (float)M_PI * WHEEL_BASE_M / 4.0f;
    int n = 1000;
    for (int i = 0; i < n; i++) {
        odom_step(&p, -s_total/n, s_total/n, 0.0f, 0.02f, false);
    }
    printf("spin 90 deg   -> x=%+.4f  y=%+.4f  yaw=%+.2f deg\n",
           p.x_m, p.y_m, p.yaw_rad * 180.0f / (float)M_PI);
}

static void arc_90(void) {
    /* Outer wheel travels (R+b/2)*pi/2, inner (R-b/2)*pi/2; the center must end
     * at (R, R) facing 90 deg. */
    odom_pose_t p = {0};
    const float R = 0.5f;
    float arc_outer = (R + WHEEL_BASE_M/2.0f) * (float)M_PI / 2.0f;
    float arc_inner = (R - WHEEL_BASE_M/2.0f) * (float)M_PI / 2.0f;
    int n = 2000;
    for (int i = 0; i < n; i++) {
        odom_step(&p, arc_inner/n, arc_outer/n, 0.0f, 0.02f, false);
    }
    printf("arc R=0.5 m   -> x=%+.4f  y=%+.4f  yaw=%+.2f deg  (expected x=+0.5000 y=+0.5000 yaw=+90.00)\n",
           p.x_m, p.y_m, p.yaw_rad * 180.0f / (float)M_PI);
}

static void wrap(void) {
    odom_pose_t p = {0};
    float s = (float)M_PI * WHEEL_BASE_M / 4.0f * (350.0f/90.0f);
    int n = 5000;
    for (int i = 0; i < n; i++) { odom_step(&p, -s/n, s/n, 0.0f, 0.02f, false); }
    printf("spin 350 deg  -> yaw=%+.2f deg  (expected -10.00)\n",
           p.yaw_rad * 180.0f / (float)M_PI);
}

static void slip(void) {
    /* Encoders report 90 deg, gyro measures 60: the fused yaw must lean toward
     * the gyro. */
    odom_pose_t p = {0};
    float s = (float)M_PI * WHEEL_BASE_M / 4.0f;
    int n = 1000;
    float dt = 0.02f;
    float wz = (60.0f * (float)M_PI/180.0f) / (n * dt);
    for (int i = 0; i < n; i++) { odom_step(&p, -s/n, s/n, wz, dt, true); }
    printf("wheel slip    -> fused yaw=%+.2f  odometry yaw=%+.2f deg\n",
           p.yaw_rad * 180.0f/(float)M_PI, p.yaw_odom_rad * 180.0f/(float)M_PI);
}

/* Alpha sweep: 90 deg turn in 1.5 s with 30 deg of slip. */
static void sweep(void) {
    float alphas[] = {0.90f, 0.98f, 0.99f, 0.995f, 0.998f, 0.999f};
    printf("\n alpha  | tau (s) | fused yaw   | error vs truth (60 deg)\n");
    printf("--------+---------+-------------+------------------------\n");
    for (unsigned k = 0; k < sizeof(alphas)/sizeof(alphas[0]); k++) {
        float a = alphas[k];
        float dt = 0.02f, T = 1.5f;
        int n = (int)(T/dt);
        float s = (float)M_PI * WHEEL_BASE_M / 4.0f;
        float wz = (60.0f*(float)M_PI/180.0f)/T;
        float yaw = 0, yaw_odom = 0;
        for (int i = 0; i < n; i++) {
            float dtheta = (s/n - (-s/n)) / WHEEL_BASE_M;
            yaw_odom += dtheta;
            float pred = yaw + wz*dt;
            yaw = pred + (1.0f-a)*(yaw_odom - pred);
        }
        float deg = yaw*180.0f/(float)M_PI;
        printf(" %.3f  |  %5.2f  |   %6.2f    |   %+6.2f\n",
               a, dt*a/(1.0f-a), deg, deg - 60.0f);
    }
}

int main(void) { straight_line(); spin_in_place(); arc_90(); wrap(); slip(); sweep(); return 0; }
