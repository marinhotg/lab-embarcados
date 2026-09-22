/* Reta de 2 m e arco de 90 graus, conferidos contra a geometria analitica. */
static void reta(void) {
    odom_pose_t p = {0};
    const float passo = 0.001f;           /* 1 mm por ciclo */
    for (int i = 0; i < 2000; i++) {
        odom_step(&p, passo, passo, 0.0f, 0.02f, false);
    }
    printf("reta 2 m      -> x=%+.4f  y=%+.4f  yaw=%+.2f deg\n",
           p.x_m, p.y_m, p.yaw_rad * 180.0f / (float)M_PI);
}

static void giro_no_lugar(void) {
    odom_pose_t p = {0};
    /* Giro de 90 graus: Dtheta = (dsR - dsL)/b, com dsR = -dsL = s
     * => 2s/b = pi/2  =>  s = pi*b/4 */
    float s_total = (float)M_PI * WHEEL_BASE_M / 4.0f;
    int n = 1000;
    for (int i = 0; i < n; i++) {
        odom_step(&p, -s_total/n, s_total/n, 0.0f, 0.02f, false);
    }
    printf("giro 90 graus -> x=%+.4f  y=%+.4f  yaw=%+.2f deg\n",
           p.x_m, p.y_m, p.yaw_rad * 180.0f / (float)M_PI);
}

static void arco_90(void) {
    /* Arco de 90 graus com raio R=0.5 m: a roda externa anda (R+b/2)*pi/2,
     * a interna (R-b/2)*pi/2. O centro deve terminar em (R, R) com yaw=90. */
    odom_pose_t p = {0};
    const float R = 0.5f;
    float arco_ext = (R + WHEEL_BASE_M/2.0f) * (float)M_PI / 2.0f;
    float arco_int = (R - WHEEL_BASE_M/2.0f) * (float)M_PI / 2.0f;
    int n = 2000;
    for (int i = 0; i < n; i++) {
        odom_step(&p, arco_int/n, arco_ext/n, 0.0f, 0.02f, false);
    }
    printf("arco R=0.5 m  -> x=%+.4f  y=%+.4f  yaw=%+.2f deg  (esperado x=+0.5000 y=+0.5000 yaw=+90.00)\n",
           p.x_m, p.y_m, p.yaw_rad * 180.0f / (float)M_PI);
}

static void wrap(void) {
    /* Gira 350 graus em passos e confere que nao explode na passagem por pi */
    odom_pose_t p = {0};
    float s = (float)M_PI * WHEEL_BASE_M / 4.0f * (350.0f/90.0f);
    int n = 5000;
    for (int i = 0; i < n; i++) { odom_step(&p, -s/n, s/n, 0.0f, 0.02f, false); }
    printf("giro 350 graus-> yaw=%+.2f deg  (esperado -10.00)\n",
           p.yaw_rad * 180.0f / (float)M_PI);
}

static void fusao(void) {
    /* Escorregamento: encoders dizem 90 graus, giroscopio mede 60.
     * Com alpha=0.98 o resultado deve puxar forte para o giroscopio. */
    odom_pose_t p = {0};
    float s = (float)M_PI * WHEEL_BASE_M / 4.0f;
    int n = 1000;
    float dt = 0.02f;
    float wz = (60.0f * (float)M_PI/180.0f) / (n * dt);
    for (int i = 0; i < n; i++) { odom_step(&p, -s/n, s/n, wz, dt, true); }
    printf("escorregamento-> yaw fundido=%+.2f  yaw odometrico=%+.2f deg\n",
           p.yaw_rad * 180.0f/(float)M_PI, p.yaw_odom_rad * 180.0f/(float)M_PI);
}


/* Varredura de alpha: curva de 90 graus em 1,5 s com 30 graus de
   escorregamento (encoders dizem 90, giroscopio mede 60). */
static void sweep(void) {
    float alphas[] = {0.90f, 0.98f, 0.99f, 0.995f, 0.998f, 0.999f};
    printf("\n alpha  | tau (s) | yaw fundido | erro vs verdade (60 deg)\n");
    printf("--------+---------+-------------+-------------------------\n");
    for (unsigned k = 0; k < sizeof(alphas)/sizeof(alphas[0]); k++) {
        float a = alphas[k];
        float dt = 0.02f, T = 1.5f;
        int n = (int)(T/dt);
        float s = (float)M_PI * WHEEL_BASE_M / 4.0f;   /* encoders -> 90 deg */
        float wz = (60.0f*(float)M_PI/180.0f)/T;       /* giroscopio -> 60 deg */
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
int main(void) { reta(); giro_no_lugar(); arco_90(); wrap(); fusao(); sweep(); return 0; }
