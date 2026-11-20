#define main sim_main
#include "sim.c"
#undef main
int main(void)
{
    double h_want = 0, h_true = 0, sq = 0, worst = 0; int k, n = 0, seg;
    sim_reset();
    (void)motion_set_wheel_speeds(100, 100); run_ms(1500, NULL);
    for (seg = 0; seg < 8; seg++) {
        const int l = (seg % 2) ? 160 : 40, r = (seg % 2) ? 40 : 160;
        (void)motion_set_wheel_speeds(l, r);
        for (k = 0; k < 15; k++) {           /* 300 ms */
            run_ms(20, NULL);
            h_want += (double)(r - l) / TRACK_WIDTH_MM * 0.020;
            h_true += (w_v[1] - w_v[0]) / TRACK_WIDTH_MM * 0.020;
            sq += (h_want - h_true) * (h_want - h_true); n++;
            if (fabs(h_want - h_true) > worst) worst = fabs(h_want - h_true);
        }
    }
    printf("S-bend heading lag: rms %.1f deg, worst %.1f deg\n",
           sqrt(sq / n) * 57.2958, worst * 57.2958);
    return 0;
}
