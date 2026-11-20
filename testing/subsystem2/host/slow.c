#include <stdlib.h>
#define main sim_main
#include "sim.c"
#undef main
int main(int argc, char **argv)
{
    double stop = atof(argv[1]);
    int req = atoi(argv[2]);
    double sum = 0, sq = 0, n = 0; int t, stopped = 0;
    w_stop[1] = stop;
    sim_reset();
    (void)motion_set_wheel_speeds(100, 100);
    run_ms(1000, NULL);
    (void)motion_set_wheel_speeds(3 * req, req);
    run_ms(600, NULL);
    memset(g_kicks, 0, sizeof(g_kicks));
    for (t = 0; t < 2000; t += 20) { run_ms(20, NULL); sum += w_v[1]; sq += w_v[1]*w_v[1]; n++; if (w_v[1] < 3) stopped++; }
    printf("R stop %.0f pm, asked %d: mean %.1f sd %.1f, kicks %d, stopped periods %d/100\n",
           stop, req, sum/n, sqrt(sq/n-(sum/n)*(sum/n)), g_kicks[1], stopped);
    return 0;
}
