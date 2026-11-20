#define main sim_main
#include "sim.c"
#undef main
/* steering step 100/100 -> 40/160 and the flip back; report reach time,
   undershoot, and speed noise at steady 100 and at 40 */
static void one(int fromL, int fromR, int toL, int side, const char *name)
{
    int k, reach = -1; double mn = 1e9, mx = -1e9; double to = (side==0)? toL : 0;
    (void)fromL; (void)fromR;
    for (k = 1; k <= 25; k++) {
        run_ms(20, NULL);
        if (reach < 0 && fabs(w_v[side] - to) <= 6.0) reach = k * 20;
        if (w_v[side] < mn) mn = w_v[side];
        if (w_v[side] > mx) mx = w_v[side];
    }
    printf(" %s: reach %4d ms, low %3.0f high %3.0f |", name, reach, mn, mx);
}
int main(void)
{
    double sum=0,sq=0; int k;
    sim_reset();
    (void)motion_set_wheel_speeds(100, 100); run_ms(1500, NULL);
    (void)motion_set_wheel_speeds(40, 160); one(100,100,40,0,"to 40/160");
    (void)motion_set_wheel_speeds(160, 40);
    { int r=-1; double mn=1e9; for (k=1;k<=25;k++){ run_ms(20,NULL); if(r<0&&fabs(w_v[1]-40)<=6) r=k*20; if(w_v[1]<mn) mn=w_v[1]; }
      printf(" flip: reach %4d ms, low %3.0f |", r, mn); }
    (void)motion_set_wheel_speeds(100, 100); run_ms(1000, NULL);
    for (k=0;k<100;k++){ run_ms(20,NULL); sum+=w_duty[0]; sq+=(double)w_duty[0]*w_duty[0]; }
    printf(" duty noise at 100: sd %.1f pm\n", sqrt(sq/100-(sum/100)*(sum/100)));
    return 0;
}
