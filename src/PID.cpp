#include "PID.h"
#include <stdio.h>
#include <math.h>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_init, double Ki_init, double Kd_init) {
    Kp = Kp_init;
    Ki = Ki_init;
    Kd = Kd_init;
    cte_prev = 0.;
    cte_curr = 0.;
    cte_diff = 0.;
    cte_total = 0.;
    cte_total_abs = 0.;

    printf("initialised PID \n");
}

void PID::UpdateError(double cte) {
    cte_prev = cte_curr;
    cte_curr = cte;
    cte_diff = cte_curr - cte_prev;
    TotalError();
}

double PID::TotalError() {
    cte_total += cte_curr;
    cte_total_abs += fabs(cte_curr);
}

double PID::Run() {
    steer_value = -Kp * cte_curr - Kd * cte_diff - Ki * cte_total;
    return steer_value;
}

double PID::Twiddle(){

    //printf("P %f I %f D %f \n", Kp, Ki, Kd);
    //i_twiddle += 1;
    //printf("Total CTE %f at step %i \n", cte_total_abs, i_twiddle);
    i_runs += 1;
    if(i_runs >= 25){
        printf("Total CTE %f ", cte_total_abs);
        //printf("init twiddle bool %i \n", init_twiddle);
        if (init_twiddle == 0){
            //d_Kp = 0.2;
            //d_Ki = 0.004;
            //d_Kd = 3.0;
            //d_Kp = 1.0;
            //d_Ki = 1.0;
            //d_Kd = 1.0;
            cte_best = 1000;
            init_twiddle = 1;
            i_twiddle = 0;
            last_twiddle = 0;
            printf("initialised Twiddle \n");
        }


        if((dp[0] + dp[1] + dp[2]) >= 0.02) {


                if(i_twiddle >= 3) {i_twiddle = 0;};
                printf("updating parameter %i \n", i_twiddle);

                if (cte_total_abs < cte_best and last_twiddle == 0) {
                    printf("choosing 1 \n");
                    cte_best = cte_total_abs;
                    dp[i_twiddle] *= 1.1;
                    last_twiddle = 0;
                }
                else if (last_twiddle == 0){
                    printf("choosing 2 \n");
                    p[i_twiddle] -= 2 * dp[i_twiddle];
                    last_twiddle = 1;
                }
                else if (cte_total_abs < cte_best and last_twiddle == 1) {
                        printf("choosing 3 \n");
                        cte_best = cte_total_abs;
                        dp[i_twiddle] *= 1.1;
                        last_twiddle = 0;
                }
                else if (last_twiddle == 1) {
                        printf("choosing 4 \n");
                        p[i_twiddle] += dp[i_twiddle];
                        dp[i_twiddle] *= 0.9;
                        last_twiddle = 0;
                }
                i_twiddle += 1;
        }
        //p[i_twiddle] += dp[i_twiddle];


        Kp = p[0];
        Ki = p[1];
        Kd = p[2];
        printf("P %f I %f D %f \n", Kp, Ki, Kd);

        Init(Kp, Ki, Kd);

        i_runs = 0;
    }
}


void PID::Print() {
    i_runs += 1;
    if(i_runs >= 150) {
        printf("P %f I %f D %f \n", Kp, Ki, Kd);

        printf("Total CTE %f at step %i \n", cte_total_abs, i_runs);

        i_runs = 0;
    }
}
