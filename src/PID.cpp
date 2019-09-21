#include "PID.h"

#include <limits>
#include <iostream>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;

    p_error = i_error = d_error = 0.0;
    cte_pre = 0.0;

    // initial best error set artificially large
    dp[0] = dp_range * Kp;
    dp[1] = dp_range * Ki;
    dp[2] = dp_range * Kd;

    dp_idx = 0;
    dp_inc = dp_dec = false;

    best_err = std::numeric_limits<double>::max();
    acc_err = 0.0;

    step_cnt = 1;
}

void PID::UpdateError(double cte, bool isTwiddle) {
    // PID
    p_error = cte;
    d_error = cte - cte_pre;
    i_error += cte;
    cte_pre = cte;

    // Twiddle
    if (isTwiddle) {
        std::cout << "----------------------------------------------------------------" << std::endl;
        std::cout << "step_cnt: " << step_cnt << std::endl;
        std::cout << "curr error: " << acc_err << std::endl;
        std::cout << "best error: " << best_err << std::endl;
        std::cout << "pid gains: " << Kp << " | " << Ki << " | " << Kd << std::endl;
        std::cout << "twiddles: " << dp[0] << " | " << dp[1] << " | " << dp[2] << std::endl;
        std::cout << "twiddle sum: " << dp_sum() << std::endl;

        if (dp_sum() > tol) {
            acc_err += cte * cte;
            if (step_cnt == 0) {
                doTwiddle();
            }
            (++step_cnt) %= twiddle_steps;
        }
    }
}

double PID::TotalError() const {
    return - Kp * p_error - Kd * d_error - Ki * i_error;
}

void PID::doTwiddle() {
    std::cout << "    doTwiddle: " << dp_idx << std::endl;
    std::cout << "    dp_inc: " << dp_inc << " | dp_dec: " << dp_dec << std::endl;
    if (!dp_inc && !dp_dec) {
        twiddler(dp_idx, dp[dp_idx]);
        dp_inc = true;
    }

    if (dp_inc && !dp_dec) {
        if (acc_err < best_err) {
            best_err = acc_err;
            dp[dp_idx] *= scale_up;
            (++dp_idx) %= 3;
        } else {
            twiddler(dp_idx, - 2 * dp[dp_idx]);
            dp_inc = false;
            dp_dec = true;
        }
    }

    if (!dp_inc && dp_dec) {
        if (acc_err < best_err) {
            best_err = acc_err;
            dp[dp_idx] *= scale_up;
        } else {
            twiddler(dp_idx, dp[dp_idx]);
            dp[dp_idx] *= scale_dn;
        }

        dp_inc = dp_dec = false;

        (++dp_idx) %= 3;
    }

    acc_err = 0.0;
}

void PID::twiddler(unsigned idx, double change) {
    switch (idx) {
        case 0:
            Kp += change;
            break;
        case 1:
            Ki += change;
            break;
        case 2:
            Kd += change;
            break;
        default:
            break;
    }
}

double PID::dp_sum() const {
    return dp[0] + dp[1] + dp[2];
}
