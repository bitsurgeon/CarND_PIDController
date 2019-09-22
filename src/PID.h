#ifndef PID_H
#define PID_H

class PID {
   public:
    /**
     * Constructor
     */
    PID();

    /**
     * Destructor.
     */
    virtual ~PID();

    /**
     * Initialize PID.
     * @param (Kp_, Ki_, Kd_) The initial PID coefficients
     */
    void Init(double Kp_, double Ki_, double Kd_);

    /**
     * Update the PID error variables given cross track error.
     * @param cte The current cross track error
     */
    void UpdateError(double cte, bool isTwiddle=true);

    /**
     * Calculate the total PID error.
     * @output The total PID error
     */
    double TotalError() const;

   private:
    /**
     * PID Errors
     */
    double p_error;
    double i_error;
    double d_error;

    double cte_pre;  // cte in previous step

    /**
     * PID Coefficients
     */
    double Kp;
    double Ki;
    double Kd;

    // Twiddle coefficients
    double dp[3];
    unsigned dp_idx;
    const double dp_range = 0.2;

    // twiddler stage control
    bool dp_inc;
    bool dp_dec;

    // twiddle adjustment factors
    const double scale_up = 1.1;
    const double scale_dn = 0.9;

    double best_err;
    double acc_err;

    // twiddle coefficients tuning interval
    unsigned step_cnt;
    const unsigned twiddle_steps = 20;

    // turning target
    const double tol = 0.1; // steering
    // const double tol = 0.015; // throttle

    // helper functions

    // core twiddle algorithm
    void doTwiddle();

    // twiddle adjustment
    void twiddler(unsigned idx, double change);

    // calculate current dp sum
    double dp_sum() const;
};

#endif  // PID_H