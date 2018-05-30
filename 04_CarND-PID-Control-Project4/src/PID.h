#ifndef PID_H
#define PID_H

class PID {
public:
    /*
    * Errors
    */
    double p_error;
    double i_error;
    double d_error;

    /*
    * Coefficients
    */
    double Kp;
    double Ki;
    double Kd;

    /*
    * PID terms
    */
    double cur_cte;
    double prev_cte;
    double int_cte;
    double diff_cte;
    bool cte_initialized;

    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();

    /*
    * Calculate sterring angle
    */
    double run(double cte);
};

#endif /* PID_H */
