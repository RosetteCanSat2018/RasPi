#ifndef PID_H
#define PID_H

class PID {
public:
    PID();
    virtual ~PID();

    // Initialize
    void Init(double Kp_, double Ki_, double Kd_);

    //update error
    void UpdateError(double error);

     // Calculate the total PID error.
    double TotalError();



private:
    // error
    double p_error; // proportion
    double i_error; // integral
    double d_error; // differential

    // gain
    double Kp;
    double Ki;
    double Kd;

    bool init_d = false;
    double prev_error;
};

#endif /* PID_H */
