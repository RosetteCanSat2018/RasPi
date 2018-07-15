#ifndef PID_H
#define PID_H
　
class PID {
public:
    PID();
    virtual ~PID();

    // Initialize
    void Init(double Kp_, double Ki_, double Kd_);

    // 偏差の更新
    void UpdateError(double error);

     // Calculate the total PID error.
    double TotalError();

private:
    // 偏差
    double p_error; // 比例
    double i_error; // 積分
    double d_error; // 微分

    //ゲイン
    double Kp;
    double Ki;
    double Kd;

    bool init_d = false;
    double prev_error;
};

#endif /* PID_H */
