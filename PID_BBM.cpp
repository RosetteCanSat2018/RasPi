#include <pigpio.h>
#include <iostream>
#include "PID.h"
#include "Stepper.h"

using namespace std;

int main()
{
    Stepper StepMotor;
    gpioInitialise();
	StepMotor.SetGPIO(4, 17, 27, 22);

    PID pid;
    pid.Init(0.13,0.0009,3.5); // ゲインを入力する

    double error; // 偏差
    double set_angle; // 目標角度
    double angle; // カルマンフィルタで算出した現在角度
    double steer_value; // PIDの操作量
    int rotate_step; // ステッピングモーターの駆動ステップ数

    while(1){
        // 現在の偏差を計算
        error = set_angle - angle;

        // 偏差の更新
        pid.UpdateError(error);

        // PIDの操作量を計算
        steer_value = pid.TotalError();

        // 駆動ステップ数への何らかの変換
        // rotate_step = steer_value

        // パドル駆動
        if (rotate_step > 0){
            StepMotor.StepCW(rotate_step);
        }else if (rotate_step < 0){
            StepMotor.StepCCW(abs(rotate_step));
        }
    }
}
