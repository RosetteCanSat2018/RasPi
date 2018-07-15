#include <pigpio.h>
#include <iostream>
#include "PID.h"
#include "Stepper.h"
#include <unistd.h> //for sleep()

using namespace std;

int main()
{
    Stepper StepMotor;
    gpioInitialise();
	StepMotor.SetGPIO(4, 17, 27, 22);

    PID pid;
    pid.Init(0.13,0.0009,3.5); // ゲインを入力する

    double error; // 偏差
    double set_angle = 0; // 目標角度
    double angle; // カルマンフィルタで算出した現在角度
    double steer_value; // PIDの操作量
    int rotate_step; // ステッピングモーターの駆動ステップ数
	int counter = 0;

    while(1){
		if (counter == 360) {
			counter = 0;
		}

		angle = counter;

        // 現在の偏差(deg)を計算
        error = set_angle - angle;

        // 偏差の更新
        pid.UpdateError(error);

        // PIDの操作量(deg)を計算
        steer_value = pid.TotalError();

        // 駆動ステップ数への変換
		rotate_step = int(steer_value / 10);

        // パドル駆動
        if (rotate_step > 0){
            StepMotor.StepCW(rotate_step);
        }else if (rotate_step < 0){
            StepMotor.StepCCW(abs(rotate_step));
        }
		sleep(2);
		counter += 60;
    }
}
