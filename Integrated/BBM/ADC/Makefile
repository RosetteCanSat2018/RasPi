# Makefile
test: ADC_test.cpp Servo.cpp RPi_BNO055.cpp PID.cpp
	g++ -Wall -pthread -o test  ADC_test.cpp Servo.cpp RPi_BNO055.cpp PID.cpp Youdan.cpp -lpigpio -lgps -lm -lrt
