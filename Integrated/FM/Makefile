# Makefile
test: main.cpp Youdan.cpp Servo.cpp RPi_BNO055.cpp Sensor.cpp MU2.cpp PID.cpp 
	g++ -Wall -pthread -o test  main.cpp Youdan.cpp Servo.cpp RPi_BNO055.cpp Sensor.cpp MU2.cpp PID.cpp -lpigpio -lgps -lm -lrt
