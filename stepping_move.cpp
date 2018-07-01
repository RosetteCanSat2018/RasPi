#ifndef _stepping_move_H_
#define _stepping_move_H_

#include "stepping_move.h"
#include <pigpiod_if2.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>

int black_A1 = 12 ;		//black
int green_A2 = 13 ;		//green
int red_B1 = 18 ;		//red
int blue_B2 = 19 ;		//blue
int wait_time = 1;

void Stepping::getPi()
{
	pi = pigpio_start(0, 0);
}

void Stepping::decidePin()
{
	gpioSetMode(black_A1, 1);
	gpioSetMode(green_A2, 1);
	gpioSetMode(red_B1, 1);
	gpioSetMode(blue_B2, 1);
}

int count = 0;

void Stepping::ClockwiseRotation(int cycle)
{
	while (count++ < cycle)
	{
		gpioWrite(pi, black_A1, 1);
		gpioWrite(pi, red_B1, 1);
		gpioWrite(pi, green_A2, 0);
		gpioWrite(pi, blue_B2, 0);
		//sleep(wait_time);

		gpioWrite(pi, black_A1, 0);
		gpioWrite(pi, red_B1, 1);
		gpioWrite(pi, green_A2, 1);
		gpioWrite(pi, blue_B2, 0);
		//sleep(wait_time);

		gpioWrite(pi, black_A1, 0);
		gpioWrite(pi, red_B1, 0);
		gpioWrite(pi, green_A2, 1);
		gpioWrite(pi, blue_B2, 1);
		//sleep(wait_time);

		gpioWrite(pi, black_A1, 1);
		gpioWrite(pi, red_B1, 0);
		gpioWrite(pi, green_A2, 0);
		gpioWrite(pi, blue_B2, 1);
		//sleep(wait_time);
	}

}


void Stepping::CounterClockwiseRotation(int cycle)
{
	while(count++ < cycle)
	{
		gpioWrite(pi, black_A1, 1);
		gpioWrite(pi, red_B1, 1);
		gpioWrite(pi, green_A2, 0);
		gpioWrite(pi, blue_B2, 0);
		//sleep(wait_time);

		gpioWrite(pi, black_A1, 1);
		gpioWrite(pi, red_B1, 0);
		gpioWrite(pi, green_A2, 0);
		gpioWrite(pi, blue_B2, 1);
		//sleep(wait_time);

		gpioWrite(pi, black_A1, 0);
		gpioWrite(pi, red_B1, 0);
		gpioWrite(pi, green_A2, 1);
		gpioWrite(pi, blue_B2, 1);
		//sleep(wait_time);

		gpioWrite(pi, black_A1, 0);
		gpioWrite(pi, red_B1, 1);
		gpioWrite(pi, green_A2, 1);
		gpioWrite(pi, blue_B2, 0);
		//sleep(wait_time);
	}

}
void Stepping::ClockwiseTick(int count)
{
	if (count == 1) {
		gpioWrite(pi, black_A1, 1);
		gpioWrite(pi, red_B1, 1);
		gpioWrite(pi, green_A2, 0);
		gpioWrite(pi, blue_B2, 0);
		//sleep(wait_time);
	}
	if (count == 2) {
		gpioWrite(pi, black_A1, 1);
		gpioWrite(pi, red_B1, 1);
		gpioWrite(pi, green_A2, 0);
		gpioWrite(pi, blue_B2, 0);
		//sleep(wait_time);

		gpioWrite(pi, black_A1, 0);
		gpioWrite(pi, red_B1, 1);
		gpioWrite(pi, green_A2, 1);
		gpioWrite(pi, blue_B2, 0);
		//sleep(wait_time);
	}
	if (count == 3) {
		gpioWrite(pi, black_A1, 1);
		gpioWrite(pi, red_B1, 1);
		gpioWrite(pi, green_A2, 0);
		gpioWrite(pi, blue_B2, 0);
		//sleep(wait_time);

		gpioWrite(pi, black_A1, 0);
		gpioWrite(pi, red_B1, 1);
		gpioWrite(pi, green_A2, 1);
		gpioWrite(pi, blue_B2, 0);
		//sleep(wait_time);

		gpioWrite(pi, black_A1, 0);
		gpioWrite(pi, red_B1, 0);
		gpioWrite(pi, green_A2, 1);
		gpioWrite(pi, blue_B2, 1);
		//sleep(wait_time);
	}

}

void Stepping::CounterClockwiseTick(int count)
{
	if (count == 1) {
		gpioWrite(pi, black_A1, 1);
		gpioWrite(pi, red_B1, 1);
		gpioWrite(pi, green_A2, 0);
		gpioWrite(pi, blue_B2, 0);
		//sleep(wait_time);
	}
	if (count == 2) {
		gpioWrite(pi, black_A1, 1);
		gpioWrite(pi, red_B1, 1);
		gpioWrite(pi, green_A2, 0);
		gpioWrite(pi, blue_B2, 0);
		//sleep(wait_time);

		gpioWrite(pi, black_A1, 1);
		gpioWrite(pi, red_B1, 0);
		gpioWrite(pi, green_A2, 0);
		gpioWrite(pi, blue_B2, 1);
		//sleep(wait_time);
	}
	if (count == 3) {
		gpioWrite(pi, black_A1, 1);
		gpioWrite(pi, red_B1, 1);
		gpioWrite(pi, green_A2, 0);
		gpioWrite(pi, blue_B2, 0);
		//sleep(wait_time);

		gpioWrite(pi, black_A1, 1);
		gpioWrite(pi, red_B1, 0);
		gpioWrite(pi, green_A2, 0);
		gpioWrite(pi, blue_B2, 1);
		//sleep(wait_time);

		gpioWrite(pi, black_A1, 0);
		gpioWrite(pi, red_B1, 0);
		gpioWrite(pi, green_A2, 1);
		gpioWrite(pi, blue_B2, 1);
		//sleep(wait_time);
	}

}

void Stepping::control(int rotation) {
	int cycle;
	int count;
	cycle = abs(rotation) / 4;
	count = abs(rotation) - (cycle * 4);
	if (rotation >= 0) {
		ClockwiseRotation(cycle);
		ClockwiseTick(count);
	}
	else if (rotation < 0) {
		CounterClockwiseRotation(cycle);
		CounterClockwiseTick(count);
	}

}

#endif
