#ifndef _stepping_move_H_
#define _stepping_move_H_

#include "stepping_move.h"
#include <pigpiod_if2.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>

#define black_A1 12 		//black
#define green_A2 13 		//green
#define red_B1 18 		//red
#define blue_B2 19 		//blue
//#define wait_time 0.5

void Stepping::getPi()
{
	pi = pigpio_start(0, 0);
}

void Stepping::decidePin()
{
	set_mode(pi, black_A1, PI_OUTPUT);
	set_mode(pi, green_A2, PI_OUTPUT);
	set_mode(pi, red_B1, PI_OUTPUT);
	set_mode(pi, blue_B2, PI_OUTPUT);
}

void Stepping::rotate360(float wait_time)
{
	int count = 0;
	while(count++ < 2)
	{
		gpio_write(pi, black_A1, 1);
		gpio_write(pi, red_B1, 0);
		gpio_write(pi, green_A2, 0);
		gpio_write(pi, blue_B2, 1);
		usleep(wait_time);

		gpio_write(pi, black_A1, 1);
		gpio_write(pi, red_B1, 1);
		gpio_write(pi, green_A2, 0);
		gpio_write(pi, blue_B2, 0);
		usleep(wait_time);

		gpio_write(pi, black_A1, 0);
		gpio_write(pi, red_B1, 1);
		gpio_write(pi, green_A2, 1);
		gpio_write(pi, blue_B2, 0);
		usleep(wait_time);

		gpio_write(pi, black_A1, 0);
		gpio_write(pi, red_B1, 0);
		gpio_write(pi, green_A2, 1);
		gpio_write(pi, blue_B2, 1);
		usleep(wait_time);
		cout<<count<<endl;
	}

}





#endif
