/*
 * main.cpp
 *
 *  Created on: Feb 22, 2021
 *      Author: liulei9
 */
/*
 * main.cpp
 *
 *  Created on: Feb 21, 2021
 *      Author: liulei9
 */
#include <system.h>

volatile char *led_adder = (char *) LEDS_BASE;
volatile char *switch_adder = (char *) SWITCHES_BASE;

int main(){
	while(1){
		*(led_adder) = *(switch_adder);
	}
}




