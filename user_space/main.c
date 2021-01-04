#include <stdio.h>
#include <unistd.h>
#include "lcd.h"

int main(int argc , char *argv[]){
	if( setup_rpi_gpio() ) {
		printf("Cannot map GPIO memory, probably use <sudo>\n");
		return -1;
	}
	for(int i = BIT_BASE; i <= RD; i++){
		INP_GPIO(i);
		OUT_GPIO(i);
	}
	//set BITS_BASE - RD to 1
	GPIO_SET = 0xFFF<<12;
	GPIO_SET = 1 << RD;
	LCD_Init();

	if(argc >= 2){
		LCD_draw_image(argv[1]);
	}
}
