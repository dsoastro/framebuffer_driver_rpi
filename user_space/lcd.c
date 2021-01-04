#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include "lcd.h"

void delay(int milli_seconds){
    usleep(1000 * milli_seconds);
}

u16 POINT_COLOR = 0x0000,BACK_COLOR = 0xFFFF;

void LCD_write(u8 VAL)
{
	LCD_CS_CLR;
	DATAOUT(VAL);
	LCD_WR_CLR;
	LCD_WR_SET;
	LCD_CS_SET;
}

void LCD_WR_REG(u8 data)
{
	LCD_RS_CLR;
	LCD_write(data);
}

void LCD_WR_DATA(u8 data)
{
	LCD_RS_SET;
	LCD_write(data);
}

void LCD_WriteReg(u8 LCD_Reg, u8 LCD_RegValue)
{
	LCD_WR_REG(LCD_Reg);
	LCD_WR_DATA(LCD_RegValue);
}

void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(0x2C);
}

void Lcd_WriteData_16Bit(u16 Data)
{
	LCD_RS_SET;
	LCD_CS_CLR;
	DATAOUT((u8)(Data>>8));
	LCD_WR_CLR;
	LCD_WR_SET;
	DATAOUT((u8)Data);
	LCD_WR_CLR;
	LCD_WR_SET;
	LCD_CS_SET;
}

void LCD_DrawPoint(u16 x,u16 y)
{
	LCD_SetCursor(x,y);
	Lcd_WriteData_16Bit(POINT_COLOR);
}

void LCD_Clear(u16 Color)
{
	unsigned int i;
	LCD_SetWindows(0,0,LCD_W-1,LCD_H-1);
	for(i=0;i<LCD_H*LCD_W;i++)
	{
		Lcd_WriteData_16Bit(Color);
	}
}
void LCD_draw_image(char *file){
	int fd = open(file, O_RDWR);
	if(fd < 0){
		perror("Open file");
		exit(1);
	}
	u16 buffer[128];
	LCD_SetWindows(0,0,LCD_W-1,LCD_H-1);
	while(1){
		int nread=read(fd, buffer, 256);
		if(nread == 0 || nread < 0)
			break;
		for(int i=0; i < nread/2; i++){
			Lcd_WriteData_16Bit(buffer[i]);
		}
	}
	close(fd);
}

void LCD_RESET(void)
{
	LCD_RST_CLR;
	delay(100);
	LCD_RST_SET;
	delay(50);
}

void LCD_Init(void)
{
	LCD_RESET();
	LCD_WR_REG(0xCF);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0xC9);
	LCD_WR_DATA(0X30);
	LCD_WR_REG(0xED);
	LCD_WR_DATA(0x64);
	LCD_WR_DATA(0x03);
	LCD_WR_DATA(0X12);
	LCD_WR_DATA(0X81);
	LCD_WR_REG(0xE8);
	LCD_WR_DATA(0x85);
	LCD_WR_DATA(0x10);
	LCD_WR_DATA(0x7A);
	LCD_WR_REG(0xCB);
	LCD_WR_DATA(0x39);
	LCD_WR_DATA(0x2C);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x34);
	LCD_WR_DATA(0x02);
	LCD_WR_REG(0xF7);
	LCD_WR_DATA(0x20);
	LCD_WR_REG(0xEA);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_REG(0xC0);    //Power control
	LCD_WR_DATA(0x1B);   //VRH[5:0]
	LCD_WR_REG(0xC1);    //Power control
	LCD_WR_DATA(0x00);   //SAP[2:0];BT[3:0] 01
	LCD_WR_REG(0xC5);    //VCM control
	LCD_WR_DATA(0x30); 	 //3F
	LCD_WR_DATA(0x30); 	 //3C
	LCD_WR_REG(0xC7);    //VCM control2
	LCD_WR_DATA(0XB7);
	LCD_WR_REG(0x36);    // Memory Access Control
	LCD_WR_DATA(0x08);
	LCD_WR_REG(0x3A);
	LCD_WR_DATA(0x55);
	LCD_WR_REG(0xB1);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x1A);
	LCD_WR_REG(0xB6);    // Display Function Control
	LCD_WR_DATA(0x0A);
	LCD_WR_DATA(0xA2);
	LCD_WR_REG(0xF2);    // 3Gamma Function Disable
	LCD_WR_DATA(0x00);
	LCD_WR_REG(0x26);    //Gamma curve selected
	LCD_WR_DATA(0x01);
	LCD_WR_REG(0xE0);    //Set Gamma
	LCD_WR_DATA(0x0F);
	LCD_WR_DATA(0x2A);
	LCD_WR_DATA(0x28);
	LCD_WR_DATA(0x08);
	LCD_WR_DATA(0x0E);
	LCD_WR_DATA(0x08);
	LCD_WR_DATA(0x54);
	LCD_WR_DATA(0XA9);
	LCD_WR_DATA(0x43);
	LCD_WR_DATA(0x0A);
	LCD_WR_DATA(0x0F);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_REG(0XE1);    //Set Gamma
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x15);
	LCD_WR_DATA(0x17);
	LCD_WR_DATA(0x07);
	LCD_WR_DATA(0x11);
	LCD_WR_DATA(0x06);
	LCD_WR_DATA(0x2B);
	LCD_WR_DATA(0x56);
	LCD_WR_DATA(0x3C);
	LCD_WR_DATA(0x05);
	LCD_WR_DATA(0x10);
	LCD_WR_DATA(0x0F);
	LCD_WR_DATA(0x3F);
	LCD_WR_DATA(0x3F);
	LCD_WR_DATA(0x0F);
	LCD_WR_REG(0x2B);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x01);
	LCD_WR_DATA(0x3f);
	LCD_WR_REG(0x2A);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0xef);
	LCD_WR_REG(0x11); //Exit Sleep
	delay(120);
	LCD_WR_REG(0x29); //display on
	LCD_WriteReg(0x36,(1<<3)|(1<<5)|(1<<6)); //direction
}

void LCD_SetWindows(u16 xStart, u16 yStart,u16 xEnd,u16 yEnd)
{
	LCD_WR_REG(0x2A);
	LCD_WR_DATA(xStart>>8);
	LCD_WR_DATA(0x00FF&xStart);
	LCD_WR_DATA(xEnd>>8);
	LCD_WR_DATA(0x00FF&xEnd);

	LCD_WR_REG(0x2B);
	LCD_WR_DATA(yStart>>8);
	LCD_WR_DATA(0x00FF&yStart);
	LCD_WR_DATA(yEnd>>8);
	LCD_WR_DATA(0x00FF&yEnd);

	LCD_WriteRAM_Prepare();
}

void LCD_SetCursor(u16 Xpos, u16 Ypos)
{
	LCD_SetWindows(Xpos,Ypos,Xpos,Ypos);
}




