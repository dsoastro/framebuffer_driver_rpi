#ifndef __LCD_H
#define __LCD_H

#include <stdint.h>
#include "rpi_gpio.h"


typedef uint16_t u16;
typedef uint8_t u8;

#define LCD_W 320
#define LCD_H 240

#define BIT_BASE 12
#define CS   20
#define RS   21
#define RST  22
#define WR   23
#define RD   24

#define	LCD_CS_SET  GPIO_SET=(1<<CS)
#define	LCD_RS_SET	GPIO_SET=(1<<RS)
#define	LCD_RST_SET	GPIO_SET=(1<<RST)
#define	LCD_WR_SET	GPIO_SET=(1<<WR)
#define	LCD_RD_SET	GPIO_SET=(1<<RD)

#define	LCD_CS_CLR  GPIO_CLR=(1<<CS)
#define	LCD_RS_CLR	GPIO_CLR=(1<<RS)
#define	LCD_RST_CLR	GPIO_CLR=(1<<RST)
#define	LCD_WR_CLR	GPIO_CLR=(1<<WR)
#define	LCD_RD_CLR	GPIO_CLR=(1<<RD)

#define DATAOUT(x) GPIO_SET=(x<<BIT_BASE);GPIO_CLR=(x<<BIT_BASE)^(0xFF<<BIT_BASE)
#define LCD_WR_D16(Data) LCD_RS_SET;LCD_CS_CLR;DATAOUT(Data);LCD_WR_CLR;LCD_WR_SET;DATAOUT((Data)<<8);LCD_WR_CLR;LCD_WR_SET;LCD_CS_SET;

#define WHITE       0xFFFF
#define BLACK      	0x0000
#define BLUE       	0x001F
#define BRED        0XF81F
#define GRED 			 	0XFFE0
#define GBLUE			 	0X07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define YELLOW      0xFFE0
#define BROWN 			0XBC40
#define BRRED 			0XFC07
#define GRAY  			0X8430


void LCD_Init(void);
void LCD_write(u8 VAL);
u16 LCD_read(void);
void LCD_Clear(u16 Color);
void LCD_SetCursor(u16 Xpos, u16 Ypos);
void LCD_DrawPoint(u16 x,u16 y);
u16  LCD_ReadPoint(u16 x,u16 y);
void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd);
u16 LCD_RD_DATA(void);
void LCD_WriteReg(u8 LCD_Reg, u8 LCD_RegValue);
void LCD_WR_REG(u8 data);
void LCD_WR_DATA(u8 data);
void LCD_ReadReg(u16 LCD_Reg,u8 *Rval,int n);
void LCD_WriteRAM_Prepare(void);
void LCD_ReadRAM_Prepare(void);
void Lcd_WriteData_16Bit(u16 Data);
u16 Lcd_ReadData_16Bit(void);
void LCD_direction(u8 direction );
u16 Color_To_565(u8 r, u8 g, u8 b);
u16 LCD_Read_ID(void);
void delay(int milli_seconds);
void LCD_draw_image(char *file);

#endif




