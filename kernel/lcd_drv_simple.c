#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <linux/fb.h>
#include <linux/init.h>

u32 *gpio;
static unsigned PORT = 0x3F200000;
static unsigned RANGE =  0x40;

static void inp_gpio(u32 g){
	u32 *addr = gpio+g/10;
	u32 val = readl(addr);
	u32 tmp =  ~(7<<((g%10)*3));
	val &= tmp;
	writel(val,addr);
}
static void out_gpio(u32 g){
	u32 *addr = gpio+g/10;
	u32 val = readl(addr);
	u32 tmp =  (1<<(((g)%10)*3));
	val |= tmp;
	writel(val,addr);
}
static void GPIO_SET(u32 val){
	writel(val,gpio+7);
}
static void GPIO_CLR(u32 val){
	writel(val,gpio+10);
}

#define BIT_BASE 12
#define CS   20
#define RS   21
#define RST  22
#define WR   23
#define RD   24

#define LCD_W 320
#define LCD_H 240

#define	LCD_CS_SET  GPIO_SET((1<<CS))
#define	LCD_RS_SET	GPIO_SET((1<<RS))
#define	LCD_RST_SET	GPIO_SET((1<<RST))
#define	LCD_WR_SET	GPIO_SET((1<<WR))
#define	LCD_RD_SET	GPIO_SET((1<<RD))

#define	LCD_CS_CLR  GPIO_CLR((1<<CS))
#define	LCD_RS_CLR	GPIO_CLR((1<<RS))
#define	LCD_RST_CLR	GPIO_CLR((1<<RST))
#define	LCD_WR_CLR	GPIO_CLR((1<<WR))
#define	LCD_RD_CLR	GPIO_CLR((1<<RD))

#define DATAOUT(x) GPIO_SET((x<<BIT_BASE));GPIO_CLR((x<<BIT_BASE)^(0xFF<<BIT_BASE))
#define LCD_WR_D16(Data) LCD_RS_SET;LCD_CS_CLR;DATAOUT(Data);LCD_WR_CLR;LCD_WR_SET;DATAOUT((Data)<<8);LCD_WR_CLR;LCD_WR_SET;LCD_CS_SET;

#define WHITE       0xFFFF
#define RED         0xF800

void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd);

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

void LCD_Clear(u16 Color)
{
	unsigned int i;
	LCD_SetWindows(0,0,LCD_W-1,LCD_H-1);
	for(i=0;i<LCD_H*LCD_W;i++)
	{
		Lcd_WriteData_16Bit(Color);
	}
}

void LCD_RESET(void)
{
	LCD_RST_CLR;
	msleep(100);
	LCD_RST_SET;
	msleep(50);
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
	msleep(120);
	LCD_WR_REG(0x29); //display on
	LCD_WriteReg(0x36,(1<<3)|(1<<5)|(1<<6)); //direction
	LCD_Clear(RED);
}

void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd)
{
	LCD_WR_REG(0x2A);
	LCD_WR_DATA(xStar>>8);
	LCD_WR_DATA(0x00FF&xStar);
	LCD_WR_DATA(xEnd>>8);
	LCD_WR_DATA(0x00FF&xEnd);

	LCD_WR_REG(0x2B);
	LCD_WR_DATA(yStar>>8);
	LCD_WR_DATA(0x00FF&yStar);
	LCD_WR_DATA(yEnd>>8);
	LCD_WR_DATA(0x00FF&yEnd);

	LCD_WriteRAM_Prepare();
}

void LCD_SetCursor(u16 Xpos, u16 Ypos)
{
	LCD_SetWindows(Xpos,Ypos,Xpos,Ypos);
}

#define W 320
#define H 240
#define VIDEOMEM_SIZE W * H * 2

static struct fb_fix_screeninfo ili9341_fix  = {
		.type        = FB_TYPE_PACKED_PIXELS,
		.visual      = FB_VISUAL_TRUECOLOR,
		.accel       = FB_ACCEL_NONE,
		.line_length = W * 2,
};

static struct fb_var_screeninfo ili9341_var  = {
		.xres        = W,
		.yres        = H,
		.xres_virtual    = W,
		.yres_virtual    = H,
		.width        = W,
		.height        = H,
		.bits_per_pixel = 16,
		.red         = {11, 5, 0},
		.green         = {5, 6, 0},
		.blue         = {0, 5, 0},
		.activate     = FB_ACTIVATE_NOW,
		.vmode     = FB_VMODE_NONINTERLACED,
};
unsigned long pseudo_palette[17];

struct ili9341 {
	struct device *dev;
	struct fb_info *info;
	unsigned char *videomem;
};

static int ili9341_probe(struct platform_device *dev);
static int ili9341_remove(struct platform_device *device);
static void ili9341_update(struct fb_info *info, struct list_head *pagelist);
int ili9341_setup(struct ili9341 *item);
static int ili9341_setcolreg(unsigned regno, unsigned red, unsigned green, unsigned blue, unsigned transp, struct fb_info *info);

static struct fb_ops ili9341_fbops = {
		.owner        = THIS_MODULE,
		.fb_write     = fb_sys_write,
		.fb_fillrect  = sys_fillrect,
		.fb_copyarea  = sys_copyarea,
		.fb_imageblit = sys_imageblit,
		.fb_setcolreg   = ili9341_setcolreg,
};

struct platform_driver ili9341_driver = {
		.probe = ili9341_probe,
		.remove = ili9341_remove,
		.driver = { .name = "my_fb_driver" }
};

static struct fb_deferred_io ili9341_defio = {
		.delay          = HZ / 25,
		.deferred_io    = &ili9341_update,
};
#define CNVT_TOHW(val,width) ((((val)<<(width))+0x7FFF-(val))>>16)

static int ili9341_setcolreg(unsigned regno,
		unsigned red, unsigned green, unsigned blue,
		unsigned transp, struct fb_info *info)
{
	int ret = 1;
	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
				7471 * blue) >> 16;
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;
			u32 value;

			red = CNVT_TOHW(red, info->var.red.length);
			green = CNVT_TOHW(green, info->var.green.length);
			blue = CNVT_TOHW(blue, info->var.blue.length);
			transp = CNVT_TOHW(transp, info->var.transp.length);

			value = (red << info->var.red.offset) |
					(green << info->var.green.offset) |
					(blue << info->var.blue.offset) |
					(transp << info->var.transp.offset);

			pal[regno] = value;
			ret = 0;
		}
		break;
	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		break;
	}
	return ret;
}

static int  ili9341_probe(struct platform_device *dev)
{
	int ret = 0;
	struct ili9341 *item;
	struct fb_info *info;
	unsigned char  *videomemory;
	printk("ili9341_probe\n");

	item = kzalloc(sizeof(struct ili9341), GFP_KERNEL);
	if (!item) {
		printk(KERN_ALERT "unable to kzalloc for ili9341\n");
		ret = -ENOMEM;
		goto out;
	}
	item->dev = &dev->dev;
	dev_set_drvdata(&dev->dev, item);

	info = framebuffer_alloc(0, &dev->dev);
	if (!info) {
		ret = -ENOMEM;
		printk(KERN_ALERT "unable to framebuffer_alloc\n");
		goto out_item;
	}
	item->info = info;

	info->par = item;
	info->dev = &dev->dev;
	info->fbops = &ili9341_fbops;
	info->flags = FBINFO_FLAG_DEFAULT;
	info->fix = ili9341_fix;
	info->var = ili9341_var;
	info->fix.smem_len = VIDEOMEM_SIZE;
	info->pseudo_palette = &pseudo_palette;

	videomemory=vmalloc(info->fix.smem_len);
	if (!videomemory)
	{
		printk(KERN_ALERT "Can not allocate memory for framebuffer\n");
		ret = -ENOMEM;
		goto out_info;
	}
	info->fix.smem_start =(unsigned long)(videomemory);
	info->screen_base = (char __iomem *)info->fix.smem_start;
	item->videomem = videomemory;

	info->fbdefio = &ili9341_defio;
	fb_deferred_io_init(info);

	ret = register_framebuffer(info);
	if (ret < 0) {
		printk(KERN_ALERT "unable to register_frambuffer\n");
		goto out_pages;
	}

	if (ili9341_setup(item)) goto out_pages;
	return ret;

	out_pages:
	kfree(videomemory);
	out_info:
	framebuffer_release(info);
	out_item:
	kfree(item);
	out:
	return ret;
}

int ili9341_setup(struct ili9341 *item)
{
	int i;
	gpio = ioremap(PORT, RANGE);
	if(gpio == NULL){
		printk(KERN_ALERT "ioremap error\n");
		return 1;
	}

	for(i = BIT_BASE; i <= RD; i++){
		inp_gpio(i);
		out_gpio(i);
	}
	GPIO_SET(0xFFF<<12);
	GPIO_SET(1 << RD);
	LCD_Init();
	printk("ili9341_setup\n");
	return 0;
}

static int ili9341_remove(struct platform_device *device)
{
	struct fb_info *info = platform_get_drvdata(device);
	struct ili9341 *item = (struct ili9341 *)info->par;
	if (info) {
		unregister_framebuffer(info);
		framebuffer_release(info);
		kfree(item->videomem);
		kfree(item);
	}
	return 0;
}

static void ili9341_update(struct fb_info *info, struct list_head *pagelist)
{
	struct ili9341 *item = (struct ili9341 *)info->par;
	u16 *videomemory = (u16 *)item->videomem;
	int i;
	LCD_SetWindows(0,0,LCD_W-1,LCD_H-1);	
	for(i = 0; i < LCD_W * LCD_H; i++){
		Lcd_WriteData_16Bit(readw(videomemory));
		videomemory++;
	}
}

static struct platform_device *ili9341_device;
static int __init ili9341_init(void)
{
	int ret = 0;
	printk("ili9341_init\n");

	ret = platform_driver_register(&ili9341_driver);
	if (ret) {
		printk(KERN_ALERT "unable to platform_driver_register\n");

	}
	else{
		ili9341_device = platform_device_alloc("my_fb_driver", 0);
		if (ili9341_device){
			ret = platform_device_add(ili9341_device);
			printk("device added\n");
		}
		else
			ret = -ENOMEM;

		if (ret) {
			platform_device_put(ili9341_device);
			platform_driver_unregister(&ili9341_driver);
		}
	}

	return ret;
}
module_init(ili9341_init);

static void __exit ili9341_exit(void)
{
	platform_device_unregister(ili9341_device);
	platform_driver_unregister(&ili9341_driver);
	printk("ili9341_exit\n");
}

module_exit(ili9341_exit);

MODULE_LICENSE("GPL");

