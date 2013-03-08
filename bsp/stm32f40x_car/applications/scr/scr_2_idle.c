#include "scr.h"
#include <rtthread.h>

const unsigned char res_fs_none[]={
0xfe,					/*[******* ]*/	
0x92,					/*[*  *  * ]*/
0x92,					/*[*  *  * ]*/
0xfe,					/*[******* ]*/
0x92,					/*[*  *  * ]*/
0x92,					/*[*  *  * ]*/
0xfe,					/*[******* ]*/
0x00,					/*[        ]*/
};



const unsigned char res_fs_udisk[]={
0xfe,					/*[******* ]*/	
0xf2,					/*[****  * ]*/
0xf2,					/*[****  * ]*/
0xfe,					/*[******* ]*/
0x92,					/*[*  *  * ]*/
0x92,					/*[*  *  * ]*/
0xfe,					/*[******* ]*/
0x00,					/*[        ]*/
};


const unsigned char res_fs_sd[]={
0xfe,					/*[******* ]*/	
0x92,					/*[*  *  * ]*/
0x92,					/*[*  *  * ]*/
0xfe,					/*[******* ]*/
0x9e,					/*[*  **** ]*/
0x9e,					/*[*  **** ]*/
0xfe,					/*[******* ]*/
0x00,					/*[        ]*/
};

const unsigned char res_fs_udisk_sd[]={
0xfe,					/*[******* ]*/	
0xf2,					/*[****  * ]*/
0xf2,					/*[****  * ]*/
0xfe,					/*[******* ]*/
0x9e,					/*[*  **** ]*/
0x9e,					/*[*  **** ]*/
0xfe,					/*[******* ]*/
0x00,					/*[        ]*/
};



const unsigned char res_print[]={
0x38,					/*[  ***   ]*/	
0x00,					/*[        ]*/
0x38,					/*[  ***   ]*/
0x00,					/*[        ]*/
0x7c,					/*[ *****  ]*/
0x38,					/*[  ***   ]*/
0x10,					/*[   *    ]*/
0x00,					/*[        ]*/
};


const unsigned char res_iccard[]={
0xfe,					/*[******* ]*/	
0x82,					/*[*     * ]*/
0xf2,					/*[****  * ]*/
0x92,					/*[*  *  * ]*/
0xf2,					/*[****  * ]*/
0x82,					/*[*     * ]*/
0xfe,					/*[******* ]*/
0x00,					/*[        ]*/
};

const unsigned char res_sms[]={
0xff,					/*[********]*/	
0xc3,					/*[**    **]*/
0xa5,					/*[* *  * *]*/
0x99,					/*[*  **  *]*/
0x81,					/*[*      *]*/
0xff,					/*[********]*/
0x00,					/*[        ]*/
0x00,					/*[        ]*/
};



const unsigned char res_sat[]={
0x00,0x00,				/*[                ]*/	
0x33,0x00,				/*[  **  **        ]*/
0x1e,0x00,				/*[   ****         ]*/
0x7f,0x80,				/*[ ********       ]*/
0x1e,0x00,				/*[   ****         ]*/
0x33,0x00,				/*[  **  **        ]*/
0x00,0x00,				/*[                ]*/
0x00,0x00,				/*[                ]*/
};

const unsigned char res_antena[]={
                  /* 8421842184218421 */
0xff,0x82,				/*[*********     * ]*/	
0x49,0x02,				/*[ *  *  *      * ]*/
0x2a,0x0a,				/*[  * * *     * * ]*/
0x1c,0x0a,				/*[   ***      * * ]*/
0x08,0x2a,				/*[    *     * * * ]*/
0x08,0xaa,				/*[    *   * * * * ]*/
0x08,0xaa,				/*[    *   * * * * ]*/
0x00,0x00,				/*[                ]*/
};

const unsigned char res_antena_err[]={
                        /* 8421842184218421 */
0xff,0x80,				/*[*********       ]*/	
0x49,0x00,				/*[ *  *  *        ]*/
0x2a,0x00,				/*[  * * *         ]*/
0x1c,0x00,				/*[   ***          ]*/
0x08,0x22,				/*[    *     *   * ]*/
0x03,0x08,				/*[    *       *   ]*/
0x08,0x22,				/*[    *     *   * ]*/
0x00,0x00,				/*[                ]*/
};



const unsigned char res_gsm[]={
                        /* 8421842184218421 */
0x3c,0x02,				/*[  ****        * ]*/	
0x66,0x02,				/*[ **  **       * ]*/
0xc0,0x0a,				/*[**          * * ]*/
0xc0,0x0a,				/*[**          * * ]*/
0xce,0x2a,				/*[**  ***   * * * ]*/
0x66,0xaa,				/*[ **  ** * * * * ]*/
0x3e,0xaa,				/*[  ***** * * * * ]*/
0x00,0x00,				/*[                ]*/
};


const unsigned char res_cdma[]={
                        /* 8421842184218421 */
0x3c,0x02,				/*[  ****        * ]*/	
0x66,0x02,				/*[ **  **       * ]*/
0xc0,0x0a,				/*[**          * * ]*/
0xc0,0x0a,				/*[**          * * ]*/
0xc0,0x2a,				/*[**        * * * ]*/
0x66,0xaa,				/*[ **  ** * * * * ]*/
0x3c,0xaa,				/*[  ****  * * * * ]*/
0x00,0x00,				/*[                ]*/
};




const unsigned char res_bat[]={
                  /* 8421842184218421 */
0x3f,0xff,				/*[  **************]*/	
0x36,0xdb,				/*[  ** ** ** ** **]*/
0xf6,0xdb,				/*[**** ** ** ** **]*/
0xf6,0xdb,				/*[**** ** ** ** **]*/
0xf6,0xdb,				/*[**** ** ** ** **]*/
0x36,0xdb,				/*[  ** ** ** ** **]*/
0x3f,0xff,				/*[  **************]*/
0x00,0x00,				/*[                ]*/
};



const unsigned char res_power_ext[]={
            /* 84218421*/
0x00,				/*[        ]*/	
0x18,				/*[   **   ]*/
0x7f,				/*[ *******]*/
0xf8,				/*[*****   ]*/
0x7f,				/*[ *******]*/
0x18,				/*[   **   ]*/
0x00,				/*[        ]*/
0x00,				/*[        ]*/
};

const unsigned char res_kmh[]={
0x00,0x00,0x00,0x00,0x00,0xE0,0x00,0x00,0xDC,0x00,0x60,0x00,0x00,0xCC,0x00,0x60,0x00,0x01,0x8C,0x00,0x60,0x00,0x01,0x8C,0x00,0x6F,0x7F,0x83,0x0F,0xC0,0x6C,0x36,
0xC3,0x0E,0x60,0x78,0x36,0xC6,0x0C,0x60,0x7C,0x36,0xC6,0x0C,0x60,0x6C,0x36,0xCC,0x0C,0x60,0x66,0x36,0xCC,0x0C,0x60,0xE7,0x76,0xD8,0x1E,0xF0};


const unsigned char res_compass_0[]={
0x00,0x00,0xF8,0x00,0x00,0x00,0x1F,0x27,0xC0,0x00,0x00,0x60,0x20,0x30,0x00,0x01,0x80,0x50,0x0C,0x00,0x02,0x00,0x50,0x02,0x00,0x04,0x00,0x88,0x01,0x00,0x08,0x00,
0x88,0x00,0x80,0x10,0x01,0x04,0x00,0x40,0x20,0x01,0x04,0x00,0x20,0x20,0x02,0x02,0x00,0x20,0x40,0x02,0x02,0x00,0x10,0x40,0x04,0x01,0x00,0x10,0x40,0x04,0x01,0x00,
0x10,0x80,0x08,0x00,0x80,0x08,0x80,0x08,0x00,0x80,0x08,0x80,0x10,0x00,0x40,0x08,0x80,0x10,0x20,0x40,0x08,0x80,0x20,0xD8,0x20,0x08,0x40,0x23,0x06,0x20,0x10,0x40,
0x4C,0x01,0x90,0x10,0x40,0x70,0x00,0x70,0x10,0x20,0xC0,0x00,0x18,0x20,0x20,0x00,0x00,0x00,0x20,0x10,0x00,0x00,0x00,0x40,0x08,0x00,0x00,0x00,0x80,0x04,0x00,0x00,
0x01,0x00,0x02,0x00,0x00,0x02,0x00,0x01,0x80,0x00,0x0C,0x00,0x00,0x60,0x00,0x30,0x00,0x00,0x1F,0x07,0xC0,0x00,0x00,0x00,0xF8,0x00,0x00};





DECL_BMP(7,7,res_fs_none);
DECL_BMP(7,7,res_fs_udisk);
DECL_BMP(7,7,res_fs_sd);
DECL_BMP(7,7,res_fs_udisk_sd);

DECL_BMP(7,7,res_print);
DECL_BMP(7,7,res_iccard);
DECL_BMP(8,7,res_sms);

DECL_BMP(10,7,res_sat);
DECL_BMP(16,7,res_antena);
DECL_BMP(16,7,res_antena_err);

DECL_BMP(16,7,res_gsm);
DECL_BMP(16,7,res_cdma);

DECL_BMP(16,7,res_bat);
DECL_BMP(8,7,res_power_ext);

DECL_BMP(36,12,res_kmh);


DECL_BMP(37,31,res_compass_0);


/*需要获取一些系统状态*/
static void show(void *thiz,unsigned int param)
{
	IMG_DEF tmp;

	lcd_bitmap(0,0,&BMP_res_compass_0,LCD_MODE_SET);
	lcd_bitmap(44,0,&BMP_res_fs_none,LCD_MODE_SET);
	lcd_bitmap(52,0,&BMP_res_iccard,LCD_MODE_SET);
	lcd_bitmap(60,0,&BMP_res_sat,LCD_MODE_INVERT);
	lcd_asc0608(72,0,"7",1,LCD_MODE_SET);
	
	//lcd_text12(80,12,"0km/h",5,LCD_MODE_SET);
	lcd_bitmap(85,8,&BMP_res_kmh,LCD_MODE_SET);

	lcd_bitmap(80,0,&BMP_res_antena,LCD_MODE_SET);
	lcd_bitmap(97,0,&BMP_res_bat,LCD_MODE_SET);
	lcd_bitmap(114,0,&BMP_res_power_ext,LCD_MODE_SET);

	//lcd_rect(40,10,60,10,LCD_MODE_SET);
	//lcd_fill_rect(40,10,70,18,LCD_MODE_SET);

	lcd_text12(44,20,"CMCC    16:02",13,LCD_MODE_SET);
	lcd_update_all( );
}


extern SCR scr_9_export_usb;

/*按键处理*/
static void keypress(void *thiz,unsigned int key)
{
	extern SCR scr_3_main;
	switch(key){
		case KEY_MENU_PRESS:	/*到主菜单界面*/
			scr_3_main.parent=(PSCR)thiz;
			*((PSCR)thiz)=scr_3_main;
			((PSCR)thiz)->show();
			break;
		case KEY_MENU_REPEAT:	/*到数据导出界面*/
			//if(rt_device_find("udisk")==RT_NULL) break;
			scr_9_export_usb.parent=(PSCR)thiz;
			*((PSCR)thiz)=scr_9_export_usb;
			((PSCR)thiz)->show();
			break;
		case KEY_UP_PRESS:		/*发送信息*/
			break;
		case KEY_UP_REPEAT:		/*卫星定位界面*/
			break;
		case KEY_DOWN_PRESS:	/*联系人界面*/
			break;
		case KEY_DOWN_REPEAT:	/*版本信息界面*/
			break;
		case KEY_OK_PRESS:		/*文件管理*/
			break;
		case KEY_OK_REPEAT:		/*打印数据,缺纸?*/
			break;

	}
		
}

/*系统时间*/
static void timetick(void *thiz,unsigned int systick)
{

}

/*处理自检状态的消息*/
static void msg(void *thiz,void *p)
{


}



SCR scr_2_idle=
{
	&show,
	&keypress,
	&timetick,
	(void*)0
};


