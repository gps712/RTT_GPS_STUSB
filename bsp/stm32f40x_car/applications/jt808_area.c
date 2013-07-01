/************************************************************
 * Copyright (C), 2008-2012,
 * FileName:		// 文件名
 * Author:			// 作者
 * Date:			// 日期
 * Description:		// 模块描述
 * Version:			// 版本信息
 * Function List:	// 主要函数及其功能
 *     1. -------
 * History:			// 历史修改记录
 *     <author>  <time>   <version >   <desc>
 *     David    96/10/12     1.0     build this moudle
 ***********************************************************/
#include <stdio.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <dfs_posix.h>

#include "stm32f4xx.h"
#include "jt808.h"
#include <finsh.h>
#include "sst25.h"
#include "math.h"
#include "jt808_gps.h"
#include "camera.h"

#define	DF_AreaAddress_Start	0x60000			///电子围栏数据存储开始位置
#define DF_AreaAddress_End		0x61000			///电子围栏数据存储结束位置
#define DF_AreaSaveSect			0x40			///电子围栏数据存储最小间隔


#define	DF_LineAddress_Start	0x61000			///路线数据存储开始位置
#define DF_LineAddress_End		0x75000			///路线数据存储结束位置
#define DF_LINESaveSect			0x1000			///路线数据存储最小间隔


#define AREANUM			24

#define LINENUM			20

#define AREA_BUF_SIZE	4096
#define LINE_BUF_SIZE	4096

#define WLENGH			111319			///每个纬度的距离，该值为固定值单位为米

#define JLENGH			111319			///赤道上每个经度的距离，单位为米



#ifndef BIT
#define BIT(i) ((unsigned long)(1<<i))
#endif


typedef enum
{
	AREA_None=0,				///无数据
	AREA_Circular,				///圆形
	AREA_Rectangle,				///矩形
	AREA_Polygon,				///多边形
	AREA_Line,					///线路
}ENUM_AREA;


typedef __packed struct				///定义了一个坐标
{
	u32				Lati;				///地球坐标纬度
	u32 			Longi;				///地球坐标经度
}TypeStruct_Coor;					


typedef __packed struct				
{
	u32				Inf_ID;				///拐点ID
	u32 			Road_ID;			///路段ID
	TypeStruct_Coor	Inf_coor;			///拐点坐标
	u8				Road_wide;			///路段宽度
	u8				Road_Attr;			///路段属性
}TypeDF_Inflexion;	


typedef __packed struct
{
	u32				ID;					///区域ID
	u16				Atrribute;			///属性
	TypeStruct_Coor	Center_Coor;		///中心点坐标
	u32				Radius;				///半径
	u8				Start_time[6];		///开始时间
	u8				Een_time[6];		///结束时间
	u16				Speed_max;			///最高速度
	u8				Duration;			///持续时间
}TypeDF_AREA_Circular;


typedef __packed struct
{
	u32			ID;					///区域ID
	u16			Atrribute;			///属性
	TypeStruct_Coor	Rect_left;			///区域外围矩形->左上角经纬度
	TypeStruct_Coor	Rect_right;			///区域外围矩形->右下角经纬度
	u8			Start_time[6];		///开始时间
	u8			Een_time[6];		///结束时间
	u16			Speed_max;			///最高速度
	u8			Duration;			///持续时间
}TypeDF_AREA_Rectangle;


typedef __packed struct
{
	u32			ID;					///区域ID
	u16			Atrribute;			///属性
	u8			Start_time[6];		///开始时间
	u8			Een_time[6];		///结束时间
	u16			Speed_max;			///最高速度
	u8			Duration;			///持续时间
	u16			Vertices ;			///顶点数量
	TypeStruct_Coor	Arr_Coor[1];		///多边形的顶点坐标
}TypeDF_AREA_Polygon;


typedef __packed struct
{
	u32			ID;					///区域ID
	u16			Atrribute;			///属性
	u8			Start_time[6];		///开始时间
	u8			Een_time[6];		///结束时间
	u16			Vertices ;			///顶点数量
	TypeStruct_Coor	Arr_Coor[1];		///多边形的顶点坐标
}TypeDF_AREA_Line;


typedef __packed struct
{
	char 		Head[4];			///幻数部分，表示当前数据区域为某固定数据开始
	u16			Len;				///数据长度，包括数据头部分内容
	ENUM_AREA	State;				///区域类型，如果为AREA_None表示无数据
	TypeStruct_Coor	Rect_left;		///区域外围矩形->左上角经纬度
	TypeStruct_Coor	Rect_right;		///区域外围矩形->右下角经纬度
	u32			ID;					///区域ID
	u8			Data[1];			///区域
}TypeDF_AreaHead;

typedef __packed struct
{
	TypeDF_AreaHead *area_data;		///电子围栏数据指针
	u8 			in_area;			///在区域里面的内部标记,0表示不在该区域,1表示在
	u32			in_tick;			///进入该区域的时间，单位为tick
	u32			speed_tick;			///超速的时间，单位为tick
}Type_AreaInfo;

typedef __packed struct
{
	TypeDF_AreaHead line_head;		///电子围栏数据指针
	TypeDF_AreaHead *line_data;		///电子围栏数据指针
	u32			Road_id;			///当前所在的路段ID
	u8 			in_area;			///在区域里面的内部标记0表示不在该区域
	u32			in_tick;			///进入该区域的时间，单位为tick
	u32			speed_tick;			///超时的时间，单位为tick
}Type_LineInfo;

typedef __packed struct
{

 u16				area_len;			///电子围栏数据使用长度
 u16				line_len;			///线路数据使用长度
 u8 				area_num;			///电子围栏数量
 u8 				line_num;			///线路数量
 Type_AreaInfo	 	area_info[AREANUM];	///电子围栏参数及数据
 Type_LineInfo		line_info[LINENUM];	///线路参数及数据
}Type_AreaPara;

static u8	area_data[AREA_BUF_SIZE];    ///存储电子围栏的数据区域
static u8	line_data[LINE_BUF_SIZE];    ///存储线路的数据区域

static const char	AREA_HEAD[]={"AREA"};
Type_AreaPara		Area_Para;


u32 Times_To_LongInt(T_TIMES *T);

/*********************************************************************************
*函数名称:u16 area_flash_read_area( u8 *pdest,u16 maxlen)
*功能描述:读取flash中的电子围栏到全局变量参数 Area_Para中
*输	入:	pdatabuf	:将要保存电子围栏数据的ram区域
		maxlen		:pdatabuf的长度，告诉函数防止溢出
*输	出:	none
*返 回 值:u16	0:发生了溢出	1:正常返回
*作	者:白养民
*创建日期:2013-06-25
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
u16 area_flash_read_area( u8 *pdatabuf,u16 maxlen)
{
 u8 *ptempbuf = pdatabuf;
 u32 TempAddress;
 TypeDF_AreaHead TempAreaHead;
 
 Area_Para.area_len		= 0;
 Area_Para.area_num		= 0;
 memset((void *)(Area_Para.area_info),0,sizeof(Area_Para.area_info));
 
 for(TempAddress=DF_AreaAddress_Start;TempAddress<DF_AreaAddress_End;)
 	{
 	sst25_read(TempAddress,(u8 *)&TempAreaHead,sizeof(TypeDF_AreaHead));
	if((strncmp(TempAreaHead.Head,AREA_HEAD,strlen(AREA_HEAD))==0)&&(TempAreaHead.State))
		{
		///如果buf长度不够或者已经读取到得区域总数量超过AREANUM，则直接返回0，表示溢出
		if((TempAreaHead.Len > maxlen - Area_Para.area_len)||( Area_Para.area_num >= AREANUM ))
			{
			return 0;
			}
		sst25_read(TempAddress,ptempbuf + Area_Para.area_len, TempAreaHead.Len);
		Area_Para.area_info[Area_Para.area_num++].area_data = (TypeDF_AreaHead*)(ptempbuf + Area_Para.area_len);
		Area_Para.area_len += TempAreaHead.Len;
		TempAddress+=(TempAreaHead.Len+DF_AreaSaveSect-1)/DF_AreaSaveSect*DF_AreaSaveSect;
		}
	else
		{
		TempAddress+=DF_AreaSaveSect;
		}
 	}
 return 1;
}


/*********************************************************************************
*函数名称:u16 area_flash_read_line( u8 *pdest,u16 maxlen)
*功能描述:读取flash中的线路参数到全局变量参数 Area_Para中
*输	入:	pdatabuf	:将要保存线路数据的ram区域
		maxlen		:pdatabuf的长度，告诉函数防止溢出
*输	出:	none
*返 回 值:u16	0:发生了溢出	1:正常返回
*作	者:白养民
*创建日期:2013-06-25
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
u16 area_flash_read_line( u8 *pdatabuf,u16 maxlen)
{
 u8 *ptempbuf = pdatabuf;
 u32 TempAddress;
 TypeDF_AreaHead TempAreaHead;
 
 Area_Para.line_len		= 0;
 Area_Para.line_num		= 0;
 memset(&Area_Para.line_info,0,sizeof(Area_Para.line_info));
 
 for(TempAddress=DF_LineAddress_Start;TempAddress<DF_LineAddress_End;)
 	{
 	sst25_read(TempAddress,(u8 *)&TempAreaHead,sizeof(TypeDF_AreaHead));
	if((strncmp(TempAreaHead.Head,AREA_HEAD,strlen(AREA_HEAD))==0)&&(TempAreaHead.State))
		{
		///如果已经读取到得区域总数量超过LINENUM，则直接返回0，表示溢出
		if( Area_Para.area_num >= LINENUM )
			{
			return 0;
			}
		memcpy(&Area_Para.line_info[Area_Para.line_num].line_head,&TempAreaHead,sizeof(TypeDF_AreaHead));
		///如果buf长度不够则不填充数据到数据buf中
		if(TempAreaHead.Len <= maxlen - Area_Para.line_len)
			{
			sst25_read(TempAddress,ptempbuf + Area_Para.line_len, TempAreaHead.Len);
			Area_Para.line_info[Area_Para.line_num].line_data= (TypeDF_AreaHead*)(ptempbuf + Area_Para.line_len);
			Area_Para.line_len += TempAreaHead.Len;
			}
		Area_Para.line_num++;
		}
	TempAddress+=DF_LINESaveSect;
 	}
 return 1;
}


/*********************************************************************************
*函数名称:u16 area_flash_write_area( u8 *pdatabuf,u16 maxlen,TypeDF_AreaHead *pData)
*功能描述:读取flash中的电子围栏到全局变量参数 Area_Para中
*输	入:	pdatabuf	:将要保存电子围栏数据的ram区域
		maxlen		:pdatabuf的长度，告诉函数防止溢出
		pData		:将要存入的电子围栏数据
*输	出:	none
*返 回 值:u16	0:发生了溢出	1:正常返回
*作	者:白养民
*创建日期:2013-06-25
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
u16 area_flash_write_area( u8 *pdatabuf,u16 maxlen,TypeDF_AreaHead *pData)
{
 u8 i;
 u8 *ptempbuf = pdatabuf;
 u32 TempAddress;
 TypeDF_AreaHead TempAreaHead;

 if((pData->State==0)||(pData->State>AREA_Polygon))
 	{
 	return 0;
 	}
 if((pData->Len > maxlen - Area_Para.area_len)||( Area_Para.area_num >= AREANUM ))
 	{
 	return 0;
 	}
 ///处理修改电子围栏区域命令
 for(TempAddress=DF_AreaAddress_Start;TempAddress<DF_AreaAddress_End;)
 	{
 	sst25_read(TempAddress,(u8 *)&TempAreaHead,sizeof(TypeDF_AreaHead));
	if((strncmp(TempAreaHead.Head,AREA_HEAD,strlen(AREA_HEAD))==0)&&(TempAreaHead.State))
		{
		if((pData->Len== TempAreaHead.Len)&&(pData->ID == TempAreaHead.ID))
			{
			sst25_write_back(TempAddress,(u8 *)pData,pData->Len);
			area_flash_read_area(area_data,AREA_BUF_SIZE);
			return 1;
			}
		TempAddress+=(TempAreaHead.Len+DF_AreaSaveSect-1)/DF_AreaSaveSect*DF_AreaSaveSect;
		}
	else
		{
		TempAddress+=DF_AreaSaveSect;
		}
 	}
 ///处理增加电子围栏区域
 memcpy(ptempbuf + Area_Para.area_len,pData,pData->Len);
 Area_Para.area_info[Area_Para.area_num++].area_data = (TypeDF_AreaHead*)(ptempbuf + Area_Para.area_len);
 Area_Para.area_len += pData->Len;

 ///擦除电子围栏区域flash
 for(TempAddress=DF_AreaAddress_Start;TempAddress<DF_AreaAddress_End;TempAddress+=0x1000)
 	{
 	sst25_erase_4k(TempAddress);
 	}
 TempAddress = DF_AreaAddress_Start;
 for( i=0; i < Area_Para.area_num; i++ )
 	{
 	if(TempAddress + Area_Para.area_info[i].area_data->Len >  DF_AreaAddress_End)
 		{
 		return 0;
 		}
 	sst25_write_through(TempAddress,(u8 *)Area_Para.area_info[i].area_data,Area_Para.area_info[i].area_data->Len);
	TempAddress += (Area_Para.area_info[i].area_data->Len+DF_AreaSaveSect-1)/DF_AreaSaveSect*DF_AreaSaveSect;
 	}
 return 1;
}


/*********************************************************************************
*函数名称:u16 area_flash_write_line( u8 *pdatabuf,u16 maxlen,TypeDF_AreaHead *pData)
*功能描述:向flash中存入线路
*输	入:	pdatabuf	:将要保存电子围栏数据的ram区域
		maxlen		:pdatabuf的长度，告诉函数防止溢出
		pData		:将要存入的线路数据
*输	出:	none
*返 回 值:u16	0:发生了溢出	1:正常返回
*作	者:白养民
*创建日期:2013-06-25
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
u16 area_flash_write_line( u8 *pdatabuf,u16 maxlen,TypeDF_AreaHead *pData)
{
 u8 i;
 u8 *ptempbuf = pdatabuf;
 u32 TempAddress;
 TypeDF_AreaHead TempAreaHead;

 if((pData->State==0)||(pData->State != AREA_Line))
 	{
 	return 0;
 	}
 if( Area_Para.line_num >= LINENUM )
 	{
 	return 0;
 	}

 TempAddress = DF_LineAddress_Start;
 for( i = 0; i < LINENUM; i++ )
 	{
 	sst25_read(TempAddress,(u8 *)&TempAreaHead,sizeof(TypeDF_AreaHead));
	if((strncmp(TempAreaHead.Head,AREA_HEAD,strlen(AREA_HEAD)))||(TempAreaHead.State == 0))
		{
		sst25_write_back(TempAddress,(u8 *)pData,pData->Len);
		break;
		}
	
	TempAddress += DF_LINESaveSect;
	}
 
 area_flash_read_line(line_data,LINE_BUF_SIZE);
 return 1;
}


/*********************************************************************************
*函数名称:void area_flash_del_area( u32 del_id , ENUM_AREA	del_State)
*功能描述:删除指定类型的电子围栏
*输	入:	del_id		:将要删除的电子围栏ID，如果该值为0表示删除所有指定类型的电子围栏
		del_State	:将要删除的电子围栏类型
*输	出:	none
*返 回 值:none
*作	者:白养民
*创建日期:2013-06-25
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
void area_flash_del_area( u32 del_id, ENUM_AREA	del_State)
{
 u32 TempAddress;
 TypeDF_AreaHead TempAreaHead;
 
 for(TempAddress=DF_AreaAddress_Start;TempAddress<DF_AreaAddress_End;)
 	{
 	sst25_read(TempAddress,(u8 *)&TempAreaHead,sizeof(TypeDF_AreaHead));
	if((strncmp(TempAreaHead.Head,AREA_HEAD,strlen(AREA_HEAD))==0)&&(TempAreaHead.State))
		{
		if(((TempAreaHead.ID == del_id)||(0 == del_id))&&(del_State == TempAreaHead.State))
			{
			TempAreaHead.State = 0;
			sst25_write_through(TempAddress,(u8 *)&TempAreaHead,sizeof(TypeDF_AreaHead));
			}
		TempAddress+=(TempAreaHead.Len+DF_AreaSaveSect-1)/DF_AreaSaveSect*DF_AreaSaveSect;
		}
	else
		{
		TempAddress+=DF_AreaSaveSect;
		}
 	}
 area_flash_read_area(area_data,AREA_BUF_SIZE);
}

/*********************************************************************************
*函数名称:void area_flash_del_line( u32 del_id )
*功能描述:删除线路
*输	入:	del_id	:将要删除的线路ID，如果该值为0表示删除所有线路
*输	出:	none
*返 回 值:none
*作	者:白养民
*创建日期:2013-06-25
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
void area_flash_del_line( u32 del_id )
{
 u32 TempAddress;
 TypeDF_AreaHead TempAreaHead;
 
 for(TempAddress=DF_LineAddress_Start;TempAddress<DF_LineAddress_End;)
 	{
 	sst25_read(TempAddress,(u8 *)&TempAreaHead,sizeof(TypeDF_AreaHead));
	if((strncmp(TempAreaHead.Head,AREA_HEAD,strlen(AREA_HEAD))==0)&&(TempAreaHead.State))
		{
		if((TempAreaHead.ID == del_id)||(0 == del_id))
			{
			TempAreaHead.State = 0;
			sst25_write_through(TempAddress,(u8 *)&TempAreaHead,sizeof(TypeDF_AreaHead));
			}
		}
	TempAddress+=DF_LINESaveSect;
 	}
 area_flash_read_line(line_data,LINE_BUF_SIZE);
}


/*********************************************************************************
*函数名称:void area_init(void)
*功能描述:电子围栏参数初始化，该函数在终端初始化时调用
*输	入:	none
*输	出:	none
*返 回 值:none
*作	者:白养民
*创建日期:2013-06-25
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
void area_init(void)
{
 //rt_kprintf("enmu len=%d,area_info len=%d \r\n",sizeof(ENUM_AREA),sizeof(area_info));
 memset((void *)&Area_Para,0,sizeof(Area_Para));
 area_flash_read_area(area_data,AREA_BUF_SIZE);
 area_flash_read_line(line_data,LINE_BUF_SIZE);
}



/*********************************************************************************
*函数名称:rt_err_t area_jt808_0x8600(uint16_t fram_num,uint8_t *pmsg,u16 msg_len)
*功能描述:计算指定的纬度上每个经度的长度.
*输	入:	latitude:当前纬度值，单位为百万分之一度
*输	出:none
*返 回 值:double 	:返回实际每个经度的长度，单位为米
*作	者:白养民
*创建日期:2013-06-16
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
double Cal_Longi_Distance(u32 latitude)
{
 double dx,dy;
 double	tempd;
 //u8 tempbuf[128];
 double pi=3.1415926;
 
 tempd = latitude;
 dy = (tempd/1000000/180) *pi ;
 dx = JLENGH * cos(dy);
 //sprintf(tempbuf,"\r\n dx=%f\r\n",dx);
 //rt_kprintf(tempbuf);
 return dx ;
}


/*********************************************************************************
*函数名称:rt_err_t area_jt808_0x8600(uint16_t fram_num,uint8_t *pmsg,u16 msg_len)
*功能描述:计算指定的纬度上每个经度的长度，然后根据用户输入的距离计算对应的经度数，单位为
			以度为单位的经度值乘以 10 的 6 次方，精确到百万分之一度
*输	入:	latitude:当前纬度值，单位为百万分之一度
		distance:用户输入的距离
*输	出:none
*返 回 值:u32 	:返回实际的经度数
*作	者:白养民
*创建日期:2013-06-16
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
u32 dis_to_Longi(u32 latitude,u32 distance)
{
 u32	tempu32data;
 double	tempd;
 //u8 tempbuf[128];
 

 tempd = distance;
 tempu32data = tempd*1000000/Cal_Longi_Distance(latitude);
 tempu32data ++;
 //sprintf(tempbuf,"\r\n longitude=%d\r\n",tempu32data);
 //rt_kprintf(tempbuf);
 return tempu32data ;
}


/*********************************************************************************
*函数名称:u32 dis_Point2Point(u32 Lati_1,u32 Longi_1,u32 Lati_2,u32 Longi_2)
*功能描述:计算两点之间的距离，
*输	入:	Lati_1	:第一个点的纬度，单位为百万分之一度
		Longi_1	:第一个点的经度，单位为百万分之一度
		Lati_2	:第二个点的纬度，单位为百万分之一度
		Longi_2	:第二个点的经度，单位为百万分之一度
*输	出:none
*返 回 值:u32 	:返回实际的距离，单位为米
*作	者:白养民
*创建日期:2013-06-16
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
u32 dis_Point2Point(u32 Lati_1,u32 Longi_1,u32 Lati_2,u32 Longi_2)
{
 u32	tempu32data;
 double	tempd1,tempd2;
 
 tempu32data	= Lati_1 + Lati_2;
 tempd1 = Cal_Longi_Distance( tempu32data/2 );
 tempd1 *= abs( Longi_1 - Longi_2 );
 tempd2 = abs( Lati_1 - Lati_2 ) * WLENGH;
 
 tempd2 *= tempd2;
 tempd1 *= tempd1;
 
 tempu32data = sqrt( tempd1 + tempd2);
 return tempu32data ;
}



/*********************************************************************************
*函数名称:s32 dis_Point2Line(u32 Cur_Lati, u32 Cur_Longi,u32 Lati_1,u32 Longi_1,u32 Lati_2,u32 Longi_2)
*功能描述:计算点到线之间的距离，
*输	入:	Cur_Lati	:当前点的纬度，单位为百万分之一度
		Cur_Longi	:当前点的经度，单位为百万分之一度
		Lati_1		:第一个点的纬度，单位为百万分之一度
		Longti_1	:第一个点的经度，单位为百万分之一度
		Lati_2		:第二个点的纬度，单位为百万分之一度
		Longti_2	:第二个点的经度，单位为百万分之一度
*输	出:none
*返 回 值:u32 	:返回实际的距离，单位为米
*作	者:白养民
*创建日期:2013-06-16
函数数学圆形:	1、当前处理的线条的函数为		y = a*x;
				2、和这个线条垂直的线条的函数为	y1= -(1/a)*x1 + b;
				3、首先计算出:  a,b及1/a，然后根据这三个值计算出两条线条相交的点的X轴坐标
				有公式	a*x	= -(1/a)*x + b;   因此  x2 = b / (a + 1/a),
				4、根据x2坐标计算y2坐标，y = a * x,然后计算这个点到当前点的距离，同时计算这个点
				是否在两点组成的线中间，是则返回正数，否则返回负数
				5、注意，所有的点的计算都以Lati_1和Longi_1作为原点,经度为x轴，纬度为y轴
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
s32 dis_Point2Line(u32 Cur_Lati, u32 Cur_Longi,u32 Lati_1,u32 Longi_1,u32 Lati_2,u32 Longi_2)
{
 double	x,y,x1,y1,x2,y2;
 u32	temp_curlati,temp_curlongi,temp_lati,temp_longi;
 double a,a1,b;
 u32	tempu32data;
 double	tempd1,tempd2;

 y1 	= Cur_Lati - Lati_1;
 x1		= Cur_Longi - Longi_1;
 y		= Lati_2 - Lati_1;
 x		= Longi_2 - Longi_1;

 if(x==0)
 	{
 	 x2	= 0;
	 y2	= y1;
 	}
 else if(y==0)
 	{
 	 x2	= x1;
	 y2	= 0;
 	}
 else
 	{
	 a 	= y / x;
	 a1	= x / y;
	 b	= y1 + a1 * x1;
	 x2	= b / ( a + a1 );
	 y2 = a * x2;
 	}

 temp_lati 	= y2 + Lati_1;
 temp_longi	= x2 + Longi_1;


 if(abs(x2) < abs(x))
 	{
 	return dis_Point2Point(temp_lati,temp_longi,Cur_Lati,Cur_Lati);
 	}
 return 0xFFFFFFFF;
}

/*********************************************************************************
*函数名称:void commit_ack_ok(u16 fram_num,u16 cmd_id,u8 isFalse)
*功能描述:通用应答，单包OK应答
*输	入:	fram_num:应答流水号
		cmd_id	:接收时的808命令
		statue 	:表示状态，0:表示OK	1:表示失败	2:表示消息有误	3:表示不支持
*输	出:	none
*返 回 值:rt_err_t
*作	者:白养民
*创建日期:2013-06-24
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
void commit_ack_ok(u16 fram_num,u16 cmd_id,u8 isFalse)
{
	u8 pbuf[8];
	data_to_buf(pbuf, fram_num, 2);
	data_to_buf(pbuf+2, cmd_id, 2);
	pbuf[4] = isFalse;
	jt808_tx_ack(0x0001,pbuf,2);
}


/*********************************************************************************
*函数名称:rt_err_t area_jt808_del(uint8_t linkno,uint8_t *pmsg, ENUM_AREA  del_State))
*功能描述:808删除圆形区域处理函数
*输	入:	pmsg		:808消息体数据
		msg_len		:808消息体长度
		del_State	:表示要删除的类型
*输	出:none
*返 回 值:rt_err_t
*作	者:白养民
*创建日期:2013-06-16
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
rt_err_t area_jt808_del(uint8_t linkno,uint8_t *pmsg, ENUM_AREA  del_State)
{
 u16 i;
 u32 tempu32data;
 u16 cmd_id;
 u8 *msg;
 u16 msg_len;
 u16 fram_num;

 cmd_id		= buf_to_data( pmsg , 2 );
 msg_len	= buf_to_data( pmsg + 2, 2 ) & 0x3FF;
 fram_num	= buf_to_data( pmsg + 10, 2 );
 pmsg 		+= 12;
 msg		= pmsg;
 if( pmsg[0] == 0)
 	{
 	if(AREA_Line== del_State)
 		{
 		area_flash_del_line( 0 );
 		}
	else
		{
		area_flash_del_area( 0 , del_State );
		}
 	}
 else
 	{
	msg++;
	for(i=0;i<pmsg[0];i++)
	 	{
	 	tempu32data = buf_to_data(msg, 2);
		if(AREA_Line== del_State)
	 		{
	 		area_flash_del_line( tempu32data );
	 		}
		else
			{
			area_flash_del_area( tempu32data , del_State );
			}
		msg += 4;
	 	}
 	}
 commit_ack_ok(fram_num,cmd_id,0);
 return RT_EOK;
}


/*********************************************************************************
*函数名称:rt_err_t area_jt808_0x8600(uint8_t linkno,uint8_t *pmsg)
*功能描述:808设置圆形区域处理函数
*输	入:	pmsg	:808消息体数据
		msg_len	:808消息体长度
*输	出:none
*返 回 值:rt_err_t
*作	者:白养民
*创建日期:2013-06-16
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
rt_err_t area_jt808_0x8600(uint8_t linkno,uint8_t *pmsg)
{
 u16 i;
 u16 datalen;
 u16 tempu16data;
 u32 tempu32data;
 u32 Longi,Lati;		///经度和纬度
 TypeDF_AreaHead *pTempHead;
 u8 tempbuf[64];
 
 u8 *msg;
 u16 msg_len;
 u16 fram_num;
 
 msg_len	= buf_to_data( pmsg + 2, 2 ) & 0x3FF;
 fram_num	= buf_to_data( pmsg + 10, 2 );
 pmsg 		+= 12;
 msg	= pmsg;

 pTempHead = (TypeDF_AreaHead *)tempbuf;
 ///808消息体部分
 msg+=2;	///设置属性 1byte,区域总数 1byte
 
 for(i=0;i<pmsg[1];i++)
 	{
 	datalen = 23;
 	datalen += 18;
 	tempu16data = buf_to_data(msg+4, 2);
	if(tempu16data & BIT(0))
		{
		datalen += 12;
		}
	if(tempu16data & BIT(1))
		{
		datalen += 3;
		}
	memcpy(pTempHead->Head, AREA_HEAD, 4);
	pTempHead->Len	= datalen;
	pTempHead->State= AREA_Circular;

	///计算该形状的外框矩形
	Lati	= buf_to_data(msg+6, 4);
	Longi	= buf_to_data(msg+10, 4);
		///计算圆半径对应的经度数值
	tempu32data=dis_to_Longi(Lati,buf_to_data(msg+14, 4));
	
	pTempHead->Rect_left.Longi	= Longi - tempu32data;
	pTempHead->Rect_right.Longi	= Longi + tempu32data;
		///计算圆半径对应的纬度数值
	tempu32data = buf_to_data(msg+14, 4)*1000000/WLENGH;
	pTempHead->Rect_left.Lati	= Lati + tempu32data;
	pTempHead->Rect_right.Lati	= Lati - tempu32data;
	///复制电子围栏数据到将要写入的结构体中
	memcpy((void *)&pTempHead->ID,msg,datalen);
	area_flash_write_area(area_data,sizeof(area_data),pTempHead);
 	}
 commit_ack_ok(fram_num,0x8600,0);
 return RT_EOK;
}


/*********************************************************************************
*函数名称:rt_err_t area_jt808_0x8601(uint8_t linkno,uint8_t *pmsg)
*功能描述:808删除圆形区域处理函数
*输	入:	pmsg	:808消息体数据
		msg_len	:808消息体长度
*输	出:none
*返 回 值:rt_err_t
*作	者:白养民
*创建日期:2013-06-16
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
rt_err_t area_jt808_0x8601(uint8_t linkno,uint8_t *pmsg)
{
 return area_jt808_del(linkno, pmsg, AREA_Circular);
}


/*********************************************************************************
*函数名称:rt_err_t area_jt808_0x8602(uint8_t linkno,uint8_t *pmsg)
*功能描述:808设置矩形区域处理函数
*输	入:	pmsg	:808消息体数据
		msg_len	:808消息体长度
*输	出:none
*返 回 值:rt_err_t
*作	者:白养民
*创建日期:2013-06-16
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
rt_err_t area_jt808_0x8602(uint8_t linkno,uint8_t *pmsg)
{
 u16 i;
 u16 datalen;
 u16 tempu16data;
 u32 tempu32data;
 u32 Longi,Lati;		///经度和纬度
 TypeDF_AreaHead *pTempHead;
 u8 tempbuf[64];
 
 u8 *msg;
 u16 msg_len;
 u16 fram_num;
 
 msg_len	= buf_to_data( pmsg + 2, 2 ) & 0x3FF;
 fram_num	= buf_to_data( pmsg + 10, 2 );
 pmsg 		+= 12;
 msg	= pmsg;

 pTempHead = (TypeDF_AreaHead *)tempbuf;
 ///808消息体部分
 msg+=2;	///设置属性 1byte,区域总数 1byte
 
 for(i=0;i<pmsg[1];i++)
 	{
 	datalen = 23;
 	datalen += 22;
 	tempu16data = buf_to_data(msg+4, 2);
	if(tempu16data & BIT(0))
		{
		datalen += 12;
		}
	if(tempu16data & BIT(1))
		{
		datalen += 3;
		}
	memcpy(pTempHead->Head, AREA_HEAD, 4);
	pTempHead->Len	= datalen;
	pTempHead->State= AREA_Rectangle;

	///计算该形状的外框矩形
	pTempHead->Rect_left.Longi	= buf_to_data(msg+10, 4);
	pTempHead->Rect_right.Longi	= buf_to_data(msg+18, 4);
	pTempHead->Rect_left.Lati	= buf_to_data(msg+6, 4);
	pTempHead->Rect_right.Lati	= buf_to_data(msg+14, 4);
	
	///复制电子围栏数据到将要写入的结构体中
	memcpy((void *)&pTempHead->ID,msg,datalen);
	area_flash_write_area(area_data,sizeof(area_data),pTempHead);
 	}
 commit_ack_ok(fram_num,0x8602,0);
 return RT_EOK;
}


/*********************************************************************************
*函数名称:rt_err_t area_jt808_0x8603(uint8_t linkno,uint8_t *pmsg)
*功能描述:808删除矩形区域处理函数
*输	入:	pmsg	:808消息体数据
		msg_len	:808消息体长度
*输	出:none
*返 回 值:rt_err_t
*作	者:白养民
*创建日期:2013-06-16
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
rt_err_t area_jt808_0x8603(uint8_t linkno,uint8_t *pmsg)
{
 return area_jt808_del(linkno, pmsg, AREA_Rectangle);
}


/*********************************************************************************
*函数名称:rt_err_t area_jt808_0x8604(uint8_t linkno,uint8_t *pmsg)
*功能描述:808设置多边形区域处理函数
*输	入:	pmsg	:808消息体数据
		msg_len	:808消息体长度
*输	出:none
*返 回 值:rt_err_t
*作	者:白养民
*创建日期:2013-06-16
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
rt_err_t area_jt808_0x8604(uint8_t linkno,uint8_t *pmsg)
{
 u16 i;
 u16 datalen;
 u16 tempu16data;
 u32 tempu32data;
 u32 Longi,Lati;		///经度和纬度
 TypeDF_AreaHead *pTempHead = NULL;
 u8 tempbuf[64];
 
 u8 *msg;
 u16 msg_len;
 u16 fram_num;
 
 msg_len	= buf_to_data( pmsg + 2, 2 ) & 0x3FF;
 fram_num	= buf_to_data( pmsg + 10, 2 );
 pmsg 		+= 12;
 msg	= pmsg;

 ///808消息体部分
 
 datalen = 23;
 datalen += 6;
 tempu16data = buf_to_data(msg+4, 2);
 if(tempu16data & BIT(0))
	 {
	 datalen += 12;
	 }
 if(tempu16data & BIT(1))
	 {
	 datalen += 3;
	 }
 ///顶点数量
 tempu16data = buf_to_data(msg+datalen, 2);
 if(tempu16data > 64)			///限定最大顶点数为64
 	tempu16data = 64;
 datalen += 2;
 
 pTempHead = (TypeDF_AreaHead *)rt_malloc( datalen + tempu16data * 8 );
 if( pTempHead )
 	{
 	///计算该形状的外框矩形
	for( i=0; i< tempu16data; i++)
	 	{
	 	Lati	= buf_to_data(msg + datalen + i*8, 4);
	 	Longi	= buf_to_data(msg + datalen + i*8 + 4, 4);
	 	if(i == 0)
	 		{
			pTempHead->Rect_left.Lati	= Lati;
			pTempHead->Rect_right.Lati	= Lati;
			pTempHead->Rect_left.Longi = Longi;
			pTempHead->Rect_right.Longi = Longi;
	 		}
		else
			{
			if(pTempHead->Rect_left.Lati < Lati )
				pTempHead->Rect_left.Lati = Lati;
			if(pTempHead->Rect_right.Lati > Lati )
				pTempHead->Rect_right.Lati = Lati;
			
			if(pTempHead->Rect_left.Longi > Longi )
				pTempHead->Rect_left.Longi = Longi;
			if(pTempHead->Rect_right.Longi < Longi )
				pTempHead->Rect_right.Longi = Longi;
			}
	 	}
	datalen += tempu16data * 8;
	
	memcpy(pTempHead->Head, AREA_HEAD, 4);
	pTempHead->Len	= datalen;
	pTempHead->State= AREA_Polygon;

	///复制电子围栏数据到将要写入的结构体中
	memcpy((void *)&pTempHead->ID,msg,datalen);
	area_flash_write_area(area_data,sizeof(area_data),pTempHead);
	commit_ack_ok(fram_num,0x8604,0);
	rt_free( pTempHead );
 	}
 else
 	{
 	commit_ack_ok(fram_num,0x8604,3);
 	}
 return RT_EOK;
}


/*********************************************************************************
*函数名称:rt_err_t area_jt808_0x8605(uint8_t linkno,uint8_t *pmsg)
*功能描述:808删除多边形区域处理函数
*输	入:	pmsg	:808消息体数据
		msg_len	:808消息体长度
*输	出:none
*返 回 值:rt_err_t
*作	者:白养民
*创建日期:2013-06-16
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
rt_err_t area_jt808_0x8605(uint8_t linkno,uint8_t *pmsg)
{
 return area_jt808_del(linkno, pmsg, AREA_Polygon);
}


/*********************************************************************************
*函数名称:rt_err_t area_jt808_0x8606(uint8_t linkno,uint8_t *pmsg)
*功能描述:808设置线路处理函数
*输	入:	pmsg	:808消息体数据
		msg_len	:808消息体长度
*输	出:none
*返 回 值:rt_err_t
*作	者:白养民
*创建日期:2013-06-16
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
rt_err_t area_jt808_0x8606(uint8_t linkno,uint8_t *pmsg)
{
 u16 i;
 u16 datalen;
 u16 tempu16data;
 u32 tempu32data;
 u32 Longi,Lati;		///经度和纬度
 TypeDF_AreaHead *pTempHead = NULL;
 u8 tempbuf[64];
 
 u8 *msg;
 u16 msg_len;
 u16 fram_num;
 
 msg_len	= buf_to_data( pmsg + 2, 2 ) & 0x3FF;
 fram_num	= buf_to_data( pmsg + 10, 2 );

 ///处理多包的情况，暂时不支持多包
 if((pmsg[2] & 0x20) == 0)
 	{
 	tempu16data	= buf_to_data( pmsg + 14, 2 );
	if( tempu16data )
		{
		updata_commit_ack_err(fram_num);
 		return RT_ERROR;
		}
 	pmsg 		+= 16;
 	}
 else
 	{
	pmsg 		+= 12;
 	}
 msg	= pmsg;

 ///808消息体部分
 datalen = 23;
 datalen += 6;
 tempu16data = buf_to_data(msg+4, 2);
 if(tempu16data & BIT(0))
	 {
	 datalen += 12;
	 }
 ///顶点数量
 tempu16data = buf_to_data(msg+datalen, 2);
 if(tempu16data > 32)			///限定最大顶点数为64
 	tempu16data = 32;
 datalen += 2;
 
 if( tempu16data > (msg_len - datalen) / 25 )
 	{
 	tempu16data = (msg_len - datalen) / 25 ;
 	}
 
 pTempHead = (TypeDF_AreaHead *)rt_malloc( datalen + tempu16data * 25 );
 if( pTempHead )
 	{
 	///计算该形状的外框矩形
	for( i=0; i< tempu16data; i++)
	 	{
	 	Lati	= buf_to_data(msg + datalen + i*25 + 8, 4);
	 	Longi	= buf_to_data(msg + datalen + i*25 + 12, 4);
	 	if(i == 0)
	 		{
			pTempHead->Rect_left.Lati	= Lati;
			pTempHead->Rect_right.Lati	= Lati;
			pTempHead->Rect_left.Longi = Longi;
			pTempHead->Rect_right.Longi = Longi;
	 		}
		else
			{
			if(pTempHead->Rect_left.Lati < Lati )
				pTempHead->Rect_left.Lati = Lati;
			if(pTempHead->Rect_right.Lati > Lati )
				pTempHead->Rect_right.Lati = Lati;
			
			if(pTempHead->Rect_left.Longi > Longi )
				pTempHead->Rect_left.Longi = Longi;
			if(pTempHead->Rect_right.Longi < Longi )
				pTempHead->Rect_right.Longi = Longi;
			}
	 	}
	datalen += tempu16data * 25;
	
	memcpy(pTempHead->Head, AREA_HEAD, 4);
	pTempHead->Len	= datalen;
	pTempHead->State= AREA_Line;

	///复制电子围栏数据到将要写入的结构体中
	memcpy((void *)&pTempHead->ID,msg,datalen);
	area_flash_write_line(line_data,sizeof(line_data),pTempHead);
	commit_ack_ok(fram_num,0x8606,0);
	rt_free( pTempHead );
 	}
 else
 	{
 	commit_ack_ok(fram_num,0x8604,3);
 	}
 return RT_EOK;
}


/*********************************************************************************
*函数名称:rt_err_t area_jt808_0x8607(uint8_t linkno,uint8_t *pmsg)
*功能描述:808删除线路区域处理函数
*输	入:	pmsg	:808消息体数据
		msg_len	:808消息体长度
*输	出:none
*返 回 值:rt_err_t
*作	者:白养民
*创建日期:2013-06-16
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
rt_err_t area_jt808_0x8607(uint8_t linkno,uint8_t *pmsg)
{
 return area_jt808_del(linkno, pmsg, AREA_Line);
}


/*********************************************************************************
*函数名称:u8 Check_CooisInRect(TypeStruct_Coor *pCoo, TypeDF_AreaHead *pHead )
*功能描述:判断当前位置是否在该矩形区域里面
*输	入:	pCoo	:当前位置坐标
		pHead	:当前矩形区域
*输	出:none
*返 回 值:u8	1:表示在该区域，	0:表示不在该区域
*作	者:白养民
*创建日期:2013-07-01
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
u8 Check_CooisInRect(TypeStruct_Coor *pCoo, TypeDF_AreaHead *pHead )
{
 if((pCoo->Lati > pHead->Rect_left.Lati)||(pCoo->Longi < pHead->Rect_left.Longi))
 	{
 	return 0;
 	}
 if((pCoo->Lati < pHead->Rect_right.Lati)||(pCoo->Longi > pHead->Rect_right.Longi))
	{
	return 0;
 	}
 return 1;
}


/*********************************************************************************
*函数名称:u8 Check_CooisInRect(TypeStruct_Coor *pCoo, TypeDF_AreaHead *pHead )
*功能描述:判断当前位置是否在该矩形区域里面
*输	入:	StartTime	:开始时间
		EndTime		:结束时间
*输	出:none
*返 回 值:u8	1:表示在该时间段，	0:表示不在该时间段
*作	者:白养民
*创建日期:2013-07-01
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
u8 Check_Area_Time(u8 * StartTime,u8 *EndTime)
{
 u8 i;
 u8 T0[6];
 u8 T1[6];
 u8 T2[6];
 u32 temp0,temp1,temp2;
 memset(T0 ,0 ,6);
 memset(T1 ,0 ,6);
 memset(T2 ,0 ,6);
 
 for(i=0;i<6;i++)
 	{
 	if( StartTime[i] )
 		{
		break;
 		}
 	}
 memcpy(T0+i, gps_datetime+i, 6-i);
 memcpy(T1+i, StartTime+i, 6-i);
 memcpy(T2+i, EndTime+i, 6-i);
 
 temp0 = Times_To_LongInt((T_TIMES *)T0);
 temp1 = Times_To_LongInt((T_TIMES *)T1);
 temp2 = Times_To_LongInt((T_TIMES *)T2);

 if((temp0 >=  temp1)&&(temp0 <  temp2))
 	{
 	return 1;
 	}
 return 0;
}

void area_enter_area_alarm( Type_AreaInfo *AreaInfo )
{
}


void area_leave_area_alarm( Type_AreaInfo *AreaInfo )
{
}


void area_speed_alarm( Type_AreaInfo *AreaInfo )
{
}

/*********************************************************************************
*函数名称:u8 area_process_circular(TypeStruct_Coor *pCoo, Type_AreaInfo *AreaInfo )
*功能描述:判断当前位置是否在该圆形区域里面
*输	入:	pCoo	:当前位置坐标
		AreaInfo:当前圆形区域
*输	出:none
*返 回 值:u8	1:表示在该区域，	0:表示不在该区域
*作	者:白养民
*创建日期:2013-07-01
*---------------------------------------------------------------------------------
*修 改 人:
*修改日期:
*修改描述:
*********************************************************************************/
u8 area_process_circular(TypeStruct_Coor *pCoo, Type_AreaInfo *AreaInfo )
{
 u16 datalen;
 u16 attri;					///区域属性
 u32 lati,longi;			///区域中心点坐标
 u32 r;						///半径
 u32 d;						///距离
 u32 curspeed;				///当前速度
 u32 speed=0xFFFFFFFF;		///最高速度
 u32 speedtime=0xFFFFFFFF;	///超速时间
 if( Check_CooisInRect( pCoo, AreaInfo->area_data) == 0)
	{
 	goto AREA_CIRCULAR_OUT;
 	}
 datalen = 14;
 attri	= buf_to_data(AreaInfo->area_data->Data[0], 2);
 if(attri & BIT(0))		///根据时间
 	{
 	if(Check_Area_Time(&AreaInfo->area_data->Data[datalen],&AreaInfo->area_data->Data[6+datalen]) == 0)
 		{
 		goto AREA_CIRCULAR_OUT;
 		}
	datalen += 12;
 	}
 if(attri & BIT(1))		///限速
 	{
 	speed = buf_to_data(AreaInfo->area_data->Data[datalen], 2);
 	speedtime = buf_to_data(AreaInfo->area_data->Data[datalen+2], 1);
	speedtime *= RT_TICK_PER_SECOND;
	datalen += 3;
 	}
 curspeed	= gps_speed;
 lati 	= buf_to_data(AreaInfo->area_data->Data[2], 4);
 longi	= buf_to_data(AreaInfo->area_data->Data[6], 4);
 r		= buf_to_data(AreaInfo->area_data->Data[10], 4);
 d		= dis_Point2Point(lati, longi, pCoo->Lati, pCoo->Longi );	///当前点到中心点的距离
 if( d <= r )
 	{
 	AREA_CIRCULAR_IN:				///进区域
 	if( AreaInfo->in_area == 0)
 		{
 		AreaInfo->in_area = 1;
		AreaInfo->in_tick = rt_tick_get();
		
 		}
	if(attri & BIT(1))		///限速
		{
		if( curspeed < speed )
			{
			AreaInfo->speed_tick = rt_tick_get();
			}
		}
	
 	return 1;
 	}
 AREA_CIRCULAR_OUT:				///出区域
 
 return 0;
}


void area_process(void)
{
 u16 i;
 TypeStruct_Coor cur_Coo;
 static u16 cur_area = 0;
 
 if(cur_area <= Area_Para.area_num)
 	cur_area = 0;
 ///获取当前位置
 cur_Coo.Lati = gps_baseinfo.latitude;
 cur_Coo.Longi= gps_baseinfo.longitude;
 
 for(i=0;i<Area_Para.area_num;i++)
 	{
 	if( Area_Para.area_info[cur_area].area_data->State == AREA_Circular )
		{
		}
	else if(Area_Para.area_info[cur_area].area_data->State == AREA_Circular )
		{
		}
	else
		{
		continue;
		}
 	}
}

#if 0

//--------  D点到直线距离计算-------
/*
     P1(x1,y1)   P2(x2,y2)  ,把点P(x1,y2)作为坐标原点，即x1=0，y2=0；

     那么两点P1，P2 确定的直线方程(两点式)为:
             (x-x1)/(x2-x1) =(y-y1)/(y2-y1)                          (1)
             
    注:  标准式直线方程为 AX+BY+C=0;
             那么平面上任意一点P(x0,y0) 到直线的距离表示为
             d=abs(Ax0+By0+C)/sqrt(A^2+B^2)

    其中把方程式(1) 转换成标准式为:
            (y2-y1)x+(x1-x2)y+x1(y1-y2)+y1(x2-x1)=0; 

   由于点(x1,y2)为原点  即x1=0，y2=0；  P1(0,y1) , P2(x2,0)
    所以   A=-y1 ,  B=-x2, C=y1x2
    那么 直线的方程:
                  -y1x-x2y+y1x2=0;  (2)  
 
  =>     d=abs(-y1x0-x2y0+y1x2)/sqrt(y1^2+x2^2)       (3)

         其中 (3)  为最终应用的公式 

        注:  先根据经纬度折合计算出 x0，y0，x1,y1,x2,y2  的数值单位为: 米     
=>  区域判断:
           根据(2) 可以求出  过 P1(0,y1) , P2(x2,0) 点与已知直线垂直的两条直线方程
              P1(0,y1) :      x2x-y1y+y1^2=0  (4)
              P2(x2,0) :      x2x-y1y-x2^2=0  (5)

          如果 y1 >=0      直线(4)    在直线(5)  的上边
          那么 点在线段区域内的判断方法是
                       (4) <=  0    且  (5)  >=0 
       另                
           如果 y1 <=0      直线(5)    在直线(4)  的上边
          那么 点在线段区域内的判断方法是
                       (4) >=  0    且  (5)  <=0 
   //------------------------------------------------------------------------------------------     
     
       纬度没有差值    1纬度  111km
       40度纬度上 1经度为  85.3km   (北京地区)   111km*cos40

       X 轴为 经度(longitude) 差值
       Y 轴为纬度 (latitude)  差值
                
     
   //------------------------------------------------------------------------------------------
*/

u32   Distance_Point2Line(u32 Cur_Lat, u32  Cur_Longi,u32 P1_Lat,u32 P1_Longi,u32 P2_Lat,u32 P2_Longi)
{   //   输入当前点 ，返回点到既有直线的距离
      long  x0=0,y0=0,Line4_Resualt=0,Line5_Resualt=0;  // 单位: 米
      long  y1=0; 
      long  x2=0;	
      long   distance=0;	 
     // long  Rabs=0;
//      long  Rsqrt=0;  
     long  DeltaA1=0,DeltaA2=0,DeltaO1=0,DeltaO2=0; //  DeltaA : Latitude     DeltaO:  Longitude 	   
     // u32   Line4_Resualt2=0,Line5_Resualt2=0; 
      double   fx0=0,fy0=0,fy1=0,fx2=0;	  
      double   FLine4_Resualt2=0,FLine5_Resualt2=0,fRabs=0,fRsqrt=0; 	  

	  // 0.   先粗略的判断 
	       DeltaA1=abs(Cur_Lat-P1_Lat); 
	       DeltaA2=abs(Cur_Lat-P2_Lat); 
		DeltaO1=abs(Cur_Lat-P1_Longi); 
	       DeltaO2=abs(Cur_Lat-P2_Longi); 
	   /* if((DeltaA1>1000000) &&(DeltaA2>1000000))	    
            {  
                rt_kprintf("\r\n  Latitude 差太大\r\n");
                return   ROUTE_DIS_Default; 
	    }	
	     if((DeltaO1>1000000) &&(DeltaO2>1000000))	    
            {  
                rt_kprintf("\r\n  Longitude 差太大\r\n"); 
                return   ROUTE_DIS_Default; 
	    }	  	
       */
	 // 1.  获取  P1(0,y1)   P2(x2,0) ,和P(x0,y0)    P(x1,y2)为原点  即x1=0，y2=0；  P1(0,y1) , P2(x2,0)
	  x2=abs(P2_Longi-P1_Longi); // a/1000000*85300=a 853/10000 m =a x 0.0853
         if(P2_Longi<P1_Longi)
		 	x2=0-x2;
	  fx2=(double)((double)x2/1000);
	 //rt_kprintf("\r\n P2_L=%d,P1_L=%d   delta=%d \r\n",P2_Longi,P1_Longi,(P2_Longi-P1_Longi));
	 // if(P2_Longi
         y1=abs(P2_Lat-P1_Lat); //  a/1000000*111000=a/9.009	除以一百万得到度数 再乘以 111000 米得到实际距离 
         if(P2_Lat<P1_Lat) 
		 	y1=0-y1;
	   fy1=(double)((double)y1/1000);	  
	  //rt_kprintf("\r\n P2_LA=%d,P1_LA=%d   delta=%d \r\n",P2_Lat,P1_Lat,(P2_Lat-P1_Lat));

      //   rt_kprintf("\r\n 已知两点坐标: P1(0,%d)   P2(%d,0) \r\n", y1,x2); 
       //    当前点
	  x0=abs(Cur_Longi-P1_Longi);     
	   if(Cur_Longi<P1_Longi)
	   	x0=0-x0;
	   fx0=(double)((double)x0/1000);  
         //rt_kprintf("\r\n Cur_L=%d,P1_L=%d   delta=%d \r\n",Cur_Longi,P1_Longi,(Cur_Longi-P1_Longi));
 
         y0=abs(Cur_Lat-P2_Lat); //  a/1000000*111000=a/9.009	    
         if(Cur_Lat<P2_Lat)
		   y0=0-y0;         
	 fy0=(double)((double)y0/1000);	 
          // rt_kprintf("\r\n Cur_La=%d,P2_La=%d   delta=%d \r\n",Cur_Lat,P2_Lat,(Cur_Lat-P2_Lat)); 
        //   rt_kprintf("\r\n当前点坐标: P0(%d,%d)    \r\n", x0,y0);  
	  // 2. 判断y1  的大小， 求出过 P1(0,y1)   P2(x2,0) ,和已知直线的方程，并判断
	  //     当前点是否在路段垂直范围内
	  
             //  2.1   将当前点带入， 过 P1(0,y1)   的 直线方程(4)  求出结果
                                  Line4_Resualt=(x2*x0)-(y1*y0)+(y1*y1);
	                            FLine4_Resualt2=fx2*fx0-fy1*fy0+fy1*fy1;
	   //     rt_kprintf("\r\n Line4=x2*x0-y1*y0+y1*y1=(%d)*(%d)-(%d)*(%d)+(%d)*(%d)=%ld     x2*x0=%d    y1*y0=%d   y1*y1=%d  \r\n",x2,x0,y1,y0,y1,y1,Line4_Resualt,x2*x0,y1*y0,y1*y1); 
          //     rt_kprintf("\r\n FLine4=fx2*fx0-fy1*fy0+fy1*fy1=(%f)*(%f)-(%f)*(%f)+(%f)*(%f)=%f      fx2*fx0=%f    fy1*fy0=%f   fy1*fy1=%f  \r\n",fx2,fx0,fy1,fy0,fy1,fy1,FLine4_Resualt2,fx2*fx0,fy1*fy0,fy1*fy1); 
  
	      //   2.2   将当前点带入， 过P2(x2,0) 的 直线方程(5)  求出结果 
                                  Line5_Resualt=(x2*x0)-y1*y0-x2*x2;
		                    FLine5_Resualt2=fx2*fx0-fy1*fy0-fx2*fx2;    
		//rt_kprintf("\r\n Line5=x2*x0-y1*y0-x2*x2=(%d)*(%d)-(%d)*(%d)-(%d)*(%d)=%ld     Se : %ld   \r\n",x2,x0,y1,y0,x2,x2,Line5_Resualt,Line5_Resualt2); 
          //    rt_kprintf("\r\n FLine5=fx2*fx0-fy1*fy0-fx2*fx2=(%f)*(%f)-(%f)*(%f)-(%f)*(%f)=%f      fx2*fx0=%f    fy1*fy0=%f   fx2*fx2=%f  \r\n",fx2,fx0,fy1,fy0,fx2,fx2,FLine5_Resualt2,fx2*fx0,fy1*fy0,fx2*fx2);   
 	      // rt_kprintf("\r\n  Line4_Resualt=%d     Line5_Resualt=%d  \r\n",Line4_Resualt,Line5_Resualt);    

	     if(fy1>=0)      //  直线(4) 在上发
	  	{
               
			//   2.3   判断区域    (4) <=  0    且  (5)  >=0     // 判断条件取反
			     if((FLine4_Resualt2>0) ||(FLine5_Resualt2<0))
				 	 return   ROUTE_DIS_Default;      //  不满足条件返回最大数值
	  	}
	     else
	     	{      //  直线(5)

                   	//   2.4   判断区域     (4) >=  0    且  (5)  <=0     // 判断条件取反 
			     if((FLine4_Resualt2<0) ||(FLine5_Resualt2>0)) 
				 	 return   ROUTE_DIS_Default;      //  不满足条件返回最大数值 

	     	}	

              rt_kprintf("\r\n In judge area \r\n");  
		//rt_kprintf("\r\n   Current== Latitude:   %d     Longitude: %d     Point1== Latitude:   %d     Longitude: %d     Point2== Latitude:   %d     Longitude: %d\r\n",Cur_Lat,Cur_Longi,P1_Lat,P1_Longi,P2_Lat,P2_Longi);   

        //  3. 将差值差算成实际距离
             #if 0
		    x2=x2*0.0853; // a/1000000*85300=a 853/10000 m =a x 0.0853
                 y1=y1/9; //  a/1000000*111000=a/9.009	除以一百万得到度数 再乘以 111000 米得到实际距离 
                 x0=x0*0.0853;     
		   y0=y0/9; //  a/1000000*111000=a/9.009	    
	      #else
		   fx2=fx2*0.0853; // a/1000000*85300=a 853/10000 m =a x 0.0853
                 fy1=fy1/9; //  a/1000000*111000=a/9.009	除以一百万得到度数 再乘以 111000 米得到实际距离 
                 fx0=fx0*0.0853;     
		   fy0=fy0/9; //  a/1000000*111000=a/9.009	      
             #endif

	 //  4. 计算距离 
	       //Rabs=0-y1*x0-x2*y0+y1*x2;  
              // rt_kprintf("\r\n Test -y1*x0=%d -y0*x2=%d  y1*x2=%d   Rabs=%d  \r\n",0-y1*x0,0-y0*x2,0-y1*x2,Rabs);          
	  #if 0
	       Rabs=abs(-y1*x0-x2*y0+y1*x2);   
	       Rsqrt=sqrt(y1*y1+x2*x2); 
	        // distance=abs(-y1*x0-x2*y0-y1*x2)/sqrt(y1*y1+x2*x2); 
	       distance=Rabs/Rsqrt;
	  // rt_kprintf("\r\n Rabs=%d    Rsqrt=%d   d=%d",Rabs,Rsqrt,distance);        
        #else
             	fRabs=abs(-fy1*fx0-fx2*fy0+fy1*fx2);   
	       fRsqrt=sqrt(fy1*fy1+fx2*fx2); 
	        // distance=abs(-y1*x0-x2*y0-y1*x2)/sqrt(y1*y1+x2*x2); 
	       distance=(long) ((fRabs/fRsqrt)*1000);
	       // rt_kprintf("\r\n Rabs=%d    Rsqrt=%d   d=%d",Rabs,Rsqrt,distance);      
	  #endif
	  
	     
     return   distance;
}

#endif


/************************************** The End Of File **************************************/
