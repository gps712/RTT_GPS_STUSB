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

#include <board.h>
#include <rtthread.h>
#include <finsh.h>

#include "stm32f4xx.h"

#include "jt808_gps.h"

typedef struct _GPSPoint
{
	int sign;
	int deg;
	int min;
	int sec;
} GPSPoint;

uint32_t	jt808_alarm		= 0x0;
uint32_t	jt808_status	= 0x0;


/*
   区域的定义,使用list关联起来，如果node过多的话，
   RAM是否够用
   使用dataflash存储，以4k作为cache,缓存访问
   每秒的位置信息都要判断
 */
struct
{
	uint32_t	id;                 /*区域ID*/
	uint16_t	attr;               /*属性*/
	uint32_t	latitude;           /*中心纬度*/
	uint32_t	logitude;           /*中心经度*/
	uint32_t	radius;             /*半径*/
	uint8_t		datetime_start[6];  /*开始时刻，使用utc是不是更好?*/
	uint8_t		datetime_end[6];
	uint16_t	speed;
	uint8_t		duration;           /*持续时间*/
} circle;

struct
{
	uint32_t	id;                 /*区域ID*/
	uint16_t	attr;               /*属性*/
	uint32_t	latitude;           /*中心纬度*/
	uint32_t	logitude;           /*中心经度*/
	uint32_t	radius;             /*半径*/
	uint8_t		datetime_start[6];  /*开始时刻，使用utc是不是更好?*/
	uint8_t		datetime_end[6];
	uint16_t	speed;
	uint8_t		duration;           /*持续时间*/
} rectangle;

/*保存gps基本位置信息*/
GPS_BASEINFO	gps_baseinfo;
/*gps的状态*/
GPS_STATUS		gps_status;


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
static double gpsToRad( GPSPoint point )
{
	return point.sign * ( point.deg + ( point.min + point.sec / 60.0 ) / 60.0 ) * 3.141592654 / 180.0;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
static double getDistance( GPSPoint latFrom, GPSPoint lngFrom, GPSPoint latTo, GPSPoint lngTo )
{
	double	latFromRad	= gpsToRad( latFrom );
	double	lngFromRad	= gpsToRad( lngFrom );
	double	latToRad	= gpsToRad( latTo );
	double	lngToRad	= gpsToRad( lngTo );
	double	lngDiff		= lngToRad - lngFromRad;
	double	part1		= pow( cos( latToRad ) * sin( lngDiff ), 2 );
	//double part2 = pow( cos(latFromRad)*sin(latToRad)*cos(lngDiff) , 2);
	double	part2 = pow( cos( latFromRad ) * sin( latToRad ) - sin( latFromRad ) * cos( latToRad ) * cos( lngDiff ), 2 );

	double	part3 = sin( latFromRad ) * sin( latToRad ) + cos( latFromRad ) * cos( latToRad ) * cos( lngDiff );
	//double centralAngle = atan2( sqrt(part1 + part2) / part3 );
	double	centralAngle = atan( sqrt( part1 + part2 ) / part3 );
	return 6371.01 * 1000.0 * centralAngle; //Return Distance in meter
}

/*
   $GNRMC,074001.00,A,3905.291037,N,11733.138255,E,0.1,,171212,,,A*655220.9*3F0E
   $GNTXT,01,01,01,ANTENNA OK*2B7,N,11733.138255,E,0.1,,171212,,,A*655220.9*3F0E
   $GNGGA,074002.00,3905.291085,N,11733.138264,E,1,11,0.9,8.2,M,-1.6,M,,,1.4*68E
   $GNGLL,3905.291085,N,11733.138264,E,074002.00,A,0*02.9,8.2,M,-1.6,M,,,1.4*68E
   $GPGSA,A,3,18,05,08,02,26,29,15,,,,,,,,,,,,,,,,,,,,,,,,,,1.6,0.9,1.4,0.9*3F8E
   $BDGSA,A,3,04,03,01,07,,,,,,,,,,,,,,,,,,,,,,,,,,,,,1.6,0.9,1.4,0.9*220.9*3F8E
   $GPGSV,2,1,7,18,10,278,29,05,51,063,08,21,052,24,02,24,140,45*4C220.9*3F8E
   $GPGSV,2,2,7,26,72,055,24,29,35,244,37,15,66,224,37*76,24,140,45*4C220.9*3F8E
   $BDGSV,1,1,4,04,27,124,38,03,42,190,34,01,38,146,37,07,34,173,35*55220.9*3F8E

   返回处理的字段数，如果正确的话
 */
uint8_t process_rmc( uint8_t * pinfo )
{
	//检查数据完整性,执行数据转换
	uint8_t		year = 0, mon = 0, day = 0, hour = 0, min = 0, sec = 0, fDateModify = 0;
	uint32_t	degrees, minutes;
	uint8_t		count;

	uint8_t		gps_time[10];
	uint8_t		gps_av	= 0;
	uint8_t		gps_ns	= 0;
	uint8_t		gps_ew	= 0;
	uint8_t		gps_latitude[16];
	uint8_t		gps_longitude[16];
	uint8_t		gps_speed[8];
	uint8_t		gps_direct[8];
	uint8_t		gps_date[8];

	uint8_t		*psrc = pinfo + 7;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   //指向开始位置
/*时间处理 */
	count = 0;
	while( ( *psrc != ',' ) && ( count < 10 ) )
	{
		gps_time[count++]	= *psrc;
		gps_time[count]		= 0;
		psrc++;
	}
	if( ( count == 0 ) || ( count == 10 ) )
	{
		return 0;
	}
	hour	= ( gps_time[0] - 0x30 ) * 10 + ( gps_time[1] - 0x30 ) + 8;
	min		= ( gps_time[2] - 0x30 ) * 10 + ( gps_time[3] - 0x30 );
	sec		= ( gps_time[4] - 0x30 ) * 10 + ( gps_time[5] - 0x30 );
	if( hour > 23 )
	{
		fDateModify = 1;
		hour		-= 24;
	}
/*A_V处理*/
	psrc++;
	if( ( *psrc == 'A' ) || ( *psrc == 'V' ) )
	{
		gps_av = *psrc;
	} else
	{
		return 1;
	}
	if( gps_av == 'A' )
	{
		jt808_status &= ~0x01;
	} else
	{
		jt808_status |= 0x01;
	}
/*纬度处理ddmm.mmmmmm*/
	psrc++;
	count = 0;
	while( ( *psrc != ',' ) && ( count < 11 ) )
	{
		gps_latitude [count++]	= *psrc;
		gps_latitude [count]	= 0;
		psrc++;
	}
	if( count == 0 )
	{
		return 2;
	}
	degrees = ( ( gps_latitude [0] - 0x30 ) * 10 + ( gps_latitude [1] - 0x30 ) ) * 60 * 100000;
	minutes = ( gps_latitude [2] - 0x30 ) * 1000000 +
	          ( gps_latitude [3] - 0x30 ) * 100000 +
	          ( gps_latitude [5] - 0x30 ) * 10000 +
	          ( gps_latitude [6] - 0x30 ) * 1000 +
	          ( gps_latitude [7] - 0x30 ) * 100 +
	          ( gps_latitude [8] - 0x30 ) * 10 +
	          ( gps_latitude [9] - 0x30 );

/*N_S处理*/
	psrc++;
	if( ( *psrc == 'N' ) || ( *psrc == 'S' ) )
	{
		gps_ns = *psrc;
	} else
	{
		return 3;
	}
	if( gps_ns == 'N' )
	{
		jt808_status &= ~0x02;
	} else
	{
		jt808_status |= 0x02;
	}

/*经度处理*/
	psrc++;
	count = 0;
	while( ( *psrc != ',' ) && ( count < 12 ) )
	{
		gps_longitude [count++] = *psrc;
		gps_longitude [count]	= 0;
		psrc++;
	}
	if( count == 0 )
	{
		return 4;
	}
	degrees = ( ( gps_latitude [0] - 0x30 ) * 100 + ( gps_latitude [1] - 0x30 ) * 10 + ( gps_latitude [2] - 0x30 ) ) * 60 * 100000;
	minutes = ( gps_latitude [3] - 0x30 ) * 1000000 +
	          ( gps_latitude [4] - 0x30 ) * 100000 +
	          ( gps_latitude [6] - 0x30 ) * 10000 +
	          ( gps_latitude [7] - 0x30 ) * 1000 +
	          ( gps_latitude [8] - 0x30 ) * 100 +
	          ( gps_latitude [9] - 0x30 ) * 10 +
	          ( gps_latitude [10] - 0x30 );
/*N_S处理*/
	psrc++;
	if( ( *psrc == 'E' ) || ( *psrc == 'W' ) )
	{
		gps_ew = *psrc;
	} else
	{
		return 5;
	}
	if( gps_ew == 'E' )
	{
		jt808_status &= ~0x04;
	} else
	{
		jt808_status |= 0x04;
	}

/*速度处理*/
	psrc++;
	count = 0;
	while( ( *psrc != ',' ) && ( count < 7 ) )
	{
		gps_speed [count++] = *psrc;
		gps_speed [count]	= 0;
		psrc++;
	}
	if( count == 0 )
	{
		return 6;
	}

/*方向处理*/
	psrc++;
	count = 0;
	while( ( *psrc != ',' ) && ( count < 7 ) )
	{
		gps_direct [count++]	= *psrc;
		gps_direct [count]		= 0;
		psrc++;
	}
	if( count == 0 )
	{
		return 7;
	}

/*日期处理*/
	psrc++;
	count = 0;
	while( ( *psrc != ',' ) && ( count < 7 ) )
	{
		gps_date [count++]	= *psrc;
		gps_date [count]	= 0;
		psrc++;
	}
	if( count == 0 )
	{
		return 8;
	}
	day		= ( ( gps_date [0] - 0x30 ) * 10 ) + ( gps_date [1] - 0x30 );
	mon		= ( ( gps_date [2] - 0x30 ) * 10 ) + ( gps_date [3] - 0x30 );
	year	= ( ( gps_date [4] - 0x30 ) * 10 ) + ( gps_date [5] - 0x30 );

	if( fDateModify )
	{
		day++;
		if( mon == 2 )
		{
			if( ( year % 4 ) == 0 )
			{
				if( day == 30 )
				{
					day = 1; mon++;
				}
			} else
			if( day == 29 )
			{
				day = 1; mon++;
			}
		} else
		if( ( mon == 4 ) || ( mon == 6 ) || ( mon == 9 ) || ( mon == 11 ) )
		{
			if( day == 31 )
			{
				mon++; day = 1;
			}
		} else
		{
			if( day == 32 )
			{
				mon++; day = 1;
			}
			if( mon == 13 )
			{
				mon = 1; year++;
			}
		}
	}
	return 0;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void process_gga( uint8_t * pinfo )
{
}

/***********************************************************
* Function:
* Description:gps收到信息后的处理，头两个字节为长度
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void gps_rx( uint8_t * pinfo, uint16_t length )

{
	uint8_t * psrc;
	psrc				= pinfo;
	*( psrc + length )	= 0;
	if( gps_status.Raw_Output )
	{
		rt_kprintf( "%d gps<%s\r\n", rt_tick_get( ), psrc );
	}
	if( ( strncmp( psrc, "$GNRMC,", 7 ) == 0 ) || ( strncmp( psrc, "$BDRMC,", 7 ) == 0 ) || ( strncmp( psrc, "$GPRMC,", 7 ) == 0 ) )
	{
		process_rmc( psrc );
	}
	if( ( strncmp( psrc, "$GNGGA,", 7 ) == 0 ) || ( strncmp( psrc, "$BDGGA,", 7 ) == 0 ) || ( strncmp( psrc, "$GPGGA,", 7 ) == 0 ) )
	{
		process_gga( psrc );
	}
	/*天线开短路检测 gps<$GNTXT,01,01,01,ANTENNA OK*2B*/
	if( strncmp( psrc + 3, "TXT", 3 ) == 0 )
	{
		if( strstr( psrc + 24, "OK" ) != RT_NULL )
		{
			gps_status.Antenna_Flag = 0;
		}
		if( strstr( psrc + 24, "OPEN" ) != RT_NULL )
		{
			gps_status.Antenna_Flag = 1;
			jt808_alarm|=(1<<5);		/*bit5 天线开路*/
		}
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void gps_dump( uint8_t mode )
{
	gps_status.Raw_Output = mode;
}

FINSH_FUNCTION_EXPORT( gps_dump, dump gps raw info );

/************************************** The End Of File **************************************/
