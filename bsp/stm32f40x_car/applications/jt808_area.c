/************************************************************
 * Copyright (C), 2008-2012,
 * FileName:		// �ļ���
 * Author:			// ����
 * Date:			// ����
 * Description:		// ģ������
 * Version:			// �汾��Ϣ
 * Function List:	// ��Ҫ�������书��
 *     1. -------
 * History:			// ��ʷ�޸ļ�¼
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

#define	DF_AreaAddress_Start	0x60000			///����Χ�����ݴ洢��ʼλ��
#define DF_AreaAddress_End		0x61000			///����Χ�����ݴ洢����λ��
#define DF_AreaSaveSect			0x40			///����Χ�����ݴ洢��С���


#define	DF_LineAddress_Start	0x61000			///·�����ݴ洢��ʼλ��
#define DF_LineAddress_End		0x75000			///·�����ݴ洢����λ��
#define DF_LINESaveSect			0x1000			///·�����ݴ洢��С���


#define AREANUM			24

#define LINENUM			20

#define AREA_BUF_SIZE	4096
#define LINE_BUF_SIZE	4096

#define WLENGH			111319			///ÿ��γ�ȵľ��룬��ֵΪ�̶�ֵ��λΪ��

#define JLENGH			111319			///�����ÿ�����ȵľ��룬��λΪ��



#ifndef BIT
#define BIT(i) ((unsigned long)(1<<i))
#endif


typedef enum
{
	AREA_None=0,				///������
	AREA_Circular,				///Բ��
	AREA_Rectangle,				///����
	AREA_Polygon,				///�����
	AREA_Line,					///��·
}ENUM_AREA;


typedef __packed struct				///������һ������
{
	u32				Lati;				///��������γ��
	u32 			Longi;				///�������꾭��
}TypeStruct_Coor;					


typedef __packed struct				
{
	u32				Inf_ID;				///�յ�ID
	u32 			Road_ID;			///·��ID
	TypeStruct_Coor	Inf_coor;			///�յ�����
	u8				Road_wide;			///·�ο��
	u8				Road_Attr;			///·������
}TypeDF_Inflexion;	


typedef __packed struct
{
	u32				ID;					///����ID
	u16				Atrribute;			///����
	TypeStruct_Coor	Center_Coor;		///���ĵ�����
	u32				Radius;				///�뾶
	u8				Start_time[6];		///��ʼʱ��
	u8				Een_time[6];		///����ʱ��
	u16				Speed_max;			///����ٶ�
	u8				Duration;			///����ʱ��
}TypeDF_AREA_Circular;


typedef __packed struct
{
	u32			ID;					///����ID
	u16			Atrribute;			///����
	TypeStruct_Coor	Rect_left;			///������Χ����->���ϽǾ�γ��
	TypeStruct_Coor	Rect_right;			///������Χ����->���½Ǿ�γ��
	u8			Start_time[6];		///��ʼʱ��
	u8			Een_time[6];		///����ʱ��
	u16			Speed_max;			///����ٶ�
	u8			Duration;			///����ʱ��
}TypeDF_AREA_Rectangle;


typedef __packed struct
{
	u32			ID;					///����ID
	u16			Atrribute;			///����
	u8			Start_time[6];		///��ʼʱ��
	u8			Een_time[6];		///����ʱ��
	u16			Speed_max;			///����ٶ�
	u8			Duration;			///����ʱ��
	u16			Vertices ;			///��������
	TypeStruct_Coor	Arr_Coor[1];		///����εĶ�������
}TypeDF_AREA_Polygon;


typedef __packed struct
{
	u32			ID;					///����ID
	u16			Atrribute;			///����
	u8			Start_time[6];		///��ʼʱ��
	u8			Een_time[6];		///����ʱ��
	u16			Vertices ;			///��������
	TypeStruct_Coor	Arr_Coor[1];		///����εĶ�������
}TypeDF_AREA_Line;


typedef __packed struct
{
	char 		Head[4];			///�������֣���ʾ��ǰ��������Ϊĳ�̶����ݿ�ʼ
	u16			Len;				///���ݳ��ȣ���������ͷ��������
	ENUM_AREA	State;				///�������ͣ����ΪAREA_None��ʾ������
	TypeStruct_Coor	Rect_left;		///������Χ����->���ϽǾ�γ��
	TypeStruct_Coor	Rect_right;		///������Χ����->���½Ǿ�γ��
	u32			ID;					///����ID
	u8			Data[1];			///����
}TypeDF_AreaHead;

typedef __packed struct
{
	TypeDF_AreaHead *area_data;		///����Χ������ָ��
	u8 			in_area;			///������������ڲ����,0��ʾ���ڸ�����,1��ʾ��
	u32			in_tick;			///����������ʱ�䣬��λΪtick
	u32			speed_tick;			///���ٵ�ʱ�䣬��λΪtick
}Type_AreaInfo;

typedef __packed struct
{
	TypeDF_AreaHead line_head;		///����Χ������ָ��
	TypeDF_AreaHead *line_data;		///����Χ������ָ��
	u32			Road_id;			///��ǰ���ڵ�·��ID
	u8 			in_area;			///������������ڲ����0��ʾ���ڸ�����
	u32			in_tick;			///����������ʱ�䣬��λΪtick
	u32			speed_tick;			///��ʱ��ʱ�䣬��λΪtick
}Type_LineInfo;

typedef __packed struct
{

 u16				area_len;			///����Χ������ʹ�ó���
 u16				line_len;			///��·����ʹ�ó���
 u8 				area_num;			///����Χ������
 u8 				line_num;			///��·����
 Type_AreaInfo	 	area_info[AREANUM];	///����Χ������������
 Type_LineInfo		line_info[LINENUM];	///��·����������
}Type_AreaPara;

static u8	area_data[AREA_BUF_SIZE];    ///�洢����Χ������������
static u8	line_data[LINE_BUF_SIZE];    ///�洢��·����������

static const char	AREA_HEAD[]={"AREA"};
Type_AreaPara		Area_Para;


u32 Times_To_LongInt(T_TIMES *T);

/*********************************************************************************
*��������:u16 area_flash_read_area( u8 *pdest,u16 maxlen)
*��������:��ȡflash�еĵ���Χ����ȫ�ֱ������� Area_Para��
*��	��:	pdatabuf	:��Ҫ�������Χ�����ݵ�ram����
		maxlen		:pdatabuf�ĳ��ȣ����ߺ�����ֹ���
*��	��:	none
*�� �� ֵ:u16	0:���������	1:��������
*��	��:������
*��������:2013-06-25
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
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
		///���buf���Ȳ��������Ѿ���ȡ������������������AREANUM����ֱ�ӷ���0����ʾ���
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
*��������:u16 area_flash_read_line( u8 *pdest,u16 maxlen)
*��������:��ȡflash�е���·������ȫ�ֱ������� Area_Para��
*��	��:	pdatabuf	:��Ҫ������·���ݵ�ram����
		maxlen		:pdatabuf�ĳ��ȣ����ߺ�����ֹ���
*��	��:	none
*�� �� ֵ:u16	0:���������	1:��������
*��	��:������
*��������:2013-06-25
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
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
		///����Ѿ���ȡ������������������LINENUM����ֱ�ӷ���0����ʾ���
		if( Area_Para.area_num >= LINENUM )
			{
			return 0;
			}
		memcpy(&Area_Para.line_info[Area_Para.line_num].line_head,&TempAreaHead,sizeof(TypeDF_AreaHead));
		///���buf���Ȳ�����������ݵ�����buf��
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
*��������:u16 area_flash_write_area( u8 *pdatabuf,u16 maxlen,TypeDF_AreaHead *pData)
*��������:��ȡflash�еĵ���Χ����ȫ�ֱ������� Area_Para��
*��	��:	pdatabuf	:��Ҫ�������Χ�����ݵ�ram����
		maxlen		:pdatabuf�ĳ��ȣ����ߺ�����ֹ���
		pData		:��Ҫ����ĵ���Χ������
*��	��:	none
*�� �� ֵ:u16	0:���������	1:��������
*��	��:������
*��������:2013-06-25
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
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
 ///�����޸ĵ���Χ����������
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
 ///�������ӵ���Χ������
 memcpy(ptempbuf + Area_Para.area_len,pData,pData->Len);
 Area_Para.area_info[Area_Para.area_num++].area_data = (TypeDF_AreaHead*)(ptempbuf + Area_Para.area_len);
 Area_Para.area_len += pData->Len;

 ///��������Χ������flash
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
*��������:u16 area_flash_write_line( u8 *pdatabuf,u16 maxlen,TypeDF_AreaHead *pData)
*��������:��flash�д�����·
*��	��:	pdatabuf	:��Ҫ�������Χ�����ݵ�ram����
		maxlen		:pdatabuf�ĳ��ȣ����ߺ�����ֹ���
		pData		:��Ҫ�������·����
*��	��:	none
*�� �� ֵ:u16	0:���������	1:��������
*��	��:������
*��������:2013-06-25
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
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
*��������:void area_flash_del_area( u32 del_id , ENUM_AREA	del_State)
*��������:ɾ��ָ�����͵ĵ���Χ��
*��	��:	del_id		:��Ҫɾ���ĵ���Χ��ID�������ֵΪ0��ʾɾ������ָ�����͵ĵ���Χ��
		del_State	:��Ҫɾ���ĵ���Χ������
*��	��:	none
*�� �� ֵ:none
*��	��:������
*��������:2013-06-25
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
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
*��������:void area_flash_del_line( u32 del_id )
*��������:ɾ����·
*��	��:	del_id	:��Ҫɾ������·ID�������ֵΪ0��ʾɾ��������·
*��	��:	none
*�� �� ֵ:none
*��	��:������
*��������:2013-06-25
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
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
*��������:void area_init(void)
*��������:����Χ��������ʼ�����ú������ն˳�ʼ��ʱ����
*��	��:	none
*��	��:	none
*�� �� ֵ:none
*��	��:������
*��������:2013-06-25
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
*********************************************************************************/
void area_init(void)
{
 //rt_kprintf("enmu len=%d,area_info len=%d \r\n",sizeof(ENUM_AREA),sizeof(area_info));
 memset((void *)&Area_Para,0,sizeof(Area_Para));
 area_flash_read_area(area_data,AREA_BUF_SIZE);
 area_flash_read_line(line_data,LINE_BUF_SIZE);
}



/*********************************************************************************
*��������:rt_err_t area_jt808_0x8600(uint16_t fram_num,uint8_t *pmsg,u16 msg_len)
*��������:����ָ����γ����ÿ�����ȵĳ���.
*��	��:	latitude:��ǰγ��ֵ����λΪ�����֮һ��
*��	��:none
*�� �� ֵ:double 	:����ʵ��ÿ�����ȵĳ��ȣ���λΪ��
*��	��:������
*��������:2013-06-16
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
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
*��������:rt_err_t area_jt808_0x8600(uint16_t fram_num,uint8_t *pmsg,u16 msg_len)
*��������:����ָ����γ����ÿ�����ȵĳ��ȣ�Ȼ������û�����ľ�������Ӧ�ľ���������λΪ
			�Զ�Ϊ��λ�ľ���ֵ���� 10 �� 6 �η�����ȷ�������֮һ��
*��	��:	latitude:��ǰγ��ֵ����λΪ�����֮һ��
		distance:�û�����ľ���
*��	��:none
*�� �� ֵ:u32 	:����ʵ�ʵľ�����
*��	��:������
*��������:2013-06-16
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
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
*��������:u32 dis_Point2Point(u32 Lati_1,u32 Longi_1,u32 Lati_2,u32 Longi_2)
*��������:��������֮��ľ��룬
*��	��:	Lati_1	:��һ�����γ�ȣ���λΪ�����֮һ��
		Longi_1	:��һ����ľ��ȣ���λΪ�����֮һ��
		Lati_2	:�ڶ������γ�ȣ���λΪ�����֮һ��
		Longi_2	:�ڶ�����ľ��ȣ���λΪ�����֮һ��
*��	��:none
*�� �� ֵ:u32 	:����ʵ�ʵľ��룬��λΪ��
*��	��:������
*��������:2013-06-16
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
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
*��������:s32 dis_Point2Line(u32 Cur_Lati, u32 Cur_Longi,u32 Lati_1,u32 Longi_1,u32 Lati_2,u32 Longi_2)
*��������:����㵽��֮��ľ��룬
*��	��:	Cur_Lati	:��ǰ���γ�ȣ���λΪ�����֮һ��
		Cur_Longi	:��ǰ��ľ��ȣ���λΪ�����֮һ��
		Lati_1		:��һ�����γ�ȣ���λΪ�����֮һ��
		Longti_1	:��һ����ľ��ȣ���λΪ�����֮һ��
		Lati_2		:�ڶ������γ�ȣ���λΪ�����֮һ��
		Longti_2	:�ڶ�����ľ��ȣ���λΪ�����֮һ��
*��	��:none
*�� �� ֵ:u32 	:����ʵ�ʵľ��룬��λΪ��
*��	��:������
*��������:2013-06-16
������ѧԲ��:	1����ǰ����������ĺ���Ϊ		y = a*x;
				2�������������ֱ�������ĺ���Ϊ	y1= -(1/a)*x1 + b;
				3�����ȼ����:  a,b��1/a��Ȼ�����������ֵ��������������ཻ�ĵ��X������
				�й�ʽ	a*x	= -(1/a)*x + b;   ���  x2 = b / (a + 1/a),
				4������x2�������y2���꣬y = a * x,Ȼ���������㵽��ǰ��ľ��룬ͬʱ���������
				�Ƿ���������ɵ����м䣬���򷵻����������򷵻ظ���
				5��ע�⣬���еĵ�ļ��㶼��Lati_1��Longi_1��Ϊԭ��,����Ϊx�ᣬγ��Ϊy��
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
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
*��������:void commit_ack_ok(u16 fram_num,u16 cmd_id,u8 isFalse)
*��������:ͨ��Ӧ�𣬵���OKӦ��
*��	��:	fram_num:Ӧ����ˮ��
		cmd_id	:����ʱ��808����
		statue 	:��ʾ״̬��0:��ʾOK	1:��ʾʧ��	2:��ʾ��Ϣ����	3:��ʾ��֧��
*��	��:	none
*�� �� ֵ:rt_err_t
*��	��:������
*��������:2013-06-24
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
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
*��������:rt_err_t area_jt808_del(uint8_t linkno,uint8_t *pmsg, ENUM_AREA  del_State))
*��������:808ɾ��Բ����������
*��	��:	pmsg		:808��Ϣ������
		msg_len		:808��Ϣ�峤��
		del_State	:��ʾҪɾ��������
*��	��:none
*�� �� ֵ:rt_err_t
*��	��:������
*��������:2013-06-16
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
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
*��������:rt_err_t area_jt808_0x8600(uint8_t linkno,uint8_t *pmsg)
*��������:808����Բ����������
*��	��:	pmsg	:808��Ϣ������
		msg_len	:808��Ϣ�峤��
*��	��:none
*�� �� ֵ:rt_err_t
*��	��:������
*��������:2013-06-16
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
*********************************************************************************/
rt_err_t area_jt808_0x8600(uint8_t linkno,uint8_t *pmsg)
{
 u16 i;
 u16 datalen;
 u16 tempu16data;
 u32 tempu32data;
 u32 Longi,Lati;		///���Ⱥ�γ��
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
 ///808��Ϣ�岿��
 msg+=2;	///�������� 1byte,�������� 1byte
 
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

	///�������״��������
	Lati	= buf_to_data(msg+6, 4);
	Longi	= buf_to_data(msg+10, 4);
		///����Բ�뾶��Ӧ�ľ�����ֵ
	tempu32data=dis_to_Longi(Lati,buf_to_data(msg+14, 4));
	
	pTempHead->Rect_left.Longi	= Longi - tempu32data;
	pTempHead->Rect_right.Longi	= Longi + tempu32data;
		///����Բ�뾶��Ӧ��γ����ֵ
	tempu32data = buf_to_data(msg+14, 4)*1000000/WLENGH;
	pTempHead->Rect_left.Lati	= Lati + tempu32data;
	pTempHead->Rect_right.Lati	= Lati - tempu32data;
	///���Ƶ���Χ�����ݵ���Ҫд��Ľṹ����
	memcpy((void *)&pTempHead->ID,msg,datalen);
	area_flash_write_area(area_data,sizeof(area_data),pTempHead);
 	}
 commit_ack_ok(fram_num,0x8600,0);
 return RT_EOK;
}


/*********************************************************************************
*��������:rt_err_t area_jt808_0x8601(uint8_t linkno,uint8_t *pmsg)
*��������:808ɾ��Բ����������
*��	��:	pmsg	:808��Ϣ������
		msg_len	:808��Ϣ�峤��
*��	��:none
*�� �� ֵ:rt_err_t
*��	��:������
*��������:2013-06-16
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
*********************************************************************************/
rt_err_t area_jt808_0x8601(uint8_t linkno,uint8_t *pmsg)
{
 return area_jt808_del(linkno, pmsg, AREA_Circular);
}


/*********************************************************************************
*��������:rt_err_t area_jt808_0x8602(uint8_t linkno,uint8_t *pmsg)
*��������:808���þ�����������
*��	��:	pmsg	:808��Ϣ������
		msg_len	:808��Ϣ�峤��
*��	��:none
*�� �� ֵ:rt_err_t
*��	��:������
*��������:2013-06-16
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
*********************************************************************************/
rt_err_t area_jt808_0x8602(uint8_t linkno,uint8_t *pmsg)
{
 u16 i;
 u16 datalen;
 u16 tempu16data;
 u32 tempu32data;
 u32 Longi,Lati;		///���Ⱥ�γ��
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
 ///808��Ϣ�岿��
 msg+=2;	///�������� 1byte,�������� 1byte
 
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

	///�������״��������
	pTempHead->Rect_left.Longi	= buf_to_data(msg+10, 4);
	pTempHead->Rect_right.Longi	= buf_to_data(msg+18, 4);
	pTempHead->Rect_left.Lati	= buf_to_data(msg+6, 4);
	pTempHead->Rect_right.Lati	= buf_to_data(msg+14, 4);
	
	///���Ƶ���Χ�����ݵ���Ҫд��Ľṹ����
	memcpy((void *)&pTempHead->ID,msg,datalen);
	area_flash_write_area(area_data,sizeof(area_data),pTempHead);
 	}
 commit_ack_ok(fram_num,0x8602,0);
 return RT_EOK;
}


/*********************************************************************************
*��������:rt_err_t area_jt808_0x8603(uint8_t linkno,uint8_t *pmsg)
*��������:808ɾ��������������
*��	��:	pmsg	:808��Ϣ������
		msg_len	:808��Ϣ�峤��
*��	��:none
*�� �� ֵ:rt_err_t
*��	��:������
*��������:2013-06-16
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
*********************************************************************************/
rt_err_t area_jt808_0x8603(uint8_t linkno,uint8_t *pmsg)
{
 return area_jt808_del(linkno, pmsg, AREA_Rectangle);
}


/*********************************************************************************
*��������:rt_err_t area_jt808_0x8604(uint8_t linkno,uint8_t *pmsg)
*��������:808���ö������������
*��	��:	pmsg	:808��Ϣ������
		msg_len	:808��Ϣ�峤��
*��	��:none
*�� �� ֵ:rt_err_t
*��	��:������
*��������:2013-06-16
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
*********************************************************************************/
rt_err_t area_jt808_0x8604(uint8_t linkno,uint8_t *pmsg)
{
 u16 i;
 u16 datalen;
 u16 tempu16data;
 u32 tempu32data;
 u32 Longi,Lati;		///���Ⱥ�γ��
 TypeDF_AreaHead *pTempHead = NULL;
 u8 tempbuf[64];
 
 u8 *msg;
 u16 msg_len;
 u16 fram_num;
 
 msg_len	= buf_to_data( pmsg + 2, 2 ) & 0x3FF;
 fram_num	= buf_to_data( pmsg + 10, 2 );
 pmsg 		+= 12;
 msg	= pmsg;

 ///808��Ϣ�岿��
 
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
 ///��������
 tempu16data = buf_to_data(msg+datalen, 2);
 if(tempu16data > 64)			///�޶���󶥵���Ϊ64
 	tempu16data = 64;
 datalen += 2;
 
 pTempHead = (TypeDF_AreaHead *)rt_malloc( datalen + tempu16data * 8 );
 if( pTempHead )
 	{
 	///�������״��������
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

	///���Ƶ���Χ�����ݵ���Ҫд��Ľṹ����
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
*��������:rt_err_t area_jt808_0x8605(uint8_t linkno,uint8_t *pmsg)
*��������:808ɾ���������������
*��	��:	pmsg	:808��Ϣ������
		msg_len	:808��Ϣ�峤��
*��	��:none
*�� �� ֵ:rt_err_t
*��	��:������
*��������:2013-06-16
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
*********************************************************************************/
rt_err_t area_jt808_0x8605(uint8_t linkno,uint8_t *pmsg)
{
 return area_jt808_del(linkno, pmsg, AREA_Polygon);
}


/*********************************************************************************
*��������:rt_err_t area_jt808_0x8606(uint8_t linkno,uint8_t *pmsg)
*��������:808������·������
*��	��:	pmsg	:808��Ϣ������
		msg_len	:808��Ϣ�峤��
*��	��:none
*�� �� ֵ:rt_err_t
*��	��:������
*��������:2013-06-16
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
*********************************************************************************/
rt_err_t area_jt808_0x8606(uint8_t linkno,uint8_t *pmsg)
{
 u16 i;
 u16 datalen;
 u16 tempu16data;
 u32 tempu32data;
 u32 Longi,Lati;		///���Ⱥ�γ��
 TypeDF_AreaHead *pTempHead = NULL;
 u8 tempbuf[64];
 
 u8 *msg;
 u16 msg_len;
 u16 fram_num;
 
 msg_len	= buf_to_data( pmsg + 2, 2 ) & 0x3FF;
 fram_num	= buf_to_data( pmsg + 10, 2 );

 ///���������������ʱ��֧�ֶ��
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

 ///808��Ϣ�岿��
 datalen = 23;
 datalen += 6;
 tempu16data = buf_to_data(msg+4, 2);
 if(tempu16data & BIT(0))
	 {
	 datalen += 12;
	 }
 ///��������
 tempu16data = buf_to_data(msg+datalen, 2);
 if(tempu16data > 32)			///�޶���󶥵���Ϊ64
 	tempu16data = 32;
 datalen += 2;
 
 if( tempu16data > (msg_len - datalen) / 25 )
 	{
 	tempu16data = (msg_len - datalen) / 25 ;
 	}
 
 pTempHead = (TypeDF_AreaHead *)rt_malloc( datalen + tempu16data * 25 );
 if( pTempHead )
 	{
 	///�������״��������
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

	///���Ƶ���Χ�����ݵ���Ҫд��Ľṹ����
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
*��������:rt_err_t area_jt808_0x8607(uint8_t linkno,uint8_t *pmsg)
*��������:808ɾ����·��������
*��	��:	pmsg	:808��Ϣ������
		msg_len	:808��Ϣ�峤��
*��	��:none
*�� �� ֵ:rt_err_t
*��	��:������
*��������:2013-06-16
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
*********************************************************************************/
rt_err_t area_jt808_0x8607(uint8_t linkno,uint8_t *pmsg)
{
 return area_jt808_del(linkno, pmsg, AREA_Line);
}


/*********************************************************************************
*��������:u8 Check_CooisInRect(TypeStruct_Coor *pCoo, TypeDF_AreaHead *pHead )
*��������:�жϵ�ǰλ���Ƿ��ڸþ�����������
*��	��:	pCoo	:��ǰλ������
		pHead	:��ǰ��������
*��	��:none
*�� �� ֵ:u8	1:��ʾ�ڸ�����	0:��ʾ���ڸ�����
*��	��:������
*��������:2013-07-01
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
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
*��������:u8 Check_CooisInRect(TypeStruct_Coor *pCoo, TypeDF_AreaHead *pHead )
*��������:�жϵ�ǰλ���Ƿ��ڸþ�����������
*��	��:	StartTime	:��ʼʱ��
		EndTime		:����ʱ��
*��	��:none
*�� �� ֵ:u8	1:��ʾ�ڸ�ʱ��Σ�	0:��ʾ���ڸ�ʱ���
*��	��:������
*��������:2013-07-01
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
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
*��������:u8 area_process_circular(TypeStruct_Coor *pCoo, Type_AreaInfo *AreaInfo )
*��������:�жϵ�ǰλ���Ƿ��ڸ�Բ����������
*��	��:	pCoo	:��ǰλ������
		AreaInfo:��ǰԲ������
*��	��:none
*�� �� ֵ:u8	1:��ʾ�ڸ�����	0:��ʾ���ڸ�����
*��	��:������
*��������:2013-07-01
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
*********************************************************************************/
u8 area_process_circular(TypeStruct_Coor *pCoo, Type_AreaInfo *AreaInfo )
{
 u16 datalen;
 u16 attri;					///��������
 u32 lati,longi;			///�������ĵ�����
 u32 r;						///�뾶
 u32 d;						///����
 u32 curspeed;				///��ǰ�ٶ�
 u32 speed=0xFFFFFFFF;		///����ٶ�
 u32 speedtime=0xFFFFFFFF;	///����ʱ��
 if( Check_CooisInRect( pCoo, AreaInfo->area_data) == 0)
	{
 	goto AREA_CIRCULAR_OUT;
 	}
 datalen = 14;
 attri	= buf_to_data(AreaInfo->area_data->Data[0], 2);
 if(attri & BIT(0))		///����ʱ��
 	{
 	if(Check_Area_Time(&AreaInfo->area_data->Data[datalen],&AreaInfo->area_data->Data[6+datalen]) == 0)
 		{
 		goto AREA_CIRCULAR_OUT;
 		}
	datalen += 12;
 	}
 if(attri & BIT(1))		///����
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
 d		= dis_Point2Point(lati, longi, pCoo->Lati, pCoo->Longi );	///��ǰ�㵽���ĵ�ľ���
 if( d <= r )
 	{
 	AREA_CIRCULAR_IN:				///������
 	if( AreaInfo->in_area == 0)
 		{
 		AreaInfo->in_area = 1;
		AreaInfo->in_tick = rt_tick_get();
		
 		}
	if(attri & BIT(1))		///����
		{
		if( curspeed < speed )
			{
			AreaInfo->speed_tick = rt_tick_get();
			}
		}
	
 	return 1;
 	}
 AREA_CIRCULAR_OUT:				///������
 
 return 0;
}


void area_process(void)
{
 u16 i;
 TypeStruct_Coor cur_Coo;
 static u16 cur_area = 0;
 
 if(cur_area <= Area_Para.area_num)
 	cur_area = 0;
 ///��ȡ��ǰλ��
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

//--------  D�㵽ֱ�߾������-------
/*
     P1(x1,y1)   P2(x2,y2)  ,�ѵ�P(x1,y2)��Ϊ����ԭ�㣬��x1=0��y2=0��

     ��ô����P1��P2 ȷ����ֱ�߷���(����ʽ)Ϊ:
             (x-x1)/(x2-x1) =(y-y1)/(y2-y1)                          (1)
             
    ע:  ��׼ʽֱ�߷���Ϊ AX+BY+C=0;
             ��ôƽ��������һ��P(x0,y0) ��ֱ�ߵľ����ʾΪ
             d=abs(Ax0+By0+C)/sqrt(A^2+B^2)

    ���аѷ���ʽ(1) ת���ɱ�׼ʽΪ:
            (y2-y1)x+(x1-x2)y+x1(y1-y2)+y1(x2-x1)=0; 

   ���ڵ�(x1,y2)Ϊԭ��  ��x1=0��y2=0��  P1(0,y1) , P2(x2,0)
    ����   A=-y1 ,  B=-x2, C=y1x2
    ��ô ֱ�ߵķ���:
                  -y1x-x2y+y1x2=0;  (2)  
 
  =>     d=abs(-y1x0-x2y0+y1x2)/sqrt(y1^2+x2^2)       (3)

         ���� (3)  Ϊ����Ӧ�õĹ�ʽ 

        ע:  �ȸ��ݾ�γ���ۺϼ���� x0��y0��x1,y1,x2,y2  ����ֵ��λΪ: ��     
=>  �����ж�:
           ����(2) �������  �� P1(0,y1) , P2(x2,0) ������ֱ֪�ߴ�ֱ������ֱ�߷���
              P1(0,y1) :      x2x-y1y+y1^2=0  (4)
              P2(x2,0) :      x2x-y1y-x2^2=0  (5)

          ��� y1 >=0      ֱ��(4)    ��ֱ��(5)  ���ϱ�
          ��ô �����߶������ڵ��жϷ�����
                       (4) <=  0    ��  (5)  >=0 
       ��                
           ��� y1 <=0      ֱ��(5)    ��ֱ��(4)  ���ϱ�
          ��ô �����߶������ڵ��жϷ�����
                       (4) >=  0    ��  (5)  <=0 
   //------------------------------------------------------------------------------------------     
     
       γ��û�в�ֵ    1γ��  111km
       40��γ���� 1����Ϊ  85.3km   (��������)   111km*cos40

       X ��Ϊ ����(longitude) ��ֵ
       Y ��Ϊγ�� (latitude)  ��ֵ
                
     
   //------------------------------------------------------------------------------------------
*/

u32   Distance_Point2Line(u32 Cur_Lat, u32  Cur_Longi,u32 P1_Lat,u32 P1_Longi,u32 P2_Lat,u32 P2_Longi)
{   //   ���뵱ǰ�� �����ص㵽����ֱ�ߵľ���
      long  x0=0,y0=0,Line4_Resualt=0,Line5_Resualt=0;  // ��λ: ��
      long  y1=0; 
      long  x2=0;	
      long   distance=0;	 
     // long  Rabs=0;
//      long  Rsqrt=0;  
     long  DeltaA1=0,DeltaA2=0,DeltaO1=0,DeltaO2=0; //  DeltaA : Latitude     DeltaO:  Longitude 	   
     // u32   Line4_Resualt2=0,Line5_Resualt2=0; 
      double   fx0=0,fy0=0,fy1=0,fx2=0;	  
      double   FLine4_Resualt2=0,FLine5_Resualt2=0,fRabs=0,fRsqrt=0; 	  

	  // 0.   �ȴ��Ե��ж� 
	       DeltaA1=abs(Cur_Lat-P1_Lat); 
	       DeltaA2=abs(Cur_Lat-P2_Lat); 
		DeltaO1=abs(Cur_Lat-P1_Longi); 
	       DeltaO2=abs(Cur_Lat-P2_Longi); 
	   /* if((DeltaA1>1000000) &&(DeltaA2>1000000))	    
            {  
                rt_kprintf("\r\n  Latitude ��̫��\r\n");
                return   ROUTE_DIS_Default; 
	    }	
	     if((DeltaO1>1000000) &&(DeltaO2>1000000))	    
            {  
                rt_kprintf("\r\n  Longitude ��̫��\r\n"); 
                return   ROUTE_DIS_Default; 
	    }	  	
       */
	 // 1.  ��ȡ  P1(0,y1)   P2(x2,0) ,��P(x0,y0)    P(x1,y2)Ϊԭ��  ��x1=0��y2=0��  P1(0,y1) , P2(x2,0)
	  x2=abs(P2_Longi-P1_Longi); // a/1000000*85300=a 853/10000 m =a x 0.0853
         if(P2_Longi<P1_Longi)
		 	x2=0-x2;
	  fx2=(double)((double)x2/1000);
	 //rt_kprintf("\r\n P2_L=%d,P1_L=%d   delta=%d \r\n",P2_Longi,P1_Longi,(P2_Longi-P1_Longi));
	 // if(P2_Longi
         y1=abs(P2_Lat-P1_Lat); //  a/1000000*111000=a/9.009	����һ����õ����� �ٳ��� 111000 �׵õ�ʵ�ʾ��� 
         if(P2_Lat<P1_Lat) 
		 	y1=0-y1;
	   fy1=(double)((double)y1/1000);	  
	  //rt_kprintf("\r\n P2_LA=%d,P1_LA=%d   delta=%d \r\n",P2_Lat,P1_Lat,(P2_Lat-P1_Lat));

      //   rt_kprintf("\r\n ��֪��������: P1(0,%d)   P2(%d,0) \r\n", y1,x2); 
       //    ��ǰ��
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
        //   rt_kprintf("\r\n��ǰ������: P0(%d,%d)    \r\n", x0,y0);  
	  // 2. �ж�y1  �Ĵ�С�� ����� P1(0,y1)   P2(x2,0) ,����ֱ֪�ߵķ��̣����ж�
	  //     ��ǰ���Ƿ���·�δ�ֱ��Χ��
	  
             //  2.1   ����ǰ����룬 �� P1(0,y1)   �� ֱ�߷���(4)  ������
                                  Line4_Resualt=(x2*x0)-(y1*y0)+(y1*y1);
	                            FLine4_Resualt2=fx2*fx0-fy1*fy0+fy1*fy1;
	   //     rt_kprintf("\r\n Line4=x2*x0-y1*y0+y1*y1=(%d)*(%d)-(%d)*(%d)+(%d)*(%d)=%ld     x2*x0=%d    y1*y0=%d   y1*y1=%d  \r\n",x2,x0,y1,y0,y1,y1,Line4_Resualt,x2*x0,y1*y0,y1*y1); 
          //     rt_kprintf("\r\n FLine4=fx2*fx0-fy1*fy0+fy1*fy1=(%f)*(%f)-(%f)*(%f)+(%f)*(%f)=%f      fx2*fx0=%f    fy1*fy0=%f   fy1*fy1=%f  \r\n",fx2,fx0,fy1,fy0,fy1,fy1,FLine4_Resualt2,fx2*fx0,fy1*fy0,fy1*fy1); 
  
	      //   2.2   ����ǰ����룬 ��P2(x2,0) �� ֱ�߷���(5)  ������ 
                                  Line5_Resualt=(x2*x0)-y1*y0-x2*x2;
		                    FLine5_Resualt2=fx2*fx0-fy1*fy0-fx2*fx2;    
		//rt_kprintf("\r\n Line5=x2*x0-y1*y0-x2*x2=(%d)*(%d)-(%d)*(%d)-(%d)*(%d)=%ld     Se : %ld   \r\n",x2,x0,y1,y0,x2,x2,Line5_Resualt,Line5_Resualt2); 
          //    rt_kprintf("\r\n FLine5=fx2*fx0-fy1*fy0-fx2*fx2=(%f)*(%f)-(%f)*(%f)-(%f)*(%f)=%f      fx2*fx0=%f    fy1*fy0=%f   fx2*fx2=%f  \r\n",fx2,fx0,fy1,fy0,fx2,fx2,FLine5_Resualt2,fx2*fx0,fy1*fy0,fx2*fx2);   
 	      // rt_kprintf("\r\n  Line4_Resualt=%d     Line5_Resualt=%d  \r\n",Line4_Resualt,Line5_Resualt);    

	     if(fy1>=0)      //  ֱ��(4) ���Ϸ�
	  	{
               
			//   2.3   �ж�����    (4) <=  0    ��  (5)  >=0     // �ж�����ȡ��
			     if((FLine4_Resualt2>0) ||(FLine5_Resualt2<0))
				 	 return   ROUTE_DIS_Default;      //  �������������������ֵ
	  	}
	     else
	     	{      //  ֱ��(5)

                   	//   2.4   �ж�����     (4) >=  0    ��  (5)  <=0     // �ж�����ȡ�� 
			     if((FLine4_Resualt2<0) ||(FLine5_Resualt2>0)) 
				 	 return   ROUTE_DIS_Default;      //  �������������������ֵ 

	     	}	

              rt_kprintf("\r\n In judge area \r\n");  
		//rt_kprintf("\r\n   Current== Latitude:   %d     Longitude: %d     Point1== Latitude:   %d     Longitude: %d     Point2== Latitude:   %d     Longitude: %d\r\n",Cur_Lat,Cur_Longi,P1_Lat,P1_Longi,P2_Lat,P2_Longi);   

        //  3. ����ֵ�����ʵ�ʾ���
             #if 0
		    x2=x2*0.0853; // a/1000000*85300=a 853/10000 m =a x 0.0853
                 y1=y1/9; //  a/1000000*111000=a/9.009	����һ����õ����� �ٳ��� 111000 �׵õ�ʵ�ʾ��� 
                 x0=x0*0.0853;     
		   y0=y0/9; //  a/1000000*111000=a/9.009	    
	      #else
		   fx2=fx2*0.0853; // a/1000000*85300=a 853/10000 m =a x 0.0853
                 fy1=fy1/9; //  a/1000000*111000=a/9.009	����һ����õ����� �ٳ��� 111000 �׵õ�ʵ�ʾ��� 
                 fx0=fx0*0.0853;     
		   fy0=fy0/9; //  a/1000000*111000=a/9.009	      
             #endif

	 //  4. ������� 
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
