/***********the next is global define *****************/
/********************************************************************
********************RN8209 operation function*************************
********************************************************************/
#ifndef _RN8209_H
#define _RN8209_H

#include "stm32f10x.h"
#include "esp8266.h"
//#include "stdint.h"
//#include	<intrins.h>
#define		Meter_Debug			IsFause
#define		Istrue		1
#define		IsFause		2
#define		MeterIC_Type		Rn8209	  //
#define		RN8209		1
#define		Rn8209	  2
//ÏµÊý
/*
#define	D_GPQA		      0xf266
#define	D_GPQB		      0xf266	
#define	D_PhsA		      0xf4
#define	D_PhsB		      0xf4
#define	D_QPhsCal       0x0000
#define	D_APOSA         0x0000	
#define	D_APOSB         0x0000	
#define	D_RPOSA         0x0000
#define	D_RPOSB         0x0000
#define	D_IARMSOS       0x0000
#define	D_IBRMSOS       0x0000
#define	D_IBGain        0x0001
#define	VIArmsK         0x0DDA  //0x16c7  //0x0f69  //0x0DDA 
#define	VIBrmsK         0x0000
#define	VUrmsK          0x0491  //0x0473    // 0x031b     //????
#define	VPrmsK		   	  0x093f  //0x0654
#define	VPBrmsK		   	  0x0000
#define	VQArmsK		   	  0x0000	   
#define	VQBrmsK		   	  0x0000	
*/


//========================================================================
//=========================================================================
//------------RN8209 const define------------------------------------------
//=========================================================================
//#define		RN8209_Debug		IsFaulse
//	#define		IsTrue		1
//	#define		IsFaulse	0

#define	Con_EC_Power	    5		//3200=100*2^5
#define	Con_EC_Const		  12		//1200imp/kWh

#define	Con_Measure_WrEn	0xE5
#define	Con_Measure_WrDis	0xDC
#define	Con_ChannelsA 	  0x5A
#define	Con_ChannelsB	    0xA5
#define	Con_Reset		      0xFA	
//=========================================================================
/****************************Rn8209 register define **********************/
//===============================Page=====================================
#define	SYSCONAddr		0x00
#define	SysconstH		0x00
//#define	SysconstL		0x43
//#define	SysconstL		0x40

#define	EMUCONAddr		0x01
#define	EmuconstH		0x00
#define	EmuconstL		0x01

#define	HFConstAddr		0x02

#define	PStartAddr		0x03
#define PStartconstH	0x00
#define PStartconstL	0x01

#define QStartAddr		0x04
#define	GPQAAddr		0x05
#define	GPQBAddr		0x06
#define	PhsAAddr		0x07
#define	PhsBAddr		0x08
#define	QPhsCalAddr	0x09
#define	APOSAAddr		0x0A
#define	APOSBAddr		0x0B
#define	RPOSAAddr		0x0C
#define	RPOSBAddr		0x0D
#define	IARMSOSAddr		0x0E
#define	IBRMSOSAddr		0x0F
#define	IBGainAddr		0x10
                	
#define PFCntAddr		0x20
#define	QFcntAddr		0x21
#define	IARMSAddr		0x22
#define	IBRMSAddr		0x23
#define	URMSAddr		0x24
#define	UFreqAddr		0x25   //???????
#define	PowerPAAddr		0x26
#define	PowerPBAddr		0x27
#define	PowerQAddr		0x28
#define	EnergyPAddr		0x29
#define	EnergyP2Addr	0x2A
#define	EnergyQAddr		0x2B
#define	EnergyQ2Addr	0x2C
#define	EMUStatusAddr	0x2D

#define	EMURefLAddr		0x34
#define	EMURefHAddr		0x35
#define	SM801Pulse		0x36

#define	SM801IEAddr		0x40
#define	SM801IFAddr		0x41
#define	SM801RIFAddr	0x42

#define	SysStatusAddr   0x43
#define	SM801RDataAddr	0x44
#define	SM801WDataAddr	0x45

#define	SM801ChAToBAddr	0xEA

#define	DeviceIDAddr	0x7F 
//??????

//
#define SPICS_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_12)
#define SPICS_Set() GPIO_SetBits(GPIOB,GPIO_Pin_12)      //spi cs

#define SPICLK_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_13)
#define SPICLK_Set() GPIO_SetBits(GPIOB,GPIO_Pin_13)     //spi clk

#define SPISIMO_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_14)
#define SPISIMO_Set() GPIO_SetBits(GPIOB,GPIO_Pin_14)    //spi slave input from master output

#define SPISOMI_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_15)
#define SPISOMI_Set() GPIO_SetBits(GPIOB,GPIO_Pin_15)   //spi slave output to master input

/*******************************************************************
***			real 8209 paramete calculate                  ***
********************************************************************/
   /*
	uint16_t	xdata	VMeasureChkSum;
	uint16_t	xdata	VIArmsK;
	uint16_t	xdata	VIBrmsK;
	uint16_t	xdata	VUrmsK;
	uint16_t	xdata	VPrmsK;

	uint16_t	xdata	VPBrmsK;
	uint16_t	xdata	VQArmsK;
	uint16_t	xdata	VQBrmsK;
	*/
//===================================================================
/********************************************************************/
uint8_t 	Read_SPI_Byte(void);
void 	Write_SPI_Byte(uint8_t Data);
uint8_t	GetMeasureRegLen(uint8_t	RegAdr);
void 	Wr801_SPIWrNByte(uint8_t Addr,uint8_t *Point,uint8_t datalen);
void 	Rd801_SPIRdNByte(uint8_t Addr,uint8_t *Point,uint8_t	datalen);
void 	Wr801_Command(uint8_t CmdWord);
void 	StartJiaoBiao(void);
void 	CheckJiaoBiao(void);
void	InitMeasure(void);
uint8_t	ComReadMeasure(uint8_t	*ptr);
uint8_t	ComWriteMeasure(uint8_t	*ptr);
//void	Cal_Measur_Rms(uint8_t	*ptr);
void	Cal_PowerFactor(void);
void	Wr_RN8209_Start(void);
uint16_t	Cal_Measur_Rms(unsigned char  channel);
void get_Measur_Data(scoket_power_status_t power_data);
//===================================================================
#define		Tb_RN8209_Num		49

#endif
