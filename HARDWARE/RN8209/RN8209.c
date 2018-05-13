
#include "RN8209.H"
#include "delay.h"

//=============RN8209Ïà¹Ø=============
unsigned int      voltage=0;            //µçÑ¹	
unsigned int      Freq=0;               //µçÍøÆµÂÊ	 
unsigned int      current=0;			      //µçÍøµçÁ÷
unsigned int      power=0;              //µçÍø¹¦ÂÊ	 
unsigned char     POW_buf[5];           //ÊÐµç¹¦ÂÊ
unsigned char     CURRENT_buf[4];       //ÊÐµçµçÁ÷
unsigned char     VOL_buf[4];           //ÊÐµçµçÑ¹
unsigned char     FREQ_buf[4];          //µçÍøÆµÂÊ    
unsigned char     HF_buf[2];          //HFCONST
unsigned char     GPQA_buf[2];        //GPQA
unsigned char     KA_buf[2];          //????
unsigned char     KU_buf[2];          //????  
unsigned char     KPa_buf[2];         //????  
unsigned char     KQa_buf[2];         //???? 
unsigned char     tempbuf[4];  
unsigned char     PEnergy[4];
unsigned char     Energy_buf[4];        //ÄÜÁ¿»º³åÇø 
unsigned char     dianneng_buf[2];    //µçÄÜ»º³åÇø
//20171203
unsigned int      P_Energy=0;	 
unsigned char     UI_SEL=0;	            //½çÃæÑ¡Ôñ
unsigned char     VPFLampTimer;
unsigned char     VPFRecFlag;
unsigned char     VPFiltNum;
unsigned char     VPPulse;
//2015.11.7
unsigned char     read_freq_flag=0;		    //?
unsigned char     read_Un_flag=0;
unsigned char     read_Irms_flag=0;
unsigned char     read_PA_flag=0;
unsigned char     read_QA_flag=0;
unsigned char     read_HFCONST_flag=0;	    //?
unsigned char     read_GPQA_flag=0;	        //?
unsigned char     read_PHSA_flag=0;	        //?
unsigned char     read_KU_flag=0;		         //? KU
unsigned char     read_KI_flag=0;            //? KI
unsigned char     read_KP_flag=0;            //? Kp
unsigned char     read_KQ_flag=0;            //? Kq
unsigned char     read_Engery_flag=0;        //? ??
unsigned char     reade_e2_test_flg=0;        //?e2?????

unsigned char     set_KU_flag=0;		        //?? KU
unsigned char     set_KI_flag=0;              //?? KI
unsigned char     set_KP_flag=0;              //?? Kp
unsigned char     set_KQ_flag=0;              //?? Kq
unsigned char     set_HFCONST_flag=0;	        //??
unsigned char     set_GPQA_flag=0;	        //??
unsigned char     set_PHSA_flag=0;	        //??	
unsigned char     set_e2_test_flg=0;          //?e2?????
//2015-12-12	   
unsigned char     correc_flg=0;              //??????
uint16_t    aim_vol=0;                 //???????
uint16_t    aim_power=0;               //???????
uint8_t    e2_test1=0;             //????RAM
uint8_t    e2_test2=0;             //????RAM
uint8_t    correc_value=0;

uint16_t  	D_HFconst=0x1425;  //5A@1200imp/KWH@1/1200???
uint16_t  	D_GPQA=0xf266;	   //GPQA??????,??????????
uint16_t  	D_GPQB=0xf266;	
uint8_t 	D_PhsA=0xf4;	     //PHSAÖ»ÓëÎó²îÓÐ¹Ø£¬Ðè¹ÒÔÚ±íÌ¨ÉÏÐ£×¼
uint8_t 	D_PhsB=0xf4;
uint16_t  	D_QPhsCal=0x0000;
uint16_t  	D_APOSA=0x0000;	
uint16_t  	D_APOSB=0x0000;	
uint16_t  	D_RPOSA=0x0000;
uint16_t  	D_RPOSB=0x0000;
uint16_t  	D_IARMSOS=0x0000;
uint16_t  	D_IBRMSOS=0x0000;
uint16_t  	D_IBGain=0x0001;
uint16_t  	VUrmsK=0x0491;   //0x0473    // 0x031b     //????
uint16_t  	VIArmsK=0x0DDA;  //0x0f69  ????	  120µO??
uint16_t  	VIBrmsK=0x0000;
uint16_t  	VPrmsK=0x093f;   //0x0654  ????
uint16_t  	VPBrmsK=0x0000;
uint16_t  	VQArmsK=0x0000;	   
uint16_t  	VQBrmsK=0x0000;	

uint8_t         BStartMeasureFlag=0;
uint8_t         BRdE2ErrFlag=0;

const uint8_t 	TB_SM801RegTable[Tb_RN8209_Num]={
	SYSCONAddr,EMUCONAddr,HFConstAddr,PStartAddr,QStartAddr,GPQAAddr,GPQBAddr,PhsAAddr,
	PhsBAddr,QPhsCalAddr,APOSAAddr,APOSBAddr,RPOSAAddr,RPOSBAddr,IARMSOSAddr,IBRMSOSAddr,
	IBGainAddr,PFCntAddr,QFcntAddr,IARMSAddr,IBRMSAddr,URMSAddr,UFreqAddr,PowerPAAddr,
	PowerPBAddr,PowerQAddr,EnergyPAddr,EnergyP2Addr,EnergyQAddr,EnergyQ2Addr,EMUStatusAddr,EMURefLAddr,
	EMURefHAddr,SM801Pulse,SM801IEAddr,SM801IFAddr,SM801RIFAddr,SysStatusAddr,SM801RDataAddr,SM801WDataAddr,
	SM801ChAToBAddr,DeviceIDAddr,0x80,0x81,0x82,0x83,0x84,0x85,0x86
	};
//???????
const uint8_t		TB_SM801RegLenTb[Tb_RN8209_Num]={
	2,2,2,2,2,2,2,1,
	1,2,2,2,2,2,2,2,
	2,2,2,3,3,3,2,4,
	4,4,3,3,3,3,3,1,
	1,1,1,1,1,1,4,2,
	1,3,2,2,2,2,2,2,2
	}; 	
#define SPISOMI          GPIO_Pin_15 //8209 SPISOMI
//SPISOMI方向
void RN8209_SPISOMI_IO_SET(unsigned char io_set) //SPISOMI 方向
{
	GPIO_InitTypeDef GPIO_InitStructure;
  if(io_set==0)
  {
  GPIO_InitStructure.GPIO_Pin = SPISOMI;          //24C02 SDA ????
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  }
  else if(io_set==1)
  {
  GPIO_InitStructure.GPIO_Pin = SPISOMI;          //24C02 SDA ????
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //????
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  }
 else
  {;}
} 
/*************************************************************************
 *
 *Function:		Read_SPI_Byte
 *
 *describe:	read data from device's current address
 *
 *Input:	sck is low , cs is enable
 *
 *Output:		read out data
 *	 ?RN8209????????
 *************************************************************************/
uint8_t Read_SPI_Byte(void)
{
	uint8_t  i,j;	
	for(j=0;j<8;j++)
	{	
		delay_us(1000);	                //baund=3kHz
		
		//SPICLK=0;		    	//sck is down lock sda to read by master
		SPICLK_Clr();
		delay_us(1000);	                //baund=3kHz
		
		i=i<<1;					    //sda is "0"
		//SPISOMI=1;              //?????1,51???
		SPISOMI_Set();
		//PE6 设置为输入
		RN8209_SPISOMI_IO_SET(1);//1  输入
		delay_us(5);
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15 )==0)            //SPISDO????
		{
			i=i|0x01;		    //sda is "1"
		}				
		RN8209_SPISOMI_IO_SET(0);//0  输出
		//SPICLK=1;		        //sck is high	after finished
		SPICLK_Set();
	}	
	return(i);             	
}
/*************************************************************************
 *
 *Function:		Write_SPI_Byte
 *
 *Describe:	write data to the device's current address
 *
 *input:	sck is low , cs is enable, Data
 *
 *output:		none
 *	?SM801???1????
 ************************************************************************/
void Write_SPI_Byte(uint8_t Data)
{
	uint8_t  j;
	for(j=0;j<8;j++)
	{ 					
		if((Data&0x80)==0)
		{
			//SPISIMO=0;			//"0" is send to sda
			SPISIMO_Clr();
		}
		else
		{
			//SPISIMO=1;		    //"1" is send to sda
			SPISIMO_Set();
		}
		Data=Data<<1;
		
		delay_us(1000);          	//baund=3kHz
		
		//SPICLK=0;			    //sck down -->lock sda data to read by slave		
    SPICLK_Clr();		

		delay_us(1000);	            //baund=3kHz
		
		//SPICLK=1;		    	//sck is high	after finished
		SPICLK_Set();
	}
}
//*******************************************************************
//*
//*Function			::	GetMeasureRegLen
//*
//*Describe			::	get measure chip registor data len
//*
//*Input				::	registor addr
//*
//*Out					::	len
//*
//*Call					::	none
//*Effection		::	?????????
//*******************************************************************
uint8_t	GetMeasureRegLen(uint8_t	RegAdr)
{
	uint8_t	 	i;
	for(i=0;i<Tb_RN8209_Num;i++)
	{
		if(TB_SM801RegTable[i]==RegAdr)
		{
			return(TB_SM801RegLenTb[i]);
		}		
	}
	return(0xFF);		//error len=0xFF
}
/*************************************************************************
 *
 *Function:		Wr801_SPIWrNByte
 *
 *Describe:	write N bytes data to 8209 by spi bus,low byte is the first 
 *
 *input:	Address,Pointer,datalen
 *
 *output:		none
 *		   ?SM801???N????
 ************************************************************************/
void Wr801_SPIWrNByte(uint8_t Addr,uint8_t *Point,uint8_t datalen)
{		
	    uint8_t  	k;	
		//SPICS=0;
	  SPICS_Clr();
		delay_us(5);
		//SPICLK=1;	
    SPICLK_Set();
		Write_SPI_Byte(Addr|0x80);	//write command the highest bit is "1"
		for(k=0;k<datalen;k++)
		{
			Write_SPI_Byte(*(Point+k));	//write three bytes to spi slave 
		}
 		//SPICS=1;
		SPICS_Set();
 		delay_us(1000);
}
/*************************************************************************
 *
 *Function:		Rd801_SPIWrNByte
 *
 *Describe:	read N bytes data from slave by spi bus.low byte is the first 
 *
 *input:		slave register Address,ram buffer address Pointer,datalen
 *
 *output:		readout three bytes data
 *
  *call:		SpiOutputByte
 *affect:	Addr is not change,Point is not change 
 *?SM801??N????
 ************************************************************************/
void Rd801_SPIRdNByte(uint8_t Addr,uint8_t *Point,uint8_t datalen)
{	
	    uint8_t  k;
		//SPICLK=1;		//?????
	  SPICLK_Set();
		delay_us(5);
		//SPICS=0;		//CS??
	  SPICS_Clr();
		delay_us(5);
			
		Write_SPI_Byte(Addr&0x7f);	//write command	
				
		for(k=0;k<datalen;k++)
		{
			*(Point+k)=Read_SPI_Byte();
		}
		//SPICS=1;		//CS??
		SPICS_Set();
		delay_us(1000);		
}
/*************************************************************************
 *
 *Function:		Wr801_Command
 *
 *Describe:	 
 *
 *input:		
 *
 *output:		
 *
  *call:		
 *affect:	
 ************************************************************************/
void Wr801_Command(uint8_t CmdWord)
{			
		//SPICLK=1;
	  SPICLK_Set();
		delay_us(1);
		//SPICS=0;		
	  SPICS_Clr();
		delay_us(1000);		
		Write_SPI_Byte(0xEA);	//
		Write_SPI_Byte(CmdWord); 
		//SPICS=1;
	  SPICS_Set();
		delay_us(1000);	
}
/**************************************************************************
 *
 *function:	StartJiaoBiao
 *
 *describe:	start Rn8209 modify
 *
 *input:		none
 *
 *output:		none
 *	 ????
 SYSCONAddr=0043
 HFConstAddr=1000
 EMUCONAddr=0003
 PStartAddr=0060
 QStartAddr=0120
 ??12????
 ??17?
 ************************************************************************/
void StartJiaoBiao(void)
{
	uint8_t	 	i;
	uint8_t	 	Buff[4];
	Wr801_Command(Con_Measure_WrEn);		        //open Rn8209 write enable	????
	delay_us(5);	
	Buff[0]=0x00;
	Buff[1]=0x43;
	Wr801_SPIWrNByte(SYSCONAddr,&Buff[0],2);		//set syscon registor

	Buff[0]=0x14;
	Buff[1]=0x25;
	Wr801_SPIWrNByte(HFConstAddr,&Buff[0],2);   	//set hfconst registor		 0x1000 n 200uΩ锰铜
	Buff[0]=0x00;
	Buff[1]=0x03;		
	Wr801_SPIWrNByte(EMUCONAddr,&Buff[0],2);		//set emucon registor
		
	Buff[0]=0x00;	
	Buff[1]=0x60;
	Wr801_SPIWrNByte(PStartAddr,&Buff[0],2);		//set pstart registor
	Buff[0]=0x01;
	Buff[1]=0x20;
	Wr801_SPIWrNByte(QStartAddr,&Buff[0],2);		//set qstart registor
	
	Buff[0]=0x00;
	Buff[1]=0x00;	
	for(i=0;i<12;i++)
	{			
		Wr801_SPIWrNByte(TB_SM801RegTable[GPQAAddr+i],&Buff[0],GetMeasureRegLen(GPQAAddr+i));		//set other registor
	}		
	Wr801_Command(Con_Measure_WrDis);		//open Rn8209 write disable

 	//VMeasureChkSum=0;		                //?????
	//VIArmsK	=1;
	
	//VIBrmsK	=1;
	//VUrmsK	=1;		
} 
/*************************************************************************
 *
 *function:	InitMeasure
 *
 *describe:	inition Rn8209
 *		
 *
 *input:	none
 *
 *output:	none
 *call	:	
*************************************************************************/
void	InitMeasure(void)
{
	uint8_t 	i;
	uint16_t   	j;
	uint16_t    tempd=0;
	uint8_t  	Buff[3];
	
	StartJiaoBiao();	

	Wr801_Command(Con_Measure_WrEn);		//RN8209???
	/*
	for(i=0;i<17;i++)	                    //?17???
	{
	    //?eeprom????,?TB_SM801E2Table[i]????????Buff[i]
		//ReadE2DataCRC(TB_SM801E2Table[i],2,&Buff[0]);//?EEPROM????
		j=(uint16_t)(Buff[0]);
		if((i!=7)&&(i!=8))
		{
			j=(j<<8)+(uint16_t)(Buff[1]);
		}
		VMeasureChkSum+=j;
		Wr801_SPIWrNByte(TB_SM801RegTable[i],&Buff[0],GetMeasureRegLen(i));
	}
	*/
	i=0;
	tempd=D_GPQA;
    Buff[0]=tempd>>8;
	Buff[1]=tempd%256;
	Wr801_SPIWrNByte(TB_SM801RegTable[GPQAAddr+i],&Buff[0],2);		//set other registor
	i=1;
	tempd=D_GPQB;
    Buff[0]=tempd>>8;
	Buff[1]=tempd%256;
	Wr801_SPIWrNByte(TB_SM801RegTable[GPQAAddr+i],&Buff[0],2);		//set other registor
	i=2;
	tempd=D_PhsA;
    Buff[0]=D_PhsA;
	Wr801_SPIWrNByte(TB_SM801RegTable[GPQAAddr+i],&Buff[0],1);		//set other registor
	i=3;
	tempd=D_PhsB;
    Buff[0]=D_PhsB;
	Wr801_SPIWrNByte(TB_SM801RegTable[GPQAAddr+i],&Buff[0],1);		//set other registor
	i=4;
    tempd=D_QPhsCal;
    Buff[0]=tempd>>8;
	Buff[1]=tempd%256;
	Wr801_SPIWrNByte(TB_SM801RegTable[GPQAAddr+i],&Buff[0],2);		//set other registor
	i=5;
	tempd=D_APOSA;
    Buff[0]=tempd>>8;
	Buff[1]=tempd%256;
	Wr801_SPIWrNByte(TB_SM801RegTable[GPQAAddr+i],&Buff[0],2);		//set other registor
	i=6;
	tempd=D_APOSB;
    Buff[0]=tempd>>8;
	Buff[1]=tempd%256;
	Wr801_SPIWrNByte(TB_SM801RegTable[GPQAAddr+i],&Buff[0],2);		//set other registor
	i=7;
    tempd=D_RPOSA;
    Buff[0]=tempd>>8;
	Buff[1]=tempd%256;
	Wr801_SPIWrNByte(TB_SM801RegTable[GPQAAddr+i],&Buff[0],2);		//set other registor
    i=8;
	tempd=D_RPOSB;
    Buff[0]=tempd>>8;
	Buff[1]=tempd%256;
	Wr801_SPIWrNByte(TB_SM801RegTable[GPQAAddr+i],&Buff[0],2);		//set other registor
    i=9;
	tempd=D_IARMSOS;
    Buff[0]=tempd>>8;
	Buff[1]=tempd%256;
	Wr801_SPIWrNByte(TB_SM801RegTable[GPQAAddr+i],&Buff[0],2);		//set other registor
	i=10;
	tempd=D_IBRMSOS;
    Buff[0]=tempd>>8;
	Buff[1]=tempd%256;
	Wr801_SPIWrNByte(TB_SM801RegTable[GPQAAddr+i],&Buff[0],2);		//set other registor
	i=11;
	tempd=D_IBGain;
    Buff[0]=tempd>>8;
	Buff[1]=tempd%256;
	Wr801_SPIWrNByte(TB_SM801RegTable[GPQAAddr+i],&Buff[0],2);		//set other registor

	Wr801_Command(Con_Measure_WrDis);		//open Rn8209 write disable	     
		
	//VMeasureChkSum=~VMeasureChkSum;	 
} 	
/*************************************************************************
 *
 *function:	Cal_Measur_Rms
 *
 *describe:	calculate measure para rms
 *		
 *
 *input:	ptr->measure.para[0]
 *
 *output:	data->*(ptr)
 *call	:	
 *Notes	:
 ?????
*************************************************************************/ 
uint16_t	Cal_Measur_Rms(uint8_t  channel)
{	
	uint32_t		m,n,p;	
	uint8_t		i,j;
	uint8_t		len,adr;
	uint16_t		ka;
	uint8_t		Buff[4]={0};
	switch(channel)
	{
		case 0:                                 //??
		Rd801_SPIRdNByte(UFreqAddr,&Buff[0],2);
		ka=Buff[0]*256;
		ka=ka+Buff[1];			                 //?????   
		Freq=44744312/ka;
		break;
		case 1:        
		ka=VUrmsK;                              //0x05B7;
		Rd801_SPIRdNByte(URMSAddr,&Buff[0],3);  //3??
		m=Buff[0]*65536+Buff[1]*256+Buff[2];
		m=m*ka;
		m=m/1000;
		m=m/1000;
		voltage=(uint16_t)(m);                        //???? 220.0V	
		break;
		case 2:                                 //????
		ka=VIArmsK;                             //0x05B7;
		Rd801_SPIRdNByte(IARMSAddr,&Buff[0],3); //3??
		m=Buff[0]*65536+Buff[1]*256+Buff[2];
		m=m*ka;
		m=m/1000;
		m=m/1000;
		current=(uint16_t)(m);                        //????10.00A	
		break;
		case 3:
		/*
		ka=VPrmsK;                                 //0x05B7;
		Rd801_SPIRdNByte(PowerPAAddr,&Buff[0],4);  //4??
		m=0;
		m=16777216-(Buff[1]*65536+Buff[2]*256+Buff[3]);
		m=m/10;
		m=m*ka;
		m=m/1000;
		m=m/1000; 	 
		power=(uint16_t)(m);                            //????W 
		*/
				// current=916;
				// voltage=2164; 
		m=(uint32_t)current*(uint32_t)voltage/1000;
		power=(uint16_t)(m);  
		break;
		 default:
		break;
	}	 
}  

void get_Measur_Data(scoket_power_status_t power_data)
{	
	//Cal_Measur_Rms(0);		             
	power_data.voltate_val = (float) Cal_Measur_Rms(1)/10;			          
	power_data.current_val = (float)Cal_Measur_Rms(2)/10;			           
	power_data.watt_val = (float)Cal_Measur_Rms(3)/10;			   
}

//电能存储
void  store_Engery(void)
{
  //EA=0;
  //write2404(dianneng_buf[0],0x11);      //???? 
  //write2404(dianneng_buf[1],0x12);      //????
  P_Energy=dianneng_buf[0]*256+dianneng_buf[1];
  //EA=1;
}
