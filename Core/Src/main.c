/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
#include "stdio.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define  ExpValve_Open()        (GPIOC->ODR&=(~(1<<2)))
#define  ExpValve_Close()       (GPIOC->ODR|=(1<<2))
#define  InspValve_Close()      (GPIOC->ODR|=(1<<3))
#define  InspValve_Open()       (GPIOC->ODR&=(~(1<<3)))


#define  CMVPC                   1
#define  CMVVC                   2
#define  SIMVPC                  3
#define  SIMVVC                  4
#define  cPAP                    5
#define  BiPAP                   6
#define  PSV                     7
#define  APRVC                   8

#define  Exp_Cycle               1
#define  Insp_Cycle              0

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const float delP_sensitivity=1.000;
const float gP_sensitivity=44.13;
const float gP_samples=10;
const float delP_samples=8;
uint16_t AdcData[2];
uint16_t gP_volt,delP_volt;
uint16_t delPraw,gPraw;
uint16_t gP_oldF,gP_newF,delP_oldF,delP_newF;
uint16_t P_Offset,P_Online,P_Online1;

float delPOffset=2532.40,delPOnline;
float pressure=0,P_cmh2o=0,P_cmh2o1=0;
float delp_flow1=0,dp=0,dp1=0;
float Volume=0,Flow_Volume;
double	Flow=0,Flow1=0,Flow2=0,delp_flow=0;
int P_UART,F_UART,V_UART;
int P_USB,F_USB,V_USB;
uint32_t UART_count=0;
uint8_t  debug=1;
#define ADS1115_ADDRESS 0x49
unsigned char ADSwrite[3];
unsigned char ADSread[2];
int16_t O2raw;
int16_t AWflowraw;
int16_t O2flowraw;
int16_t Pressureraw;
float P_volt;
float F_volt;
uint16_t	flow_F=0;
	long c2=0,c1=0;
	
/* tab communication data Manipulating Parameters */	

char Rx_UART[]="s0000";
char Rx_USB[6];
char Tx_UART[]="999\n";
char Tx_UART_Regular[]="P00Q0000V00000\n";
char Tx_UART_Sampled[]="00p0000q0000v0\n";
char Tx_USB_Regular[]="P00Q0000V00000\n";
char Tx_USB_Sampled[]="00p0000q0000v0\n";
float reacedVolume=0,reacedPIP=0,reacedPEEP=0;
char Flag;
uint8_t mode=1,stop=0,confirm=0,status=0,u_status=0; 
int value1=0;


/* set data Calculating Parameters */	
uint8_t PIP=50,PEEP=0,reached_pip=0,reached_peep=0;
uint8_t RespiratoryRate=20,Iratio=1,Eratio=2;
float Tinsp2=0,Texp2=0,Tinsp1=0,Texp1=0,Tresp=0,b_speed=0;
uint16_t Tinsp=0,Texp=0,last_insp=0,last_exp=0;
uint16_t reached_volume=0,PIP_Raw=0,PEEP_Raw=0; 
float FiO2=21,Trise=0;
uint16_t VOL=250,Timeslice=1;
uint8_t speed=0;
uint16_t DataValue=0, blowerSpeed=0,Timeslice;
uint8_t cycle=0,count=0,breath_cycle=0,stabilize_count=0;
uint16_t fl1=0;
uint8_t vflag=0;
typedef struct 
{
	uint8_t     running_mode;
	uint8_t     RR;
	uint8_t     I;
	uint8_t     E;
	uint8_t     Pip;
	uint8_t     Peep;
	uint16_t    Vol;
	float       O2;
	float       Trise;
}running_data;
 
running_data Data1={0,0,0,0,0,0,0,0,0}; 

typedef struct 
{
	uint8_t    RespiratoryRate;
	uint8_t    InspirationRatio;
	uint8_t    ExpirationRatio;
	uint8_t    pip;
	uint8_t    peep;
	uint16_t   Volume;
	float      Oxygen;
	float      Risetime;
}Modes;

  Modes CMV_PC= {15,1,2,50,5,250,21,1};
	Modes CMV_VC= {15,1,2,30,5,1000,21,1};
	Modes SIMV_PC= {15,1,2,30,5,1,21,1};
	Modes SIMV_VC= {15,1,2,30,5,5,21,1};
	Modes PS_V=    {15,1,2,30,5,5,21,1};
	Modes cPAP_V= {15,1,2,30,5,5,21,1};
	Modes BiPAP_V= {15,1,2,30,5,1,21,1};
	Modes APR_VC= {15,1,2,30,5,5,21,1};
	
	
struct States
{
	void(*functionPointer)(void);
	uint32_t countInstances;
	const struct States *Next_Sequence[2];
};
typedef const struct States Sequence;

ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Breath_update(void)
{
	if(last_insp!=Tinsp)
	{
	HAL_TIM_Base_Stop_IT(&htim2);
	TIM2->ARR=Tinsp;
	last_insp=Tinsp;
	HAL_TIM_Base_Start_IT(&htim2);
	}
	else
	{
	HAL_TIM_Base_Stop_IT(&htim2);
	TIM2->ARR=Tinsp;
	HAL_TIM_Base_Start_IT(&htim2);
	}
}
int Decode(char Rx1[])
{
	int value=0;
	if(Rx1[0]=='\n')return 0;
	else
{
	Flag=Rx1[0];
  for(int i=1;Rx1[i]!='\n';i++)
	{
	value=(value*10)+(Rx1[i]-48);
	}
}
	return value;
}
void Tinex_calc(void)
	{
	Tresp=(600000/(Data1.RR));                               // 10000 timercount=1sec,600000count=min
	Tinsp2=1.00*Data1.I/(Data1.I+Data1.E);
	Tinsp1=Tinsp2*Tresp;	
  Texp1=Tresp-Tinsp1;
	Tinsp=(uint16_t)Tinsp1;
 	Texp=(uint16_t)Texp1;		
  Tinsp= Tinsp-1;	
	Texp=Texp-1;
	}
uint16_t F_gPCalc(uint16_t value)
{
	  gPraw=value;
	/*  gP_newF=gPraw;
		gP_newF=((gP_newF-gP_oldF)/gP_samples)+gP_oldF;
		gP_oldF=gP_newF;*/
 	  gP_volt=(gPraw*5000)/1023;
	  return gP_volt;
}



	void Task_BreathSequence(void)
{

	
}	
uint16_t ADS1115_Getdata(uint8_t channel)
{
		  uint16_t  ADCraw=0;
		ADSwrite[0] = 0x01;
		switch(channel)
		{
			case 0:
		 				ADSwrite[1] =0xCD;                         // 1100 1101       0.256V,Channel_0      
		   			break;
		 case 1:
			     ADSwrite[1] = 0xD3;                        // 1101 0011       4.096V,Channel_1
		   		 break;
		 case 2:
			     ADSwrite[1] = 0xE3;                        // 1110 0011       4.096V,Channel_2
		       break;
		 case 3:
			     ADSwrite[1] = 0xC3;                        // 1111 0011   4.096V,Channel_3//changed to  1100 0011   4.096V,Channel_3
		       break;
		}     
	      	
	        ADSwrite[2] = 0x83;                         // 10000011     128 SamplesperSecond,No comparator //changed to 0x83 
		      HAL_Delay(10);    
      		HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS<<1, ADSwrite, 3, 10);
				  ADSwrite[0] = 0x00;
		  		HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS<<1, ADSwrite, 1, 10);
	      	
					HAL_I2C_Master_Receive(&hi2c1, (ADS1115_ADDRESS<<1),ADSread,2,10);
		     
		    ADCraw = (ADSread[0] << 8 | ADSread[1]);
		    
		    return   ADCraw;
		}
float Raw_Volt(uint16_t r)
{
	float volt=0;
	volt=(r*4096.00)/32768;
	return volt;
}
uint16_t F_delPCalc(uint16_t value)
{ 
	 	delPraw=value;
  	delP_newF=delPraw;
		delP_newF=((delP_newF-delP_oldF)/5.00)+delP_oldF;
	 delP_oldF=delP_newF;
	  return delP_newF;
}
	 void GetFlow(void)
	 {
	 	AWflowraw=ADS1115_Getdata(3);
	flow_F=F_delPCalc(AWflowraw);
F_volt=Raw_Volt(flow_F);
	 }
	 void	function_gPcalibration(void)
	{
		for(int n=0;n<1000;n++)
		{
    P_Offset=F_gPCalc(AdcData[1]);
		HAL_Delay(1);
		}
		
	}
  
	void	function_delPcalibration(void)
	{
		for(int n=0;n<1000;n++)
		{
		GetFlow();
	  delPOffset=(F_volt);
  	HAL_Delay(1);			
	 }
	}
	void Task_OffsetCalculations(void)
{
	  InspValve_Close();
	  ExpValve_Close();
	  HAL_Delay(2000);
		function_gPcalibration();
  	function_delPcalibration();
}
void Task_ReceivingPara(void)
{
	if(Flag=='Z')
	  {
		switch(stop)
		{
		 case 0:
			 
		 debug=0;
//				if(u_status==1)
//			{	
//					HAL_TIM_Base_Start_IT(&htim2);
//			} 
			break;
		 case 1:
					 debug=1; 
//			HAL_TIM_Base_Stop_IT(&htim2);
//		  ExpValve_Open();
//		  InspValve_Close();
			break;
		 default:
			 break;
	}
}
		switch(mode)
		{
		case CMVPC:
		CMV_PC.RespiratoryRate=RespiratoryRate;
		CMV_PC.InspirationRatio=Iratio;
		CMV_PC.ExpirationRatio=Eratio;
		CMV_PC.pip=PIP;
		CMV_PC.peep=PEEP;
		CMV_PC.Volume=VOL;
		CMV_PC.Risetime=Trise;
		CMV_PC.Oxygen=FiO2;
			break;
			case CMVVC:
		CMV_VC.RespiratoryRate=RespiratoryRate;
		CMV_VC.InspirationRatio=Iratio;
		CMV_VC.ExpirationRatio=Eratio;
		CMV_VC.pip=PIP;
		CMV_VC.peep=PEEP;
		CMV_VC.Volume=VOL;
		CMV_VC.Risetime=Trise;
		CMV_VC.Oxygen=FiO2;
		 	break;
			case SIMVVC:
		SIMV_VC.RespiratoryRate=RespiratoryRate;
		SIMV_VC.InspirationRatio=Iratio;
		SIMV_VC.ExpirationRatio=Eratio;
		SIMV_VC.pip=PIP;
		SIMV_VC.peep=PEEP;
		SIMV_VC.Volume=VOL;
		SIMV_VC.Risetime=Trise;
		SIMV_VC.Oxygen=FiO2;
		 	break;
			case SIMVPC:
		SIMV_PC.RespiratoryRate=RespiratoryRate;
		SIMV_PC.InspirationRatio=Iratio;
		SIMV_PC.ExpirationRatio=Eratio;
		SIMV_PC.pip=PIP;
		SIMV_PC.peep=PEEP;
		SIMV_PC.Volume=VOL;
		SIMV_PC.Risetime=Trise;
		SIMV_PC.Oxygen=FiO2;
		 	break;
			case cPAP:
		cPAP_V.RespiratoryRate=RespiratoryRate;
		cPAP_V.InspirationRatio=Iratio;
		cPAP_V.ExpirationRatio=Eratio;
		cPAP_V.pip=PIP;
		cPAP_V.peep=PEEP;
		cPAP_V.Volume=VOL;
		cPAP_V.Risetime=Trise;
		cPAP_V.Oxygen=FiO2;
		  break;
			case BiPAP:
		BiPAP_V.RespiratoryRate=RespiratoryRate;
		BiPAP_V.InspirationRatio=Iratio;
		BiPAP_V.ExpirationRatio=Eratio;
		BiPAP_V.pip=PIP;
		BiPAP_V.peep=PEEP;
		BiPAP_V.Volume=VOL;
		BiPAP_V.Risetime=Trise;
		BiPAP_V.Oxygen=FiO2;
		   break;
			case APRVC:
		APR_VC.RespiratoryRate=RespiratoryRate;
		APR_VC.InspirationRatio=Iratio;
		APR_VC.ExpirationRatio=Eratio;
		APR_VC.pip=PIP;
		APR_VC.peep=PEEP;
		APR_VC.Volume=VOL;
		APR_VC.Risetime=Trise;
		APR_VC.Oxygen=FiO2;
			break;
			case PSV:
		PS_V.RespiratoryRate=RespiratoryRate;
		PS_V.InspirationRatio=Iratio;
		PS_V.ExpirationRatio=Eratio;
		PS_V.pip=PIP;
		PS_V.peep=PEEP;
		PS_V.Volume=VOL;
		PS_V.Risetime=Trise;
		PS_V.Oxygen=FiO2;
				break;
			default:
			break;
	}
	if(confirm==1)
	//		if((stop==0)&&(confirm==1))
		 {
		 Data1.running_mode=mode;
		 switch(Data1.running_mode)
		 {
	     case CMVPC:
		Data1.RR=CMV_PC.RespiratoryRate;
    Data1.I=CMV_PC.InspirationRatio;
		Data1.E=CMV_PC.ExpirationRatio;
		Data1.Pip=CMV_PC.pip;
		Data1.Peep=CMV_PC.peep;
		Data1.Vol=CMV_PC.Volume;
		Data1.Trise=CMV_PC.Risetime;
		Data1.O2=CMV_PC.Oxygen;
	     break;
			 case CMVVC:
	  Data1.RR=CMV_VC.RespiratoryRate;
    Data1.I=CMV_VC.InspirationRatio;
		Data1.E=CMV_VC.ExpirationRatio;
		Data1.Pip=CMV_VC.pip;
		Data1.Peep=CMV_VC.peep;
		Data1.Vol=CMV_VC.Volume;
		Data1.Trise=CMV_VC.Risetime;
		Data1.O2=CMV_VC.Oxygen;
			 break;
			 case SIMVVC:
		Data1.RR=SIMV_VC.RespiratoryRate;
    Data1.I=SIMV_VC.InspirationRatio;
		Data1.E=SIMV_VC.ExpirationRatio;
		Data1.Pip=SIMV_VC.pip;
		Data1.Peep=SIMV_VC.peep;
		Data1.Vol=SIMV_VC.Volume;
		Data1.Trise=SIMV_VC.Risetime;
		Data1.O2=SIMV_VC.Oxygen;
			 break;
			 case SIMVPC:
		Data1.RR=SIMV_PC.RespiratoryRate;
    Data1.I=SIMV_PC.InspirationRatio;
		Data1.E=SIMV_PC.ExpirationRatio;
		Data1.Pip=SIMV_PC.pip;
		Data1.Peep=SIMV_PC.peep;
		Data1.Vol=SIMV_PC.Volume;
		Data1.Trise=SIMV_PC.Risetime;
		Data1.O2=SIMV_PC.Oxygen;
			break;
			 case cPAP:
		Data1.RR=cPAP_V.RespiratoryRate;
    Data1.I=cPAP_V.InspirationRatio;
		Data1.E=cPAP_V.ExpirationRatio;
		Data1.Pip=cPAP_V.pip;
		Data1.Peep=cPAP_V.peep;
		Data1.Vol=cPAP_V.Volume;
		Data1.Trise=cPAP_V.Risetime;
		Data1.O2=cPAP_V.Oxygen;
				break;
			 case BiPAP:
		Data1.RR=BiPAP_V.RespiratoryRate;
    Data1.I=BiPAP_V.InspirationRatio;
		Data1.E=BiPAP_V.ExpirationRatio;
		Data1.Pip=BiPAP_V.pip;
		Data1.Peep=BiPAP_V.peep;
		Data1.Vol=BiPAP_V.Volume;
		Data1.Trise=BiPAP_V.Risetime;
		Data1.O2=BiPAP_V.Oxygen;
			 	break;
			 case APRVC:
  	Data1.RR=APR_VC.RespiratoryRate;
    Data1.I=APR_VC.InspirationRatio;
		Data1.E=APR_VC.ExpirationRatio;
		Data1.Pip=APR_VC.pip;
		Data1.Peep=APR_VC.peep;
		Data1.Vol=APR_VC.Volume;
		Data1.Trise=APR_VC.Risetime;
		 Data1.O2=APR_VC.Oxygen;
		  	break;
			 case PSV:
		Data1.RR=PS_V.RespiratoryRate;
    Data1.I=PS_V.InspirationRatio;
		Data1.E=PS_V.ExpirationRatio;
		Data1.Pip=PS_V.pip;
		Data1.Peep=PS_V.peep;
		Data1.Vol=PS_V.Volume;
		Data1.Trise=PS_V.Risetime;
		 Data1.O2=PS_V.Oxygen;
			 break;
			 default:
			break;
		}
	 	Tinex_calc();	
		breath_cycle=1;
		HAL_TIM_Base_Start_IT(&htim2);
		Volume=0;
		u_status=1;
		confirm=0; 
	   } 
	 }


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

	 void _Flow()
{
   GetFlow();
	delp_flow1 =(F_volt-delPOffset)/1000.00;  //kpa

			dp=delp_flow1*10; //mbar

			Flow1=0.1512*dp*dp*dp-3.3424*dp*dp+41.657*dp;
		//	y=-0.0030x2+1.6819x+1.6929
	//y=-0.0047x2+1.9158x-6.8949 // distorted values taken +ve flow
	//y=0.0000x3+0.0109x2+1.8974x+6.6549 // distorted values -ve flow
	//y=-0.0001x3+0.0073x2+1.0867x-0.2701 // sunday upto 100 cubic
	/////////////////////
			if(Flow1>=0)
			{
				//Flow1=-0.0070*Flow1*Flow1+1.5340*Flow1-2.5515;
		Flow2=(0.0073*Flow1*Flow1)+(1.0867*Flow1)-0.2701-(0.0001*Flow1*Flow1*Flow1); 
//	      Flow1=0.007*Flow1*Flow1+0.449*Flow1+0.0021; //new equation from ADS1115
//				Flow1=-0.0030*Flow1*Flow1+1.6819*Flow1+1.6929; //+Ve

//		    Flow=-0.0047*Flow*Flow+1.9158*Flow-6.8949; //distorted +Ve
			}
			else
			{
			        Flow2=0.0039*Flow1*Flow1+1.1436*Flow1-0.5171;
				   //   Flow1=0.0012*Flow1*Flow1+0.8367*Flow1-7.1496;
			
//				    Flow1=-0.004*Flow1*Flow1+1.496*Flow1+1.743; //-ve
//					//Flow=0.0109*Flow*Flow+1.8974*Flow-6.6549;  //distorted -Ve
			}
		//	if((Flow2<1.2)&&(Flow2>-1.2))
	//		{
		//		Flow2=0;
		//	}
				F_USB=(int)Flow2;
	    	F_UART=F_USB;
							sprintf(Tx_USB_Regular,"P%02dQ%04dV%04d%d\n",(int)P_USB,(int)F_USB,(int)V_USB,cycle);
            	CDC_Transmit_FS((uint8_t*)Tx_USB_Regular,strlen(Tx_USB_Regular));
				    	sprintf(Tx_UART_Regular,"P%02dQ%04dV%04d%d\n",(int)P_UART,(int)F_UART,(int)V_UART,cycle);
	            HAL_UART_Transmit(&huart3,(uint8_t*)Tx_UART_Regular,sizeof(Tx_UART_Regular),5);
			
    sprintf(Tx_USB_Sampled,"p%02dq%04dv%04d%d\n",(int)reacedPEEP,(int)reacedPIP,(int)reacedVolume,cycle);
		CDC_Transmit_FS((uint8_t*)Tx_USB_Sampled,strlen(Tx_USB_Regular));
	 	sprintf(Tx_UART_Sampled,"p%02dq%04dv%04d%d\n",(int)reacedPEEP,(int)reacedPIP,(int)reacedVolume,cycle);
	  HAL_UART_Transmit(&huart3,(uint8_t*)Tx_UART_Sampled,sizeof(Tx_UART_Sampled),5);
		 
	 HAL_UART_Receive_IT(&huart3,(uint8_t*)Rx_UART,sizeof(Rx_UART));
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC_Init();

  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	  GPIOE->ODR&=(~(1<<5));  // BLDC disable PE5
//		HAL_DAC_Start(&hdac,DAC1_CHANNEL_1);
//	HAL_DAC_SetValue(&hdac,DAC1_CHANNEL_1,DAC_ALIGN_12B_R,0);	
	HAL_Delay(3000);
	HAL_UART_Receive_IT(&huart3,(uint8_t*)Rx_UART,sizeof(Rx_UART));
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)AdcData,2);
	Task_OffsetCalculations();
	HAL_TIM_Base_Start_IT(&htim3);
 	HAL_DAC_Start(&hdac,DAC1_CHANNEL_1);
 // BLDC enable PE5
	Volume=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
P_Online=F_gPCalc(AdcData[1]); 
pressure=P_Online-P_Offset;
P_cmh2o1=pressure/gP_sensitivity;
P_cmh2o=P_cmh2o1;
P_USB=(int)P_cmh2o;
P_UART=P_USB;
	_Flow();
			if(debug==1)
		{
			 InspValve_Open();
			 ExpValve_Close();
			 reacedPIP=P_cmh2o;
		   reacedVolume=Volume;
       _Flow();
		}
		////////////////////////////////////////////////////
			if(debug==0)
		{
			////////////////////////////////////////////////CMVPC
if(stabilize_count>3)
				{
		if(Data1.running_mode==CMVPC)
				{
		if(breath_cycle==0)
			{
		 GPIOE->ODR|=(1<<5);
		blowerSpeed=(PIP/60.00)*4095.00;
		HAL_DAC_SetValue(&hdac,DAC1_CHANNEL_1,DAC_ALIGN_12B_R,blowerSpeed);	
		if(P_cmh2o>=PIP)
	    { 
				
					 reacedPIP=P_cmh2o;
				  reacedVolume=Volume;
	while(breath_cycle==0)
					{
				HAL_DAC_SetValue(&hdac,DAC1_CHANNEL_1,DAC_ALIGN_12B_R,50);	
				InspValve_Close();
				ExpValve_Close();
		P_Online=F_gPCalc(AdcData[1]);
		pressure=P_Online-P_Offset;
P_cmh2o1=pressure/gP_sensitivity;
P_cmh2o=P_cmh2o1;
P_USB=(int)P_cmh2o;
P_UART=P_USB;
 _Flow();

					}
			}
			else
			{
				
				InspValve_Open();
				ExpValve_Close();
			}
		}
				if(breath_cycle==1)
		{
		if(P_cmh2o<=PEEP)
	    { 
						reacedPEEP=P_cmh2o; 
			    
 while(breath_cycle==1)
					{		
				InspValve_Close();
				ExpValve_Close();
			  P_Online=F_gPCalc(AdcData[1]);
		pressure=P_Online-P_Offset;
	P_cmh2o1=pressure/gP_sensitivity;
	P_cmh2o=P_cmh2o1;
	P_USB=(int)P_cmh2o;
	P_UART=P_USB;
						_Flow();
		   	}
		}
			else
			{
					ExpValve_Open();
				InspValve_Close();
			
		}
		
	}	
}
				/////////////////////////////////////////////////////////////
		if(Data1.running_mode==CMVVC)
				{
							if(breath_cycle==0)
			{
				 GPIOE->ODR|=(1<<5);
				blowerSpeed=(VOL/1200.00)*4095;
				HAL_DAC_SetValue(&hdac,DAC1_CHANNEL_1,DAC_ALIGN_12B_R,blowerSpeed);
	if(Volume>=VOL)
	    {  
    		 reacedPIP=P_cmh2o;
				  reacedVolume=Volume;
		while(breath_cycle==0)
					{
		 		HAL_DAC_SetValue(&hdac,DAC1_CHANNEL_1,DAC_ALIGN_12B_R,0);
				InspValve_Close();
				ExpValve_Close();
		P_Online=F_gPCalc(AdcData[1]);
		pressure=P_Online-P_Offset;
P_cmh2o1=pressure/gP_sensitivity;
P_cmh2o=P_cmh2o1;
P_USB=(int)P_cmh2o;
P_UART=P_USB;
 _Flow();
			}
			}
			else
			{

				InspValve_Open();
				ExpValve_Close();
			}
		}
				if(breath_cycle==1)
		{
		if(P_cmh2o<=PEEP)
	    { 
			 reacedPEEP=P_cmh2o; 
        while(breath_cycle==1)
					{		
				InspValve_Close();
				ExpValve_Close();
			  P_Online=F_gPCalc(AdcData[1]);
		pressure=P_Online-P_Offset;
	P_cmh2o1=pressure/gP_sensitivity;
	P_cmh2o=P_cmh2o1;
	P_USB=(int)P_cmh2o;
	P_UART=P_USB;
						_Flow();
			}
		}
			else
			{
					ExpValve_Open();
				InspValve_Close();
			
		}
	}		
	}

				if(Data1.running_mode==SIMVPC)
				{
		if(breath_cycle==0)
			{
		reacedPEEP=P_cmh2o; 
		if(P_cmh2o>=PIP)
	    { 
					 reacedPIP=P_cmh2o;
				  reacedVolume=Volume;
	while(breath_cycle==0)
					{
		//		InspValve_Close();
				ExpValve_Close();
				P_Online=F_gPCalc(AdcData[1]);
		pressure=P_Online-P_Offset;
P_cmh2o1=pressure/gP_sensitivity;
P_cmh2o=P_cmh2o1;
P_USB=(int)P_cmh2o;
P_UART=P_USB;
 _Flow();

					}
			}
			else
			{
				
				InspValve_Open();
				ExpValve_Close();
			}
		}
				if(breath_cycle==1)
		{
		if(P_cmh2o<=PEEP)
	    { 
				 
	     
while(breath_cycle==1)
					{		
				InspValve_Close();
				ExpValve_Close();
			  P_Online=F_gPCalc(AdcData[1]);
		pressure=P_Online-P_Offset;
	P_cmh2o1=pressure/gP_sensitivity;
	P_cmh2o=P_cmh2o1;
	P_USB=(int)P_cmh2o;
	P_UART=P_USB;
						_Flow();
			}
		}
			else
			{
					ExpValve_Open();
				InspValve_Close();
			
		}
		
	}	
}
			/////////////////////////////////////////////////////////////
		if(Data1.running_mode==SIMVVC)
				{
							if(breath_cycle==0)
			{
				blowerSpeed=(VOL/1200.00)*4095;
				HAL_DAC_SetValue(&hdac,DAC1_CHANNEL_1,DAC_ALIGN_12B_R,blowerSpeed);
	if(Volume>=VOL)
	    { 
					 reacedPIP=P_cmh2o;
				  reacedVolume=Volume;
		while(breath_cycle==0)
					{
				HAL_DAC_SetValue(&hdac,DAC1_CHANNEL_1,DAC_ALIGN_12B_R,0);
				InspValve_Close();
				ExpValve_Close();
		P_Online=F_gPCalc(AdcData[1]);
		pressure=P_Online-P_Offset;
P_cmh2o1=pressure/gP_sensitivity;
P_cmh2o=P_cmh2o1;
P_USB=(int)P_cmh2o;
P_UART=P_USB;
 _Flow();
			}
			}
			else
			{

				InspValve_Open();
				ExpValve_Close();
			}
		}
				if(breath_cycle==1)
		{
		if(P_cmh2o<=PEEP)
	    { 
				 
	        	reacedPEEP=P_cmh2o; 
while(breath_cycle==1)
					{		
				InspValve_Close();
				ExpValve_Close();
			  P_Online=F_gPCalc(AdcData[1]);
		pressure=P_Online-P_Offset;
	P_cmh2o1=pressure/gP_sensitivity;
	P_cmh2o=P_cmh2o1;
	P_USB=(int)P_cmh2o;
	P_UART=P_USB;
						_Flow();
			}
		}
			else
			{
					ExpValve_Open();
				InspValve_Close();
			
		}
		
	}
	}
	stabilize_count=4;
}
}
}
}
//	else if(vflag==0){
//		Volume=0;
//		vflag=1;
//	}		
 
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

 value1=Decode((char*)Rx_UART);
	switch(Flag)
	{
		case 'm':
			mode=value1;
			break;
		case 'r':
			RespiratoryRate=value1;
			break;
		case 'i':
			Iratio=value1;
			break;
   	case 'e':
			 Eratio=value1;
		if(Eratio<1)Eratio=1;
		break;
		case 'p':
			PEEP=value1;
			break;
		case 'q':
	  PIP=value1;
		if(PIP>=60)PIP=60;
		break;
		case 'c':
			VOL=value1;
		break;
		case 'o':
			FiO2=value1;
		break;
		case 'Z':
		  stop=value1;
		break;
		case 'C':
		  confirm=value1;
		break;
		default:
			break;
	}
	 Task_ReceivingPara();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2)
	  {
			if(debug==0)
			{
	breath_cycle^=1;
	if(breath_cycle==0)
	{
		cycle^=1;
	 if(Volume!=0)
	 {
	delp_flow=0;
	Volume=0;
	 }
 
  // GPIOC->ODR&=(~(1<<3));
	 //GPIOC->ODR|=(1<<2);
	  HAL_TIM_Base_Stop_IT(&htim2);
	  TIM2->ARR=Texp;
	  HAL_TIM_Base_Start_IT(&htim2);
   }
		if(breath_cycle==1)
		{

   //GPIOC->ODR|=(1<<3);
   //GPIOC->ODR&=(~(1<<2));
   Breath_update();
		}
		}
	stabilize_count++;
	//	if(stabilize_count>3)Volume=0;

		}
if (htim->Instance == TIM3)
	 {
		 Flow_Volume=(Flow2/60.00)*60.00;
		 Volume=Volume+Flow_Volume;
		 V_USB=(int)Volume;
		 V_UART= V_USB;
	 }
 }
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	//sprintf(Tx_UART_Sampled,"p%02dq%04dv%04d%d\n",(int)reacedPIP,(int)reacedPEEP,(int)reacedVolume,cycle);
	//HAL_UART_Transmit_DMA(&huart3,(uint8_t*)Tx_UART_Sampled,sizeof(Tx_UART_Sampled));	
}
 void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
// 	sprintf(Tx_UART_Regular,"P%02dQ%04dV%04d%d\n",(int)P_UART,(int)F_UART,(int)V_UART,cycle);
//	HAL_UART_Transmit_IT(&huart3,(uint8_t*)Tx_UART_Regular,sizeof(Tx_UART_Regular));
}


	
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
