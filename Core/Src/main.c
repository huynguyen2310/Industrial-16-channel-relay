/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modbus.h"
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define		CRC_START_MODBUS	0xFFFF
#define maxwriteregs 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//static uint16_t crc_tab16[256];
struct Device{
    uint8_t data;
    uint16_t address;
    uint8_t id;
    struct Device *next;
};
typedef struct Device modbusDigReg;
modbusDigReg *_digReg;
modbusDigReg *_lastReg;

struct Slave{
    uint8_t *msg;
    uint16_t len;
    uint16_t crc;

    modbusDigReg *_device;
};
typedef struct Slave modbusSlave;

		modbusSlave *base=NULL;
    modbusDigReg* head = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

modbusDigReg* searchNode(modbusDigReg *head,uint16_t key) //key = address
{
    modbusDigReg *temp = head;
    while(temp != NULL)
    {
         if(temp->address == key)
             return temp;
         temp = temp->next;
    }
    return 0;
}

uint8_t getID(modbusDigReg* head){
    return head->id;
}

void insertStart(modbusDigReg** head, uint32_t address){
		if(address<20000){
    modbusDigReg* newNode = (modbusDigReg*) malloc(sizeof(modbusDigReg));

    newNode->address = address;
		newNode->id = getID(*head);
    newNode->data = 0;
    newNode->next = 0;
	
			
		if(_digReg ==0){
			_digReg = newNode;
			_lastReg = _digReg;
		}
		else{
			_lastReg->next=newNode;
			_lastReg=newNode;
		}
    //changing the new head to this freshly entered node
    *head = _digReg;
		}
}
void setID(modbusDigReg* head, uint8_t id_device ){
    head->id=id_device;
}

void setValue(modbusDigReg* head, uint16_t address, uint16_t value){
    if(address<20000){
        modbusDigReg *temp;
        temp=searchNode(head,address);
        if(value) temp->data=0xFF;
        else temp->data=0x00;
    }
}

uint8_t getValue(modbusDigReg* head, uint16_t address){ 
    if(address<20000){
        modbusDigReg *temp;
        temp=searchNode(head,address);
        if(temp!=NULL) {
            return temp->data;
        }
        else return NULL;
    }
}

unsigned short int CRC16 (modbusSlave *slave, char *nData,unsigned short int wLength)
{
static const unsigned short int wCRCTable[] = {
0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

unsigned char nTemp;
unsigned short int wCRCWord = 0xFFFF;

   while (wLength--)
   {
      nTemp = *nData++ ^ wCRCWord;
      wCRCWord >>= 8;
      wCRCWord ^= wCRCTable[nTemp];
   }
   return wCRCWord;

}

void inStart(modbusSlave** slave, modbusDigReg **head){

    modbusSlave* newSla = (modbusSlave*) malloc(sizeof(modbusSlave));
		newSla->_device=*head;
		*slave=newSla;
}
	
void setup_Input(modbusSlave *slave){

		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)) setValue(slave->_device,10001,0x00);
		else setValue(slave->_device,10001,0xFF);
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)) setValue(slave->_device,10002,0x00);
		else setValue(slave->_device,10002,0xFF);
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)) setValue(slave->_device,10003,0x00);
		else setValue(slave->_device,10003,0xFF00);
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)) setValue(slave->_device,10004,0x00);
		else setValue(slave->_device,10004,0xFF00);
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)) setValue(slave->_device,10005,0x00);
		else setValue(slave->_device,10005,0xFF00);
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)) setValue(slave->_device,10006,0x00);
		else setValue(slave->_device,10006,0xFF00);
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_4)) setValue(slave->_device,10007,0x00);
		else setValue(slave->_device,10007,0xFF00);
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5)) setValue(slave->_device,10008,0x00);
		else setValue(slave->_device,10008,0xFF00);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)) setValue(slave->_device,10009,0x00);
		else setValue(slave->_device,10009,0xFF00);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)) setValue(slave->_device,10010,0x00);
		else setValue(slave->_device,10010,0xFF00);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)) setValue(slave->_device,10011,0x00);
		else setValue(slave->_device,10011,0xFF00);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)) setValue(slave->_device,10012,0x00);
		else setValue(slave->_device,10012,0xFF00);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10)) setValue(slave->_device,10013,0x00);
		else setValue(slave->_device,10013,0xFF00);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)) setValue(slave->_device,10014,0x00);
		else setValue(slave->_device,10014,0xFF00);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)) setValue(slave->_device,10015,0x00);
		else setValue(slave->_device,10015,0xFF00);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)) setValue(slave->_device,10016,0x00);
		else setValue(slave->_device,10016,0xFF00);

}
void sendData(modbusSlave *slave){
	
		slave->msg = (uint8_t*) malloc(sizeof(uint8_t)*8);
		slave->len = sizeof(uint8_t)*8;
		
    if(HAL_UART_Receive(&huart2, slave->msg, slave->len,200)==HAL_OK){
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
	}
}
uint8_t ver = 200;
void getVer(modbusSlave *slave, uint8_t funcType, uint16_t reg, uint16_t numReg){
		uint8_t func = reg>>8;
		slave->len = 7;
		slave->msg = (uint8_t*) malloc(sizeof(uint8_t)*slave->len);
		slave->msg[0] = getID(slave->_device);
		slave->msg[1] = funcType;
		slave->msg[2] = 0x02;
			switch(func){
				case 0x80:
					slave->msg[3] = ver>>8;
					slave->msg[4] = ver&0xff;
					break;
				case 0x40:
					slave->msg[3] = numReg>>8;
					slave->msg[4] = numReg&0xff;
			}

		uint16_t crc = CRC16(slave, (char*)slave->msg, slave->len-2);
		slave->msg[6] = crc >> 8;
		slave->msg[5] = crc & 0xFF;
}

uint16_t new_baud[8] = {48,96,192,384,576,1152,1280,2560};
void setBaud_Add(modbusSlave *slave, uint8_t funcType, uint16_t reg, uint16_t val){
		slave->len = 8;
		slave->msg = (uint8_t*) malloc(sizeof(uint8_t)*slave->len);
		slave->msg[0] = getID(slave->_device);
		slave->msg[1] = funcType;
		slave->msg[2] = reg>>8;
		slave->msg[3] = reg&0xff;
		slave->msg[4] = val>>8;
		slave->msg[5] = val&0xff;
		uint8_t new_id = slave->msg[5];
		switch(slave->msg[2]){
			case 0x20:
				HAL_UART_Abort_IT(&huart2);
				HAL_UART_DeInit(&huart2);
				huart2.Init.BaudRate = new_baud[(int)val]*100;
				if(HAL_UART_Init(&huart2)!=HAL_OK){
					Error_Handler();
				}
				break;
			case 0x40:
				while(slave->_device!=NULL){
					setID(slave->_device, new_id);
					slave->_device = slave->_device->next;
				}
				break;
		}
		uint16_t crc = CRC16(slave, (char*)slave->msg, slave->len-2);
		slave->msg[7] = crc >> 8;
		slave->msg[6] = crc & 0xFF;
}

void getDigitalStatus(modbusSlave *slave, uint8_t funcType, uint16_t startReg, uint16_t numReg){

    uint8_t bn =0;
    if(funcType==READ_DI) {
			startReg+=10001;
		}
    else {
			startReg+=1;
		}
    slave->len=numReg/8;
    if(numReg%8) slave->len++;
    slave->len+=5;
    slave->msg = (uint8_t*) malloc(sizeof(uint8_t)*slave->len);
    slave->msg[0]=getID(slave->_device);
    slave->msg[1]=funcType;
    slave->msg[2]=slave->len-5;
		switch (slave->msg[1]){
			case READ_DI:
					while (numReg--)
					{
									 if(getValue(slave->_device, startReg) && (bn<8)){
										
									 slave->msg[3] = slave->msg[3] &~ (1<<bn); //bitSet
									}
										else {  
									slave->msg[3] = slave->msg[3] | (1<<bn);  //bitClear
									}
										if(getValue(slave->_device, startReg) && (bn>=8)){
										
									 slave->msg[4] = slave->msg[4] &~ (1<<(bn-8)); //bitSet
									}
										else {  
									slave->msg[4] = slave->msg[4] | (1<<(bn-8));  //bitClear
									}
									bn++;                                     //increment the bit index
									startReg++;                               //increment the register
									}
					break;
			case READ_DO:
					while (numReg--)
					{				
									 if(getValue(slave->_device, startReg) && (bn<8)){
										
									 slave->msg[3] = slave->msg[3] | (1<<bn); //bitSet
									}
									else {  
									slave->msg[3] = slave->msg[3] &~ (1<<bn);  //bitClear
									}
									if(getValue(slave->_device, startReg) && (bn>=8)){
										
									 slave->msg[4] = slave->msg[4] | (1<<(bn-8)); //bitSet
									}
									else {  
									slave->msg[4] = slave->msg[4] &~ (1<<(bn-8));  //bitClear
									}
									bn++;                                     //increment the bit index
									startReg++;                               //increment the register
									}
					break;
					}
		slave->crc = CRC16(slave, (char*)slave->msg, slave->len-2);  
    slave->msg[slave->len-2] = slave->crc & 0xFF;
		slave->msg[slave->len-1] = slave->crc>>8;
}

void getStatus(modbusSlave *slave, uint8_t funcType, uint16_t reg, uint16_t val){
	slave->len=8;
	slave->msg = (uint8_t*) malloc(sizeof(uint8_t)*slave->len);
	
	if(funcType==WRITE_DO) {
		setValue(slave->_device,reg+1,val>>8);
		slave->msg[1] = WRITE_DO;
		
	}
	else{
		setValue(slave->_device,reg+40001,(val>>8));
		slave->msg[1] = WRITE_AO;
	}
	if(slave->msg[1] == WRITE_DO){
			slave->msg[0] = getID(slave->_device);
			slave->msg[2] = reg>>8;
			slave->msg[3] = reg&0xff;
			slave->msg[4] = val>>8;
			slave->msg[5] = val&0xff;
			uint16_t crc = CRC16(slave, (char*)slave->msg, slave->len-2);
			slave->msg[7] = crc >> 8;
			slave->msg[6] = crc & 0xFF;
			switch(slave->msg[3]){
				case 0x00:
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,slave->msg[4]&1);
				break;
				case 0x01:
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,slave->msg[4]&1);
				break;
				case 0x02:
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,slave->msg[4]&1);
				break;
				case 0x03:
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,slave->msg[4]&1);
				break;
				case 0x04:
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,slave->msg[4]&1);
				break;
				case 0x05:
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,slave->msg[4]&1);
				break;
				case 0x06:
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,slave->msg[4]&1);
				break;
				case 0x07:
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,slave->msg[4]&1);
				break;
				case 0x08:
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,slave->msg[4]&1);
				break;
				case 0x09:
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,slave->msg[4]&1);
				break;
				case 0x0A:
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,slave->msg[4]&1);
				break;
				case 0x0B:
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,slave->msg[4]&1);
				break;
				case 0x0C:
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,slave->msg[4]&1);
				break;
				case 0x0D:
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,slave->msg[4]&1);
				break;
				case 0x0E:
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,slave->msg[4]&1);
				break;
				case 0x0F:
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,slave->msg[4]&1);
				break;
			}
		}

//	else if (slave->msg[1] == WRITE_AO){	
//				deleteDevice(slave->_device);
//				free(slave->msg);
//				setup(val&0xff);
//				slave->_device=head;
//				slave->len=8;
//				slave->msg = (uint8_t*) malloc(sizeof(uint8_t)*slave->len);
//				 slave->msg[1] = WRITE_AO;
//				 slave->msg[2] = reg>>8;
//				 slave->msg[3] = reg&0xff;
//				 slave->msg[4] = val>>8;
//				 slave->msg[5] = val&0xff;
//				if(slave->msg[5] == getID(slave->_device)) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,1);
//				uint16_t crc = CRC16( slave, (char*) slave->msg,  slave->len-2);
//				 slave->msg[7] = crc >> 8;
//				 slave->msg[6] = crc & 0xFF;
//		}
}

void getStatus2(modbusSlave *slave, uint8_t funcType, uint16_t reg, uint16_t val){
	slave->len=8;
	if(funcType==WRITE_MULTIAO) {
		slave->msg[1] = WRITE_MULTIAO;
	}
	else{
		return;
	}
	slave->msg[2] = reg>>8;
	slave->msg[3] = reg&0xff;
	slave->msg[4] = val>>8;
	slave->msg[5] = val&0xff;
	uint16_t crc = CRC16(slave, (char*)slave->msg, slave->len-2);
  slave->msg[7] = crc >> 8;
  slave->msg[6] = crc & 0xFF;
}

void run(modbusSlave *slave){
    uint8_t funcT;
    uint16_t byte1, byte2, byte3;
    sendData(slave);
    funcT = slave->msg[1];
    byte1 = slave->msg[2] <<8 | slave->msg[3];
    byte2 = slave->msg[4] <<8 | slave->msg[5];
		if(funcT == WRITE_MULTIAO){
			if(byte2<maxwriteregs){;
				int st=0;
				for(int i=0;i<byte2;i++){
					byte3 = (slave->msg[7+st]<<8) | slave->msg[8+st];
					setValue(slave->_device, byte1 + 40001 + i, byte3);
					st+=2;
				}
			}
			else{
				return;
			}
		}
		free(slave->msg);
    slave->len=0;
    switch (funcT)
    {
    case READ_DI:
        getDigitalStatus(slave,funcT, byte1, byte2);
        break;
    case READ_DO:
        getDigitalStatus(slave,funcT, byte1, byte2);
        break;
		case WRITE_DO:
				getStatus(slave, funcT, byte1, byte2);
				break;
		case WRITE_AO:
				setBaud_Add(slave, funcT, byte1, byte2);
				break;
		case READ_AO:
        getVer(slave,funcT, byte1, byte2);
        break;
    default:
        break;
    }

    if(slave->len){
        HAL_UART_Transmit(&huart2, slave->msg, slave->len,200);
				free(slave->msg);
				return;
    }
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
//		setup(1);			//set id device
		insertStart(&head,1);
		setID(head, 1);
		for(int i=2;i<17; i++){
			insertStart(&head,i);
		}
		//Analog
		for(int i=1; i<17; i++){
			insertStart(&head,10000+i);
		}
		
		inStart(&base,&head);
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
					
		setup_Input(base);
    run(base);
	}
  /* USER CODE END 3 */
}

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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2
                          |GPIO_PIN_6|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC0 PC1 PC2
                           PC6 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2
                          |GPIO_PIN_6|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA5
                           PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB6
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

