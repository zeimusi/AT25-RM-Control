#include "Serialport.h"
void SerialportSend(enum SerialType Type);
void Vofa_JustFloat(float *Data, uint8_t Num);
unsigned char temp_end[4] = {0, 0, 0x80, 0x7F};

void Serialport_Task(){
     static portTickType currentTime;	
	   for(;;){
       vTaskSuspendAll();	
		    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)Serialport, (SerialChannel) * 4);
			  HAL_Delay(1);
        HAL_UART_Transmit(&huart1, temp_end,4,0x1ff);
	     xTaskResumeAll();
			 vTaskDelayUntil(&currentTime,20);		 
	   }

}

//typedef union
//{
//    float fdata;
//    unsigned long ldata;
//} FloatLongType;
//FloatLongType Data;
//uint8_t Tail[4] = {0x00, 0x00, 0x80, 0x7f};  //Êý¾Ý°üÎ²
//uint8_t SerialFloat[SerialChannel*4]; 
//void SerialportSend(enum SerialType Type){	
//			 Serialport[0] ++;
//	if(Type == FireWater){
//		 for(int i =0;i<SerialChannel-1;i++){
//	   printf("%f,",Serialport[i]);
//		 }
//     printf("%f\n",Serialport[SerialChannel-1]);
//	 }else if(Type == JustFloat){
//		Vofa_JustFloat(Serialport,4);
//	 }else if(Type == RawData){
//		 for(int i =0;i<SerialChannel;i++){
//	   printf("%f\n",Serialport[i]);
//		 HAL_UART_Transmit_DMA(&huart1,(uint8_t *)Serialport,sizeof(Serialport));
//		 }
//	 }
//}
