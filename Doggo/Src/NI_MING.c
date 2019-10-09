#include "main.h"
uint8_t TC=1;
extern UART_HandleTypeDef huart1;
uint8_t send_buf[21],send_buf_[18];
void Ni_Ming(uint8_t fun,float Pid_ref1,float Pid_ref2,float Pid_ref3,float Pid_ref4)
{
  unsigned char *p1,*p2,*p3,*p4;
  p1=(unsigned char *)&Pid_ref1;
	p2=(unsigned char *)&Pid_ref2;
	p3=(unsigned char *)&Pid_ref3;
	p4=(unsigned char *)&Pid_ref4;
	
	send_buf[0]=0XAA;	//帧头
	send_buf[1]=0XAA;	//帧头
	send_buf[2]=fun;	//功能字
	send_buf[3]=16;	//数据长度
  send_buf[4]=(unsigned char)(*(p1+3));
  send_buf[5]=(unsigned char)(*(p1+2));
  send_buf[6]=(unsigned char)(*(p1+1));
  send_buf[7]=(unsigned char)(*(p1+0));
	send_buf[8]=(unsigned char)(*(p2+3));
	send_buf[9]=(unsigned char)(*(p2+2));
	send_buf[10]=(unsigned char)(*(p2+1));
	send_buf[11]=(unsigned char)(*(p2+0));
	send_buf[12]=(unsigned char)(*(p3+3));
	send_buf[13]=(unsigned char)(*(p3+2));
	send_buf[14]=(unsigned char)(*(p3+1));
  send_buf[15]=(unsigned char)(*(p3+0));
	send_buf[16]=(unsigned char)(*(p4+3));
  send_buf[17]=(unsigned char)(*(p4+2));
  send_buf[18]=(unsigned char)(*(p4+1));
  send_buf[19]=(unsigned char)(*(p4+0));
	send_buf[20]=0;
	for(uint8_t i=0;i<20;i++)send_buf[20]+=send_buf[i];	//计算校验和
//	for(uint8_t i=0;i<21;i++)
//	{
//		while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)==RESET){}; 
//    USART1->DR=send_buf[i];
//	}
	HAL_UART_Transmit(&huart1,send_buf,21,10);
}
void serial_plot (uint8_t channels_num ,float Pid_ref1,float Pid_ref2,float Pid_ref3,float Pid_ref4)
{
  unsigned char *p1,*p2,*p3,*p4;
  p1=(unsigned char *)&Pid_ref1;
	p2=(unsigned char *)&Pid_ref2;
	p3=(unsigned char *)&Pid_ref3;
	p4=(unsigned char *)&Pid_ref4;
	
	send_buf_[0]=0XAA;//帧头
	send_buf_[1]=channels_num*4;
	if(channels_num==1||channels_num==2||channels_num==3||channels_num==4)
	{
		send_buf_[2]=(unsigned char)(*(p1+0));
		send_buf_[3]=(unsigned char)(*(p1+1));
		send_buf_[4]=(unsigned char)(*(p1+2));
		send_buf_[5]=(unsigned char)(*(p1+3));
	}
	if(channels_num==2||channels_num==3||channels_num==4)
	{
		send_buf_[6]=(unsigned char)(*(p2+0));
		send_buf_[7]=(unsigned char)(*(p2+1));
		send_buf_[8]=(unsigned char)(*(p2+2));
		send_buf_[9]=(unsigned char)(*(p2+3));
	}
	if(channels_num==3||channels_num==4)
	{
		send_buf_[10]=(unsigned char)(*(p3+0));
		send_buf_[11]=(unsigned char)(*(p3+1));
		send_buf_[12]=(unsigned char)(*(p3+2));
		send_buf_[13]=(unsigned char)(*(p3+3));
	}
	if(channels_num==4)
	{
		send_buf_[14]=(unsigned char)(*(p4+0));
		send_buf_[15]=(unsigned char)(*(p4+1));
		send_buf_[16]=(unsigned char)(*(p4+2));
		send_buf_[17]=(unsigned char)(*(p4+3));
	}
	HAL_UART_Transmit(&huart1,send_buf_,(channels_num*4)+2,10);
}