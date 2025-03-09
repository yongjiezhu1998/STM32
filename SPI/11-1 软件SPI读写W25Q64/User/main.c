#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "W25Q64.h"

uint8_t MID;							//定义用于存放MID号的变量
uint16_t DID;							//定义用于存放DID号的变量

uint8_t ArrayWrite[] = {0x01, 0x02, 0x03, 0x04};	//定义要写入数据的测试数组
uint8_t ArrayRead[4];								//定义要读取数据的测试数组

int main(void)
{
	/*模块初始化*/
	OLED_Init();						//OLED初始化
	W25Q64_Init();
	
	W25Q64_ReadID(&MID, &DID);
	OLED_ShowHexNum(1, 1, MID, 2);	// 读厂商ID
	OLED_ShowHexNum(1, 8, DID, 4);	// 读设备ID
	
	while (1)
	{
		
	}
}
