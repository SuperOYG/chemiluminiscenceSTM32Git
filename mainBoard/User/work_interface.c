#include "work_interface.h"
#include "eeprom.h"

#include "can.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "motor.h"
#include "S_Moter1.h"
#include "S_Moter2.h"

//中断计时变量
uint32_t time;

uint8_t R0, R1, R2, R3, R4, R5, R6, R7, R8, R9; //用来接收串口下发的指令。
//串口通讯变量
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

//当前使用的串口位；
extern uint8_t CurUartNum;

extern uint8_t  RecieveBuffer[1];//暂存接收到的字符
extern uint8_t  Rx_end;  //指令接收完成标志位
extern uint8_t  RxLine;  //rxbuf接收的数据长度为:RxLine+1;
extern uint8_t  rxbuf[50];//收到的数据存放处


//下位机执行状态标识位数组： 每两个字节为一组，第一个字节表示当前处于第几个指令任务，第二个字节表示指令所处的状态。   第一组为检测单元，2~4组属于1~3个动作单元。

uint8_t CmdFlagBuf[12] = {0xEE,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0xFF,0x0D,0x0A};

uint8_t TextBuffer[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x44, 0x55, 0x55};

//通讯时间间隔变量：
uint8_t latstime=0;


////CAN通讯变量：
extern CAN_TxHeaderTypeDef TXHeader;
extern CAN_RxHeaderTypeDef RXHeader;
extern uint8_t CanTXmessage[8];
extern uint8_t CanRXmessage[8];
extern uint32_t pTxMailbox;



//Can发送数据函数：
void CAN_senddata(CAN_HandleTypeDef *hcan, uint8_t Data[8], u32 id)
{
    TXHeader.StdId = id;
    TXHeader.ExtId = 0x12345000;
    TXHeader.DLC = 8;
    TXHeader.IDE = CAN_ID_STD;
    TXHeader.RTR = CAN_RTR_DATA;
    TXHeader.TransmitGlobalTime = DISABLE;
    HAL_CAN_AddTxMessage(hcan, &TXHeader, Data, &pTxMailbox);
}




// ADC转换值
__IO uint32_t ADC_ConvertedValue;
// 用于保存转换计算后的电压值
float ADC_Vol;


/*
*  *********************************检测使用的中间变量*******************************
*/
u32 moter3_now;      //Z轴电机当前位置
u32 moter3_next;     //Z轴电机下一步位置

u8 motor3Flag; //  Z轴电机复位标志位

u32 moter1_now;      //X轴电机当前位置
u32 moter1_next;     //X轴电机下一步位置

u32 moter2_now;      //y轴电机当前位置
u32 moter2_next;     //y轴电机下一步位置

u8 motor1Flag;//电机1复位标志位：
u8 motor2Flag;//电机2复位标志位：


//当前被插入试剂的卡槽数
uint8_t NumberOfCards;
//当前机器状态
uint8_t testStatus;
//当前测试的试剂的卡槽序号
uint8_t testIndex;

//转盘移动的初始距离
uint8_t StartStep;

//转盘移动的间隔距离
uint8_t SingleStep;

//卡槽状态标志位  1：有卡 0：无卡
uint8_t card1_flag;
uint8_t card2_flag;
uint8_t card3_flag;
uint8_t card4_flag;
uint8_t card5_flag;
uint8_t card6_flag;


uint16_t Ka_now;  //检测镜头当前位置
uint16_t Ka_next;//检测镜头下一步位置


float ADC_ConvertedValueLocal;


u8 state = 0;  //判断是否检测到带条码的试剂卡进入
extern u8 UART_Code[7]; //条码值


//快门标识位：
u8 shutterFlag;


//下位机子单元的当前状态：
u8 uint1_stateFlag,uint2_stateFlag,uint3_stateFlag;






//////////////////////////////////////////////
//发送帧头
/////////////////////////
void USART_SendByte(uint8_t data)
{
    if (CurUartNum == 2)
    {
        Usart2_SendByte(data);
    }
    else if (CurUartNum == 3)
    {
        Usart3_SendByte(data);
    }

}

void send_front()
{

    USART_SendByte(0xAA);
    USART_SendByte(0x55);

}
//////////////////////////////////////////////
//发送帧尾
/////////////////////////
void send_end()
{
    USART_SendByte(0xFF);
    USART_SendByte(0xFE);
}



void Open_Scanning(void)//打开条码器
{

}

void Close_Scanning(void)//关闭条码器
{

}

//函数申明
void u16ToHexArray(u16 num, u8* hexArray);


//参数初始化
void init()
{
		moter2_int(200);
		moter1_int(200);
    printf("system init");
    HAL_Delay(10000);//系统晚启动10s
    Read_equipment_parament(); // 发送子板电机参数。
    for (int i = 1; i < 9; i++) {
        CmdFlagBuf[i] = 0x00;
    }
//    CmdFlagBuf[3]=0x00;
//		//获取机器开机状态。
//		uint1_stateFlag=I2C_getByte(Unit1_Status_addres);
//		uint2_stateFlag=I2C_getByte(Unit2_Status_addres);
//		uint3_stateFlag=I2C_getByte(Unit3_Status_addres);
	
//		uint8_t buf1[8] = {0x06, uint1_stateFlag, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//			CAN_senddata(&hcan, buf1, 0x01);

//		uint8_t buf2[8] = {0x06, uint2_stateFlag, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//			CAN_senddata(&hcan, buf2, 0x02);
//		uint8_t buf3[8] = {0x06, uint3_stateFlag, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//			CAN_senddata(&hcan, buf3, 0x03);
}


void PC_test(void)    //PC机检测
{

}


void getPosition()
{

}


void setPosition(u8 n)
{
    u16 params = (R3 - 0x30) * 10000 + (R4 - 0x30) * 1000 + (R5 - 0x30) * 100 + (R6 - 0x30) * 10 + (R7 - 0x30);
    if (n == 0x05)
    {
        I2C_write2Byte(equipment_addres_5, params);
        equipment_parament_5 = I2C_get2Byte(equipment_addres_5);
    }
    else if (n == 0x06)
    {
        I2C_write2Byte(equipment_addres_6, params);
        equipment_parament_6 = I2C_get2Byte(equipment_addres_6);
    }
}



/**
*   @brief: 获取倍增管的测量结果值
*   @param:  void
*   @return: void
*   @author Hang
*   @date: 20230410
*/

void getTestValue() {


}

//对收到来自CAN和串口的指令进行分析处理
void  getPraVue()
{
    uint8_t hexArray[4];
    uint32_t params;
    params = (R3 - 0x30) * 10000 + (R4 - 0x30) * 1000 + (R5 - 0x30) * 100 + (R6 - 0x30) * 10 + (R7 - 0x30);
    switch (R0)
    {
    case 0x01:                      //0x01:  单步调试
        switch (R1)
        {
        case 0x00://电机复位
            if(R2==0x01) { //横移电机复位
				if(moter2_now!=200){
					moter2_int(200);				
				}
                moter1_int(200);
            } else if(R2==0x02) { //竖直方向电机复位
                moter2_int(200);
            } else if(R2==0x00) {
                moter2_int(200);
                moter1_int(200);
            }
            break;
        case 0x01://电机移动
            if(R2==0x01) { //横移电机移动
				if(moter2_now!=200){
					moter2_int(200);				
				}
				 if(params>6765){
					params=6765;
				 }			
                moter1_step(params,0);
            } else if(R2==0x02) {
				  if(params>2000){
					 params=2000;
				  }
                moter2_step(params,0);
            }
            break;
        }
        break;
    case 0xD3:  //倍增管采集
        getTestValue();
        moter2_int(200);
        moter1_int(200);
        break;
    case 0x04: //设置单元组电机的运动补偿参数：
        params=(R4 - 0x30) * 1000 + (R5 - 0x30) * 100 + (R6 - 0x30) * 10 + (R7 - 0x30);
        if(R1==0x01) { //设置第一组参数
            if(R2==0x01) { //出仓
                I2C_write2Byte(equipment_addres_1, params);
                I2C_writeByte(equipment_addres_1_1,R3);
                HAL_Delay(20);
                equipment_parament_1 = I2C_get2Byte(equipment_addres_1);
                printf("第一组出仓电机补偿111=%d\r\n",equipment_parament_1);
            } else if(R2==0x02) { //磁棒
                I2C_write2Byte(equipment_addres_1+2, params);
                I2C_writeByte(equipment_addres_1_1+1,R3);
                HAL_Delay(10);
                equipment_parament_2 = I2C_get2Byte(equipment_addres_2);
                printf("第一组磁棒电机补偿222=%d\r\n",equipment_parament_2);
            } else if(R2==0x03) { //震荡
                I2C_write2Byte(equipment_addres_1+4, params);
                I2C_writeByte(equipment_addres_1_1+2,R3);
                HAL_Delay(10);
                equipment_parament_3 = I2C_get2Byte(equipment_addres_3);
                printf("第一组震荡电机补偿333=%d\r\n",equipment_parament_3);
            }
        }
        else if(R1==0x02) { //设置第二组参数
            if(R2==0x01) { //出仓
                I2C_write2Byte(equipment_addres_1+6, params);
                I2C_writeByte(equipment_addres_1_1+3,R3);
                HAL_Delay(10);
                equipment_parament_4 = I2C_get2Byte(equipment_addres_4);
                printf("第二组出仓电机补偿444=%d\r\n",equipment_parament_4);
            } else if(R2==0x02) { //磁棒
                I2C_write2Byte(equipment_addres_1+8, params);
                I2C_writeByte(equipment_addres_1_1+4,R3);
                HAL_Delay(10);
                equipment_parament_5 = I2C_get2Byte(equipment_addres_5);
                printf("第二组磁棒电机补偿555=%d\r\n",equipment_parament_5);
            } else if(R2==0x03) { //震荡
                I2C_write2Byte(equipment_addres_1+10, params);
                I2C_writeByte(equipment_addres_1_1+5,R3);
                HAL_Delay(10);
                equipment_parament_6 = I2C_get2Byte(equipment_addres_6);
                printf("第二组震荡电机补偿666=%d\r\n",equipment_parament_6);
            }
        } else if(R1==0x03) {
            if(R2==0x01) { //出仓
                I2C_write2Byte(equipment_addres_1+12, params);
                I2C_writeByte(equipment_addres_1_1+6,R3);
                HAL_Delay(10);
                equipment_parament_7 = I2C_get2Byte(equipment_addres_7);
                printf("第三组出仓电机补偿77=%d\r\n",equipment_parament_7);
            } else if(R2==0x02) { //磁棒
                I2C_write2Byte(equipment_addres_1+14, params);
                I2C_writeByte(equipment_addres_1_1+7,R3);
                HAL_Delay(10);
                equipment_parament_8 = I2C_get2Byte(equipment_addres_8);
                printf("第三组磁棒电机补偿888=%d\r\n",equipment_parament_8);
            } else if(R2==0x03) { //震荡
                I2C_write2Byte(equipment_addres_1+16, params);
                I2C_writeByte(equipment_addres_1_1+8,R3);
                HAL_Delay(10);
                equipment_parament_9 = I2C_get2Byte(equipment_addres_9);
                printf("第三组震荡电机补偿999=%d\r\n",equipment_parament_9);
            }
        }
        Read_equipment_parament();
        break;

    case 0x05: //获取子控制板 命令执行状态：

        //帧头
        Usart2_SendByte(0xEE);
        if(R1==0x01) {

            if(HAL_GPIO_ReadPin(Can_Signal_1_GPIO_Port,Can_Signal_1_Pin)==1) { //1号板
                Usart2_SendByte(0xAA);
                Usart2_SendByte(0x01);
                Usart2_SendByte(0xBB);
            } else {
                Usart2_SendByte(0xCC);
                Usart2_SendByte(0x01);
                Usart2_SendByte(0xDD);
            }

        } else if(R1==0x02) {
            if(HAL_GPIO_ReadPin(Can_Signal_2_GPIO_Port,Can_Signal_2_Pin)==1) { //2号板
                Usart2_SendByte(0xAA);
                Usart2_SendByte(0x02);
                Usart2_SendByte(0xBB);
            } else {
                Usart2_SendByte(0xCC);
                Usart2_SendByte(0x02);
                Usart2_SendByte(0xDD);
            }

        } else if(R1==0x03) {
            if(HAL_GPIO_ReadPin(Can_Signal_3_GPIO_Port,Can_Signal_3_Pin)==1) { //3号板
                Usart2_SendByte(0xAA);
                Usart2_SendByte(0x03);
                Usart2_SendByte(0xBB);
            } else {
                Usart2_SendByte(0xCC);
                Usart2_SendByte(0x03);
                Usart2_SendByte(0xDD);
            }
        }
        Usart2_SendByte(0x0D);
        Usart2_SendByte(0x0A);
        break;
    case 0x06: //控制电磁阀 
			  if(R1==0x01){ //开启
					HAL_GPIO_WritePin(shutter_GPIO_Port,shutter_Pin,GPIO_PIN_SET);
					shutterFlag=1;
//					HAL_Delay(5000); //5S后关闭
				}else if(R1==0x00){//关闭
					HAL_GPIO_WritePin(shutter_GPIO_Port,shutter_Pin,GPIO_PIN_RESET);
					shutterFlag=2;
				}
				break;
		case 0x07://进行PMT读数	
				    HAL_GPIO_WritePin(shutter_GPIO_Port,shutter_Pin,GPIO_PIN_SET);	
					shutterFlag=1;
					HAL_Delay(1000);
					uint8_t sendBuffer[8] = {0xD3, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
					CAN_senddata(&hcan, sendBuffer, 0xFF);
					break;
        default:
        break;
    }
}

//接收上位机指令：对上位机指令进行预处理
void receiveCmd()
{
    if (rxbuf[1] == 0x00) //指令为主CPU处理指令
    {
        if(CmdFlagBuf[1]>250) {
            CmdFlagBuf[1]=0x00;
        }
        CmdFlagBuf[1]=CmdFlagBuf[1]+1;   //这里产生BUG。 一旦上位机上下位机发送指令后，下位回复指令而上位机未接收到时，
										 //进行重新发送时，此时下位机数据指令ID号会再次增加。
										 //这样就会导致下位机回传的指令和上位机当前发送的指令永远不会再次匹配上。
        CmdFlagBuf[2]=0x01;
        HAL_UART_Transmit(&huart2,CmdFlagBuf,12,1000);   //接收应答
        HAL_Delay(50);
		HAL_UART_Transmit(&huart2,CmdFlagBuf,12,1000);   //接收应答
        HAL_Delay(50);
        R0 = rxbuf[2];
        R1 = rxbuf[3];
        R2 = rxbuf[4];
        R3 = rxbuf[5];
        R4 = rxbuf[6];
        R5 = rxbuf[7];
        R6 = rxbuf[8];
        R7 = rxbuf[9];
        getPraVue();
        CmdFlagBuf[2]=0x02;
		HAL_Delay(50);
		HAL_UART_Transmit(&huart2,CmdFlagBuf,12,1000);	//执行应答
        HAL_Delay(50);
    } 
	else if(rxbuf[1]==0x10) { //清除状态标志位
        if(rxbuf[2]==0x00) {
            CmdFlagBuf[1]=0x00;
            CmdFlagBuf[2]=0x00;
        } else if(rxbuf[2]==0x01) {
            CmdFlagBuf[3]=0x00;
            CmdFlagBuf[4]=0x00;
        } else if(rxbuf[2]==0x02) {
            CmdFlagBuf[5]=0x00;
            CmdFlagBuf[6]=0x00;
        } else if(rxbuf[2]==0x03) {
            CmdFlagBuf[7]=0x00;
            CmdFlagBuf[8]=0x00;
        } else if(rxbuf[2]==0x04) {
            for (int i = 1; i <= 8; i++) {
                CmdFlagBuf[i] = 0x00;
            }
        }
    } else if(rxbuf[1]==0x11) { //查询下位机状态标识位：
        HAL_UART_Transmit(&huart2,CmdFlagBuf,12,1000);	//执行应答
        HAL_Delay(50);
    }
    else
    {
        for (int i = 0; i < 8; i++)
        {
            TextBuffer[i] = rxbuf[i + 2];
        }
        if (rxbuf[1] == 0x01) //发送给1号子板
        {
            if(CmdFlagBuf[3]>250) {
                CmdFlagBuf[3]=0x00;
            }
            CmdFlagBuf[3]=CmdFlagBuf[3]+1;
            CmdFlagBuf[4]=0x01;
            HAL_UART_Transmit(&huart2,CmdFlagBuf,12,1000);
            HAL_Delay(50);
            CAN_senddata(&hcan, TextBuffer, 0x01);
        }
        else if (rxbuf[1] == 0x02)  //发送给2号子板
        {
            if(CmdFlagBuf[5]>250) {
                CmdFlagBuf[5]=0x00;
            }
            CmdFlagBuf[5]=CmdFlagBuf[5]+1;
            CmdFlagBuf[6]=0x01;
            HAL_UART_Transmit(&huart2,CmdFlagBuf,12,1000);
            HAL_Delay(50);
            CAN_senddata(&hcan, TextBuffer, 0x02);
        }
        else if (rxbuf[1] == 0x03)  //发送给3号子板
        {
            if(CmdFlagBuf[7]>250) {
                CmdFlagBuf[7]=0x00;
            }
            CmdFlagBuf[7]=CmdFlagBuf[7]+1;
            CmdFlagBuf[8]=0x01;
            HAL_UART_Transmit(&huart2,CmdFlagBuf,12,1000);
            HAL_Delay(50);
            CAN_senddata(&hcan, TextBuffer, 0x03);
        }
    }
}

//解析CAN 数据帧消息： 1.倍增管的测试数据，2.各组的温度数据
void getCanData() {
	
	R0 = CanRXmessage[0];
    R1 = CanRXmessage[1];
    R2 = CanRXmessage[2];
    R3 = CanRXmessage[3];
    R4 = CanRXmessage[4];
    R5 = CanRXmessage[5];
    R6 = CanRXmessage[6];
    R7 = CanRXmessage[7];

	uint8_t buf[13];
	if(RXHeader.StdId==0x15){ //倍增管数据 
		if(R0==0xD3&&R1==0x72){
		uint32_t countResult = (CanRXmessage[4] << 24) + (CanRXmessage[5] << 16) + (CanRXmessage[6] << 8) + CanRXmessage[7];			
		//通过串口将数据返回。
		buf[0]=0xEE;
		buf[1]=0x01; //指令码
		buf[10]=0xFF;
		buf[11]=0x0D;
		buf[12]=0x0A; 
		for(int i=0;i<8;i++){
			buf[i+2]=CanRXmessage[i];
		}
			HAL_Delay(50);
			HAL_UART_Transmit(&huart2,buf,13,1000);
			HAL_Delay(50);
		}
	}else { //温度数据
//			uint32_t params = (R3 - 0x30) * 10000 + (R4 - 0x30) * 1000 + (R5 - 0x30) * 100 + (R6 - 0x30) * 10 + (R7 - 0x30);
//		  buf[0]=0xEE;
//			buf[1]=0x02; //指令码
//			buf[2]=subId; //第几组
//			buf[10]=0xFF;
//			buf[11]=0x0D;
//			buf[12]=0x0A;
//			
//			for(int i=0;i<9;i++){
//				buf[i+2]=CanRXmessage[i];
//			}
//				HAL_Delay(50);
//			 HAL_UART_Transmit(&huart2,buf,13,1000);
//			HAL_Delay(200);
	}
	
}





void test()
{
    if (Rx_end == 1)
    {
        receiveCmd();
        Rx_end = 0;
//		printf("收到串口指令");
    }
    else if (Rx_end == 2)
    {
        if (RXHeader.StdId == 0x11)
        {
            getCanData();
        }
        else if (RXHeader.StdId == 0x12)
        {
            getCanData();
        }
        else if (RXHeader.StdId == 0x13)
        {
            getCanData();
        }
		else if(RXHeader.StdId == 0x15){ //收到倍增管读数。
			getCanData();	
		}
        Rx_end = 0;
    }
}

//定时器回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */
    static uint32_t num = 0;
    static uint32_t num1 = 0;
    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM2) // 定时器2基地址
    {
        // 自定义应用程序
        time++;
        num++;
        if (num == 100) // 每1秒LED灯翻转一次
        {
            if (motor1Flag == 1)
            {
                EN1_H;
                motor1Flag++;
            }
            else if (motor1Flag == 2)
            {
                EN1_L;
            }

            if (motor2Flag == 1)
            {
                EN2_H;
                motor2Flag++;
            }
            else if (motor2Flag == 2)
            {
                EN2_L;
            }
            num = 0;
        }
				if(shutterFlag==1){
						num1++;
						if(num1==10000){		//设置电磁铁的最大工作时间，以防过热烧毁。
						HAL_GPIO_WritePin(shutter_GPIO_Port,shutter_Pin,GPIO_PIN_RESET);
								shutterFlag=2;
							num1=0;
				}
				}
    }
    else if (htim->Instance == TIM4)
    {
        TIM4Fun();
    }
    else if (htim->Instance == TIM3)
    {
        TIM3_Fun();
    }
}
/**
*   @brief: 将 u16型 数据 转换成 5个字节的hex型数组，便于通过CAN 数据格式往下发送。
*   @param:  void
*   @return: void
*   @author Hang
*   @date: 2023-05-20
*/

void u16ToHexArray(u16 num, u8* hexArray)
{
    int i;
    for (i = 0; i < 4; i++)
    {
        hexArray[i] = (num >> (12 - 4 * i)) & 0x0F;
        hexArray[i] += (hexArray[i] < 10) ? 0x30 : 0x37;  // 转换为对应的ASCII码
    }
}


