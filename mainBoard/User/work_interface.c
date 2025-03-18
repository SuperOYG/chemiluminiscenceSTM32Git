#include "work_interface.h"
#include "eeprom.h"

#include "can.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "motor.h"
#include "S_Moter1.h"
#include "S_Moter2.h"

//�жϼ�ʱ����
uint32_t time;

uint8_t R0, R1, R2, R3, R4, R5, R6, R7, R8, R9; //�������մ����·���ָ�
//����ͨѶ����
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

//��ǰʹ�õĴ���λ��
extern uint8_t CurUartNum;

extern uint8_t  RecieveBuffer[1];//�ݴ���յ����ַ�
extern uint8_t  Rx_end;  //ָ�������ɱ�־λ
extern uint8_t  RxLine;  //rxbuf���յ����ݳ���Ϊ:RxLine+1;
extern uint8_t  rxbuf[50];//�յ������ݴ�Ŵ�


//��λ��ִ��״̬��ʶλ���飺 ÿ�����ֽ�Ϊһ�飬��һ���ֽڱ�ʾ��ǰ���ڵڼ���ָ�����񣬵ڶ����ֽڱ�ʾָ��������״̬��   ��һ��Ϊ��ⵥԪ��2~4������1~3��������Ԫ��

uint8_t CmdFlagBuf[12] = {0xEE,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0xFF,0x0D,0x0A};

uint8_t TextBuffer[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x44, 0x55, 0x55};

//ͨѶʱ����������
uint8_t latstime=0;


////CANͨѶ������
extern CAN_TxHeaderTypeDef TXHeader;
extern CAN_RxHeaderTypeDef RXHeader;
extern uint8_t CanTXmessage[8];
extern uint8_t CanRXmessage[8];
extern uint32_t pTxMailbox;



//Can�������ݺ�����
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




// ADCת��ֵ
__IO uint32_t ADC_ConvertedValue;
// ���ڱ���ת�������ĵ�ѹֵ
float ADC_Vol;


/*
*  *********************************���ʹ�õ��м����*******************************
*/
u32 moter3_now;      //Z������ǰλ��
u32 moter3_next;     //Z������һ��λ��

u8 motor3Flag; //  Z������λ��־λ

u32 moter1_now;      //X������ǰλ��
u32 moter1_next;     //X������һ��λ��

u32 moter2_now;      //y������ǰλ��
u32 moter2_next;     //y������һ��λ��

u8 motor1Flag;//���1��λ��־λ��
u8 motor2Flag;//���2��λ��־λ��


//��ǰ�������Լ��Ŀ�����
uint8_t NumberOfCards;
//��ǰ����״̬
uint8_t testStatus;
//��ǰ���Ե��Լ��Ŀ������
uint8_t testIndex;

//ת���ƶ��ĳ�ʼ����
uint8_t StartStep;

//ת���ƶ��ļ������
uint8_t SingleStep;

//����״̬��־λ  1���п� 0���޿�
uint8_t card1_flag;
uint8_t card2_flag;
uint8_t card3_flag;
uint8_t card4_flag;
uint8_t card5_flag;
uint8_t card6_flag;


uint16_t Ka_now;  //��⾵ͷ��ǰλ��
uint16_t Ka_next;//��⾵ͷ��һ��λ��


float ADC_ConvertedValueLocal;


u8 state = 0;  //�ж��Ƿ��⵽��������Լ�������
extern u8 UART_Code[7]; //����ֵ


//���ű�ʶλ��
u8 shutterFlag;


//��λ���ӵ�Ԫ�ĵ�ǰ״̬��
u8 uint1_stateFlag,uint2_stateFlag,uint3_stateFlag;






//////////////////////////////////////////////
//����֡ͷ
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
//����֡β
/////////////////////////
void send_end()
{
    USART_SendByte(0xFF);
    USART_SendByte(0xFE);
}



void Open_Scanning(void)//��������
{

}

void Close_Scanning(void)//�ر�������
{

}

//��������
void u16ToHexArray(u16 num, u8* hexArray);


//������ʼ��
void init()
{
		moter2_int(200);
		moter1_int(200);
    printf("system init");
    HAL_Delay(10000);//ϵͳ������10s
    Read_equipment_parament(); // �����Ӱ���������
    for (int i = 1; i < 9; i++) {
        CmdFlagBuf[i] = 0x00;
    }
//    CmdFlagBuf[3]=0x00;
//		//��ȡ��������״̬��
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


void PC_test(void)    //PC�����
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
*   @brief: ��ȡ�����ܵĲ������ֵ
*   @param:  void
*   @return: void
*   @author Hang
*   @date: 20230410
*/

void getTestValue() {


}

//���յ�����CAN�ʹ��ڵ�ָ����з�������
void  getPraVue()
{
    uint8_t hexArray[4];
    uint32_t params;
    params = (R3 - 0x30) * 10000 + (R4 - 0x30) * 1000 + (R5 - 0x30) * 100 + (R6 - 0x30) * 10 + (R7 - 0x30);
    switch (R0)
    {
    case 0x01:                      //0x01:  ��������
        switch (R1)
        {
        case 0x00://�����λ
            if(R2==0x01) { //���Ƶ����λ
				if(moter2_now!=200){
					moter2_int(200);				
				}
                moter1_int(200);
            } else if(R2==0x02) { //��ֱ��������λ
                moter2_int(200);
            } else if(R2==0x00) {
                moter2_int(200);
                moter1_int(200);
            }
            break;
        case 0x01://����ƶ�
            if(R2==0x01) { //���Ƶ���ƶ�
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
    case 0xD3:  //�����ܲɼ�
        getTestValue();
        moter2_int(200);
        moter1_int(200);
        break;
    case 0x04: //���õ�Ԫ�������˶�����������
        params=(R4 - 0x30) * 1000 + (R5 - 0x30) * 100 + (R6 - 0x30) * 10 + (R7 - 0x30);
        if(R1==0x01) { //���õ�һ�����
            if(R2==0x01) { //����
                I2C_write2Byte(equipment_addres_1, params);
                I2C_writeByte(equipment_addres_1_1,R3);
                HAL_Delay(20);
                equipment_parament_1 = I2C_get2Byte(equipment_addres_1);
                printf("��һ����ֵ������111=%d\r\n",equipment_parament_1);
            } else if(R2==0x02) { //�Ű�
                I2C_write2Byte(equipment_addres_1+2, params);
                I2C_writeByte(equipment_addres_1_1+1,R3);
                HAL_Delay(10);
                equipment_parament_2 = I2C_get2Byte(equipment_addres_2);
                printf("��һ��Ű��������222=%d\r\n",equipment_parament_2);
            } else if(R2==0x03) { //��
                I2C_write2Byte(equipment_addres_1+4, params);
                I2C_writeByte(equipment_addres_1_1+2,R3);
                HAL_Delay(10);
                equipment_parament_3 = I2C_get2Byte(equipment_addres_3);
                printf("��һ���𵴵������333=%d\r\n",equipment_parament_3);
            }
        }
        else if(R1==0x02) { //���õڶ������
            if(R2==0x01) { //����
                I2C_write2Byte(equipment_addres_1+6, params);
                I2C_writeByte(equipment_addres_1_1+3,R3);
                HAL_Delay(10);
                equipment_parament_4 = I2C_get2Byte(equipment_addres_4);
                printf("�ڶ�����ֵ������444=%d\r\n",equipment_parament_4);
            } else if(R2==0x02) { //�Ű�
                I2C_write2Byte(equipment_addres_1+8, params);
                I2C_writeByte(equipment_addres_1_1+4,R3);
                HAL_Delay(10);
                equipment_parament_5 = I2C_get2Byte(equipment_addres_5);
                printf("�ڶ���Ű��������555=%d\r\n",equipment_parament_5);
            } else if(R2==0x03) { //��
                I2C_write2Byte(equipment_addres_1+10, params);
                I2C_writeByte(equipment_addres_1_1+5,R3);
                HAL_Delay(10);
                equipment_parament_6 = I2C_get2Byte(equipment_addres_6);
                printf("�ڶ����𵴵������666=%d\r\n",equipment_parament_6);
            }
        } else if(R1==0x03) {
            if(R2==0x01) { //����
                I2C_write2Byte(equipment_addres_1+12, params);
                I2C_writeByte(equipment_addres_1_1+6,R3);
                HAL_Delay(10);
                equipment_parament_7 = I2C_get2Byte(equipment_addres_7);
                printf("��������ֵ������77=%d\r\n",equipment_parament_7);
            } else if(R2==0x02) { //�Ű�
                I2C_write2Byte(equipment_addres_1+14, params);
                I2C_writeByte(equipment_addres_1_1+7,R3);
                HAL_Delay(10);
                equipment_parament_8 = I2C_get2Byte(equipment_addres_8);
                printf("������Ű��������888=%d\r\n",equipment_parament_8);
            } else if(R2==0x03) { //��
                I2C_write2Byte(equipment_addres_1+16, params);
                I2C_writeByte(equipment_addres_1_1+8,R3);
                HAL_Delay(10);
                equipment_parament_9 = I2C_get2Byte(equipment_addres_9);
                printf("�������𵴵������999=%d\r\n",equipment_parament_9);
            }
        }
        Read_equipment_parament();
        break;

    case 0x05: //��ȡ�ӿ��ư� ����ִ��״̬��

        //֡ͷ
        Usart2_SendByte(0xEE);
        if(R1==0x01) {

            if(HAL_GPIO_ReadPin(Can_Signal_1_GPIO_Port,Can_Signal_1_Pin)==1) { //1�Ű�
                Usart2_SendByte(0xAA);
                Usart2_SendByte(0x01);
                Usart2_SendByte(0xBB);
            } else {
                Usart2_SendByte(0xCC);
                Usart2_SendByte(0x01);
                Usart2_SendByte(0xDD);
            }

        } else if(R1==0x02) {
            if(HAL_GPIO_ReadPin(Can_Signal_2_GPIO_Port,Can_Signal_2_Pin)==1) { //2�Ű�
                Usart2_SendByte(0xAA);
                Usart2_SendByte(0x02);
                Usart2_SendByte(0xBB);
            } else {
                Usart2_SendByte(0xCC);
                Usart2_SendByte(0x02);
                Usart2_SendByte(0xDD);
            }

        } else if(R1==0x03) {
            if(HAL_GPIO_ReadPin(Can_Signal_3_GPIO_Port,Can_Signal_3_Pin)==1) { //3�Ű�
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
    case 0x06: //���Ƶ�ŷ� 
			  if(R1==0x01){ //����
					HAL_GPIO_WritePin(shutter_GPIO_Port,shutter_Pin,GPIO_PIN_SET);
					shutterFlag=1;
//					HAL_Delay(5000); //5S��ر�
				}else if(R1==0x00){//�ر�
					HAL_GPIO_WritePin(shutter_GPIO_Port,shutter_Pin,GPIO_PIN_RESET);
					shutterFlag=2;
				}
				break;
		case 0x07://����PMT����	
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

//������λ��ָ�����λ��ָ�����Ԥ����
void receiveCmd()
{
    if (rxbuf[1] == 0x00) //ָ��Ϊ��CPU����ָ��
    {
        if(CmdFlagBuf[1]>250) {
            CmdFlagBuf[1]=0x00;
        }
        CmdFlagBuf[1]=CmdFlagBuf[1]+1;   //�������BUG�� һ����λ������λ������ָ�����λ�ظ�ָ�����λ��δ���յ�ʱ��
										 //�������·���ʱ����ʱ��λ������ָ��ID�Ż��ٴ����ӡ�
										 //�����ͻᵼ����λ���ش���ָ�����λ����ǰ���͵�ָ����Զ�����ٴ�ƥ���ϡ�
        CmdFlagBuf[2]=0x01;
        HAL_UART_Transmit(&huart2,CmdFlagBuf,12,1000);   //����Ӧ��
        HAL_Delay(50);
		HAL_UART_Transmit(&huart2,CmdFlagBuf,12,1000);   //����Ӧ��
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
		HAL_UART_Transmit(&huart2,CmdFlagBuf,12,1000);	//ִ��Ӧ��
        HAL_Delay(50);
    } 
	else if(rxbuf[1]==0x10) { //���״̬��־λ
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
    } else if(rxbuf[1]==0x11) { //��ѯ��λ��״̬��ʶλ��
        HAL_UART_Transmit(&huart2,CmdFlagBuf,12,1000);	//ִ��Ӧ��
        HAL_Delay(50);
    }
    else
    {
        for (int i = 0; i < 8; i++)
        {
            TextBuffer[i] = rxbuf[i + 2];
        }
        if (rxbuf[1] == 0x01) //���͸�1���Ӱ�
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
        else if (rxbuf[1] == 0x02)  //���͸�2���Ӱ�
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
        else if (rxbuf[1] == 0x03)  //���͸�3���Ӱ�
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

//����CAN ����֡��Ϣ�� 1.�����ܵĲ������ݣ�2.������¶�����
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
	if(RXHeader.StdId==0x15){ //���������� 
		if(R0==0xD3&&R1==0x72){
		uint32_t countResult = (CanRXmessage[4] << 24) + (CanRXmessage[5] << 16) + (CanRXmessage[6] << 8) + CanRXmessage[7];			
		//ͨ�����ڽ����ݷ��ء�
		buf[0]=0xEE;
		buf[1]=0x01; //ָ����
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
	}else { //�¶�����
//			uint32_t params = (R3 - 0x30) * 10000 + (R4 - 0x30) * 1000 + (R5 - 0x30) * 100 + (R6 - 0x30) * 10 + (R7 - 0x30);
//		  buf[0]=0xEE;
//			buf[1]=0x02; //ָ����
//			buf[2]=subId; //�ڼ���
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
//		printf("�յ�����ָ��");
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
		else if(RXHeader.StdId == 0x15){ //�յ������ܶ�����
			getCanData();	
		}
        Rx_end = 0;
    }
}

//��ʱ���ص�����
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */
    static uint32_t num = 0;
    static uint32_t num1 = 0;
    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM2) // ��ʱ��2����ַ
    {
        // �Զ���Ӧ�ó���
        time++;
        num++;
        if (num == 100) // ÿ1��LED�Ʒ�תһ��
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
						if(num1==10000){		//���õ�����������ʱ�䣬�Է������ջ١�
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
*   @brief: �� u16�� ���� ת���� 5���ֽڵ�hex�����飬����ͨ��CAN ���ݸ�ʽ���·��͡�
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
        hexArray[i] += (hexArray[i] < 10) ? 0x30 : 0x37;  // ת��Ϊ��Ӧ��ASCII��
    }
}


