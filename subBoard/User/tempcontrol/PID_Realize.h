
typedef struct realizePID
{ 
  float P,I,D,I_limit, PWMLIMITE;
}realizePID;
  
typedef struct realizeError
{
  float Current_Error;
  float Last_Error;
	float Accu_Error;
}realizeError;
 
float PID_Realize( realizeError *error, realizePID *pid);
///* 
// *  @brief         λ��ʽPID
// *  @since         v1.0
// *  @error         ������
// *  @pid           PID����
// *  @curPlace      ��ǰλ��
// *  @tarPoint      Ŀ��λ�� 
// */
//float PID_Realize( realizeError *error, realizePID *pid){
// 
//	int PWM;    //ʵ�����                                              
//	
//	error->Accu_Error +=  error->Current_Error;
// 
//	error->Accu_Error = error->Accu_Error > pid->I_limit ?  pid->I_limit:error->Accu_Error;//�����޷�
//  error->Accu_Error = error->Accu_Error < -pid->I_limit ? -pid->I_limit:error->Accu_Error;
//	
//	PWM =     pid->P * error->Current_Error                           //����P
//			      + pid->D * (error->Current_Error - error->Last_Error)   //΢��D
//            + pid->I * error->Accu_Error;;                          //����I
//	
//	error->Last_Error = error->Current_Error;		  	                  // �����ϴ����
//	if(PWM > pid->PWMLIMITE)  PWM = pid->PWMLIMITE;
//	if(PWM < -pid->PWMLIMITE) PWM = -pid->PWMLIMITE;
//	return PWM;	// ����ʵ��ֵ 
//}
