
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
// *  @brief         位置式PID
// *  @since         v1.0
// *  @error         误差参数
// *  @pid           PID参数
// *  @curPlace      当前位置
// *  @tarPoint      目标位置 
// */
//float PID_Realize( realizeError *error, realizePID *pid){
// 
//	int PWM;    //实际输出                                              
//	
//	error->Accu_Error +=  error->Current_Error;
// 
//	error->Accu_Error = error->Accu_Error > pid->I_limit ?  pid->I_limit:error->Accu_Error;//积分限幅
//  error->Accu_Error = error->Accu_Error < -pid->I_limit ? -pid->I_limit:error->Accu_Error;
//	
//	PWM =     pid->P * error->Current_Error                           //比例P
//			      + pid->D * (error->Current_Error - error->Last_Error)   //微分D
//            + pid->I * error->Accu_Error;;                          //积分I
//	
//	error->Last_Error = error->Current_Error;		  	                  // 更新上次误差
//	if(PWM > pid->PWMLIMITE)  PWM = pid->PWMLIMITE;
//	if(PWM < -pid->PWMLIMITE) PWM = -pid->PWMLIMITE;
//	return PWM;	// 返回实际值 
//}
