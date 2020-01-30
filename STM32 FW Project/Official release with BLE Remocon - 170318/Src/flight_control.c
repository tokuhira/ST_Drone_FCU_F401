#include "flight_control.h"
#include "rc.h"
#include <math.h>

// https://www.cqpub.co.jp/interface/download/2020/3/IF2003T.zip
#define IF2003T

float pid_x_integ1 = 0;
float pid_y_integ1 = 0;
float pid_z_integ1 = 0;
float pid_x_integ2 = 0;
float pid_y_integ2 = 0;
float pid_z_integ2 = 0;
float pid_x_pre_error2 = 0;
float pid_y_pre_error2 = 0;
float pid_z_pre_error2 = 0;
float pid_x_pre_deriv = 0;
float pid_y_pre_deriv = 0;

extern int16_t gTHR;
int16_t motor_thr;
float dt_recip;

#ifdef IF2003T
// https://www.cqpub.co.jp/interface/download/2020/3/IF2003T.zip
// p.87, list 2
float rp_rctrl_Fa[] = RP_RCTRL_FA;
float rp_rctrl_Ga[] = RP_RCTRL_GA;
float rp_rctrl_Ha[] = RP_RCTRL_HA;
float rp_rctrl_Aod[] = RP_RCTRL_AOD;
float rp_rctrl_Bod[] = RP_RCTRL_BOD;
float rp_rctrl_Cod[] = RP_RCTRL_COD;
float egx_integ = 0;	// gxの追従偏差の積分値
float egy_integ = 0;	// gyの追従偏差の積分値
float dmxe	= 0;	// MXの推定値
float dmye	= 0;	// MYの推定値
#endif

void PIDControlInit(P_PI_PIDControlTypeDef *pid)
{
  pid->ts = PID_SAMPLING_TIME;

  pid->x_kp1 = PITCH_PID_KP1;
  pid->x_ki1 = PITCH_PID_KI1;
  pid->x_i1_limit = PITCH_PID_I1_LIMIT;
  pid->x_kp2 = PITCH_PID_KP2;
  pid->x_ki2 = PITCH_PID_KI2;
  pid->x_kd2 = PITCH_PID_KD2;
  pid->x_i2_limit = PITCH_PID_I2_LIMIT;
  pid->x_s1 = 0;
  pid->x_s2 = 0;

  pid->y_kp1 = ROLL_PID_KP1;
  pid->y_ki1 = ROLL_PID_KI1;
  pid->y_i1_limit = ROLL_PID_I1_LIMIT;
  pid->y_kp2 = ROLL_PID_KP2;
  pid->y_ki2 = ROLL_PID_KI2;
  pid->y_kd2 = ROLL_PID_KD2;
  pid->y_i2_limit = ROLL_PID_I2_LIMIT;
  pid->y_s1 = 0;
  pid->y_s2 = 0;

  pid->z_kp1 = YAW_PID_KP1;
  pid->z_ki1 = YAW_PID_KI1;
  pid->z_i1_limit = YAW_PID_I1_LIMIT;
  pid->z_kp2 = YAW_PID_KP2;
  pid->z_ki2 = YAW_PID_KI2;
  pid->z_kd2 = YAW_PID_KD2;
  pid->z_i2_limit = YAW_PID_I2_LIMIT;
  pid->z_s1 = 0;
  pid->z_s2 = 0;
}

void FlightControlPID(EulerAngleTypeDef *euler_rc, EulerAngleTypeDef *euler_ahrs, Gyro_Rad *gyro_rad, AHRS_State_TypeDef *ahrs, P_PI_PIDControlTypeDef *pid, MotorControlTypeDef *motor_pwm)
{
  float error, deriv;

  if(gTHR<MIN_THR)
  {
    pid_x_integ1 = 0;
    pid_y_integ1 = 0;
    pid_z_integ1 = 0;
    pid_x_integ2 = 0;
    pid_y_integ2 = 0;
    pid_z_integ2 = 0;
  }

  
  //x-axis pid
  error = euler_rc->thx - euler_ahrs->thx;
  pid_x_integ1 += error*pid->ts;
  if(pid_x_integ1 > pid->x_i1_limit)
    pid_x_integ1 = pid->x_i1_limit;
  else if(pid_x_integ1 < -pid->x_i1_limit)
    pid_x_integ1 = -pid->x_i1_limit;
  pid->x_s1 =  pid->x_kp1*error + pid->x_ki1*pid_x_integ1;

  error = euler_rc->thx - gyro_rad->gx;
  pid_x_integ2 += error*pid->ts;
  if(pid_x_integ2 > pid->x_i2_limit)
    pid_x_integ2 = pid->x_i2_limit;
  else if(pid_x_integ2 < -pid->x_i2_limit)
    pid_x_integ2 = -pid->x_i2_limit;
  deriv = error - pid_x_pre_error2;
  pid_x_pre_error2 = error;
  pid->x_s2 = pid->x_kp2*error + pid->x_ki2*pid_x_integ2 + pid->x_kd2*deriv;

  if(pid->x_s2 > MAX_ADJ_AMOUNT)  pid->x_s2 = MAX_ADJ_AMOUNT;
  if(pid->x_s2 < -MAX_ADJ_AMOUNT)  pid->x_s2 = -MAX_ADJ_AMOUNT;


  //y-axis pid
  error = euler_rc->thy - euler_ahrs->thy;
  pid_y_integ1 += error*pid->ts;
  if(pid_y_integ1 > pid->y_i1_limit)
    pid_y_integ1 = pid->y_i1_limit;
  else if(pid_y_integ1 < -pid->y_i1_limit)
    pid_y_integ1 = -pid->y_i1_limit;
  pid->y_s1 =  pid->y_kp1*error + pid->y_ki1*pid_y_integ1;

  error = euler_rc->thy - gyro_rad->gy;
  pid_y_integ2 += error*pid->ts;
  if(pid_y_integ2 > pid->y_i2_limit)
    pid_y_integ2 = pid->y_i2_limit;
  else if(pid_y_integ2 < -pid->y_i2_limit)
    pid_y_integ2 = -pid->y_i2_limit;
  deriv = error - pid_y_pre_error2;
  pid_y_pre_error2 = error;
  pid->y_s2 = pid->y_kp2*error + pid->y_ki2*pid_y_integ2 + pid->y_kd2*deriv;

  if(pid->y_s2 > MAX_ADJ_AMOUNT)  pid->y_s2 = MAX_ADJ_AMOUNT;
  if(pid->y_s2 < -MAX_ADJ_AMOUNT)  pid->y_s2 = -MAX_ADJ_AMOUNT;


  //z-axis pid
  error = euler_rc->thz - gyro_rad->gz;
  pid_z_integ2 += error*pid->ts;
  if(pid_z_integ2 > pid->z_i2_limit)
    pid_z_integ2 = pid->z_i2_limit;
  else if(pid_z_integ2 < -pid->z_i2_limit)
    pid_z_integ2 = -pid->z_i2_limit;
  deriv = error - pid_z_pre_error2;
  pid_z_pre_error2 = error;
  pid->z_s2 = pid->z_kp2*error + pid->z_ki2*pid_y_integ2 + pid->z_kd2*deriv;

  if(pid->z_s2 > MAX_ADJ_AMOUNT)  pid->z_s2 = MAX_ADJ_AMOUNT;
  if(pid->z_s2 < -MAX_ADJ_AMOUNT)  pid->z_s2 = -MAX_ADJ_AMOUNT;

  #ifdef MOTOR_DC

    motor_thr = ((int16_t) (0.33333f*(float)gTHR + 633.333f));           //Devo7E >> 630 to 1700
  
  #endif
  
  #ifdef MOTOR_ESC
  
    //motor_thr = 0.28f*gTHR + 750.0f;                          //TGY-i6 remocon and external ESC STEVAL-ESC001V1
    motor_thr = ((int16_t) (0.28f*(float)gTHR + 850.0f));       //TGY-i6 remocon and external ESC Afro12A

  #endif
  
  
  motor_pwm->motor1_pwm = motor_thr - pid->x_s2 - pid->y_s2 + pid->z_s2 + MOTOR_OFF1;
  motor_pwm->motor2_pwm = motor_thr + pid->x_s2 - pid->y_s2 - pid->z_s2 + MOTOR_OFF2;
  motor_pwm->motor3_pwm = motor_thr + pid->x_s2 + pid->y_s2 + pid->z_s2 + MOTOR_OFF3;
  motor_pwm->motor4_pwm = motor_thr - pid->x_s2 + pid->y_s2 - pid->z_s2 + MOTOR_OFF4;


}

void FlightControlPID_OuterLoop(EulerAngleTypeDef *euler_rc, EulerAngleTypeDef *euler_ahrs, AHRS_State_TypeDef *ahrs, P_PI_PIDControlTypeDef *pid)
{
  float error;

  if(gTHR<MIN_THR)
  {
    pid_x_integ1 = 0;
    pid_y_integ1 = 0;
    pid_z_integ1 = 0;
  }

  //x-axis pid
  error = euler_rc->thx - euler_ahrs->thx;
  pid_x_integ1 += error*pid->ts;
  if(pid_x_integ1 > pid->x_i1_limit)
    pid_x_integ1 = pid->x_i1_limit;
  else if(pid_x_integ1 < -pid->x_i1_limit)
    pid_x_integ1 = -pid->x_i1_limit;
  pid->x_s1 =  pid->x_kp1*error + pid->x_ki1*pid_x_integ1;

  //y-axis pid
  error = euler_rc->thy - euler_ahrs->thy;
  pid_y_integ1 += error*pid->ts;
  if(pid_y_integ1 > pid->y_i1_limit)
    pid_y_integ1 = pid->y_i1_limit;
  else if(pid_y_integ1 < -pid->y_i1_limit)
    pid_y_integ1 = -pid->y_i1_limit;
  pid->y_s1 =  pid->y_kp1*error + pid->y_ki1*pid_y_integ1;

  //z-axis pid
  error = euler_rc->thz - euler_ahrs->thz;
  pid_z_integ1 += error*pid->ts;
  if(pid_z_integ1 > pid->z_i1_limit)
    pid_z_integ1 = pid->z_i1_limit;
  else if(pid_z_integ1 < -pid->z_i1_limit)
    pid_z_integ1 = -pid->z_i1_limit;
  pid->z_s1 =  pid->z_kp1*error + pid->z_ki1*pid_z_integ1;
}

#ifdef IF2003T
// https://www.cqpub.co.jp/interface/download/2020/3/IF2003T.zip
// p.87, list 2
float u1_F, u2_F;
float u1_G, u2_G;
float u1_H, u2_H;
float mx;			// モーメントMX推定値
float my;			// モーメントMY推定値
float egx;		// gxの追従偏差
float egy;		// gyの追従偏差
float x_s2_l, y_s2_l;	// x_s2, y_s2の値（状態推定器用）
float dmxe_next;	// dmxeの1サンプル更新後の値
float dmye_next;	// dmyeの1サンプル更新後の値
#endif

void FlightControlPID_innerLoop(EulerAngleTypeDef *euler_rc, Gyro_Rad *gyro_rad, AHRS_State_TypeDef *ahrs, P_PI_PIDControlTypeDef *pid, MotorControlTypeDef *motor_pwm)
{
  float error, deriv;

  if(gTHR<MIN_THR)
  {
    pid_x_integ2 = 0;
    pid_y_integ2 = 0;
    pid_z_integ2 = 0;
  }
  
  dt_recip = 1/pid->ts;

#ifdef IF2003T
// https://www.cqpub.co.jp/interface/download/2020/3/IF2003T.zip
// p.87, list 2
  if(gTHR<MIN_THR)
  {
    egx_integ = 0; 
    egy_integ = 0;
    dmxe = 0;
    dmye = 0;
  }

  //XY Axis
  mx = rp_rctrl_Cod[0] * dmxe + rp_rctrl_Cod[1] * dmye;
  my = rp_rctrl_Cod[2] * dmxe + rp_rctrl_Cod[3] * dmye;
  u1_F = rp_rctrl_Fa[0] * mx + rp_rctrl_Fa[1] * my +
         rp_rctrl_Fa[2] * gyro_rad->gx + rp_rctrl_Fa[3] * gyro_rad->gy;
  u2_F = rp_rctrl_Fa[4] * mx + rp_rctrl_Fa[5] * my +
         rp_rctrl_Fa[6] * gyro_rad->gx + rp_rctrl_Fa[7] * gyro_rad->gy;
  egx = pid->x_s1 - gyro_rad->gx;
  egy = pid->y_s1 - gyro_rad->gy;
  egx_integ += egx * pid->ts;
  if(egx_integ > EGX_I_LIMIT)
    egx_integ = EGX_I_LIMIT;
  else if(egx_integ < -EGX_I_LIMIT)
    egx_integ = -EGX_I_LIMIT;
  egy_integ += egy * pid->ts;
  if(egy_integ > EGY_I_LIMIT)
    egy_integ = EGY_I_LIMIT;
  else if(egy_integ < -EGY_I_LIMIT)
    egy_integ = -EGY_I_LIMIT;
  u1_G = rp_rctrl_Ga[0] * egx_integ + rp_rctrl_Ga[1] * egy_integ;
  u2_G = rp_rctrl_Ga[2] * egx_integ + rp_rctrl_Ga[3] * egy_integ;
  u1_H = rp_rctrl_Ha[0] * pid->x_s1 + rp_rctrl_Ha[1] * pid->y_s1;
  u2_H = rp_rctrl_Ha[2] * pid->x_s1 + rp_rctrl_Ha[3] * pid->y_s1;
  pid->x_s2 = u1_F + u1_G + u1_H;
  pid->y_s2 = u2_F + u2_G + u2_H;

  if(pid->x_s2 > MAX_ADJ_AMOUNT)  pid->x_s2 = MAX_ADJ_AMOUNT;
  if(pid->x_s2 < -MAX_ADJ_AMOUNT)  pid->x_s2 = -MAX_ADJ_AMOUNT;

  if(pid->y_s2 > MAX_ADJ_AMOUNT)  pid->y_s2 = MAX_ADJ_AMOUNT;
  if(pid->y_s2 < -MAX_ADJ_AMOUNT)  pid->y_s2 = -MAX_ADJ_AMOUNT;

  x_s2_l = pid->x_s2;
  if (x_s2_l > X_S2_LIMIT_O)
    x_s2_l = X_S2_LIMIT_O;
  else if (x_s2_l < -X_S2_LIMIT_O)
    x_s2_l = -X_S2_LIMIT_O;
  y_s2_l = pid->y_s2;
  if (y_s2_l > Y_S2_LIMIT_O)
    y_s2_l = Y_S2_LIMIT_O;
  else if (y_s2_l < -Y_S2_LIMIT_O)
    y_s2_l = -Y_S2_LIMIT_O;
  dmxe_next = rp_rctrl_Aod[0] * dmxe + rp_rctrl_Aod[1] * dmye +
    rp_rctrl_Bod[0] * x_s2_l + rp_rctrl_Bod[1] * y_s2_l;
  dmye_next = rp_rctrl_Aod[2] * dmxe + rp_rctrl_Aod[3] * dmye +
    rp_rctrl_Bod[2] * x_s2_l + rp_rctrl_Bod[3] * y_s2_l;
  dmxe = dmxe_next;
  dmye = dmye_next;
#else
  //X Axis
  error = pid->x_s1 - gyro_rad->gx;
  pid_x_integ2 += error*pid->ts;
  if(pid_x_integ2 > pid->x_i2_limit)
    pid_x_integ2 = pid->x_i2_limit;
  else if(pid_x_integ2 < -pid->x_i2_limit)
    pid_x_integ2 = -pid->x_i2_limit;
  deriv = (error - pid_x_pre_error2)*dt_recip;
  pid_x_pre_error2 = error;
  deriv = pid_x_pre_deriv + (deriv - pid_x_pre_deriv)*D_FILTER_COFF;
  pid_x_pre_deriv = deriv;
  pid->x_s2 = pid->x_kp2*error + pid->x_ki2*pid_x_integ2 + pid->x_kd2*deriv;
  
  if(pid->x_s2 > MAX_ADJ_AMOUNT)  pid->x_s2 = MAX_ADJ_AMOUNT;
  if(pid->x_s2 < -MAX_ADJ_AMOUNT)  pid->x_s2 = -MAX_ADJ_AMOUNT;

  //Y Axis
  error = pid->y_s1 - gyro_rad->gy;
  pid_y_integ2 += error*pid->ts;
  if(pid_y_integ2 > pid->y_i2_limit)
    pid_y_integ2 = pid->y_i2_limit;
  else if(pid_y_integ2 < -pid->y_i2_limit)
    pid_y_integ2 = -pid->y_i2_limit;
  deriv = (error - pid_y_pre_error2)*dt_recip;
  pid_y_pre_error2 = error;
  deriv = pid_y_pre_deriv + (deriv - pid_y_pre_deriv)*D_FILTER_COFF;
  pid_y_pre_deriv = deriv;
  pid->y_s2 = pid->y_kp2*error + pid->y_ki2*pid_y_integ2 + pid->y_kd2*deriv;

  if(pid->y_s2 > MAX_ADJ_AMOUNT)  pid->y_s2 = MAX_ADJ_AMOUNT;
  if(pid->y_s2 < -MAX_ADJ_AMOUNT)  pid->y_s2 = -MAX_ADJ_AMOUNT;
#endif

  //Z Axis
  error = pid->z_s1 - gyro_rad->gz;
  pid_z_integ2 += error*pid->ts;
  if(pid_z_integ2 > pid->z_i2_limit)
    pid_z_integ2 = pid->z_i2_limit;
  else if(pid_z_integ2 < -pid->z_i2_limit)
    pid_z_integ2 = -pid->z_i2_limit;
  deriv = (error - pid_z_pre_error2)*dt_recip;
  pid_z_pre_error2 = error;
  pid->z_s2 = pid->z_kp2*error + pid->z_ki2*pid_z_integ2 + pid->z_kd2*deriv;

  if(pid->z_s2 > MAX_ADJ_AMOUNT_YAW)  pid->z_s2 = MAX_ADJ_AMOUNT_YAW;
  if(pid->z_s2 < -MAX_ADJ_AMOUNT_YAW)  pid->z_s2 = -MAX_ADJ_AMOUNT_YAW;

  
#ifdef MOTOR_DC

  motor_thr = ((int16_t) (0.05f*(float)gTHR + 633.333f));           //Official MiniDrone Kit >> 630 to 1700
  //motor_thr =((int16_t) (0.333f*(float)gTHR + 633.33f));           //Remocon Devo7E >> 630 to 1700
  
#endif
  
#ifdef MOTOR_ESC
  
  //motor_thr = 0.28f*gTHR + 750.0f;                 //TGY-i6 remocon and external ESC STEVAL-ESC001V1
    motor_thr = ((int16_t) (0.28f*(float)gTHR + 850.0f));                 //TGY-i6 remocon and external ESC Afro12A

#endif

  motor_pwm->motor1_pwm = motor_thr - pid->x_s2 - pid->y_s2 + pid->z_s2 + MOTOR_OFF1;
  motor_pwm->motor2_pwm = motor_thr + pid->x_s2 - pid->y_s2 - pid->z_s2 + MOTOR_OFF2;
  motor_pwm->motor3_pwm = motor_thr + pid->x_s2 + pid->y_s2 + pid->z_s2 + MOTOR_OFF3;
  motor_pwm->motor4_pwm = motor_thr - pid->x_s2 + pid->y_s2 - pid->z_s2 + MOTOR_OFF4;

}

void PIDOuterLoopFrameTrans(P_PI_PIDControlTypeDef *pid, EulerAngleTypeDef *euler_ahrs)
{
  float cosx;
  
  cosx = cos(euler_ahrs->thx);
  pid->y_s1 = cosx*pid->y_s1;

}
