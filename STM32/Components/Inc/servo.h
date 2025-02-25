#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#ifdef USE_HAL_DRIVER
#include "stm32f7xx_hal.h"         // Include the main HAL header
#include <math.h>
#include "stm32f7xx_hal_tim.h"
#endif

typedef TIM_HandleTypeDef* PWM_TIM_TypeDef;
typedef uint32_t PWM_Channel_TypeDef;

typedef struct {
  PWM_TIM_TypeDef Timer;
  PWM_Channel_TypeDef Channel;
  float Duty;
} PWM_Handle_TypeDef;

typedef struct {
  PWM_Handle_TypeDef PwmOut;
  float Position;
  uint8_t ID;
} SERVO_Handle_TypeDef;

#define PWM_INIT_HANDLE(TIMER_HANDLE, CHANNEL) \
  {                                            \
    .Timer = TIMER_HANDLE,                     \
    .Channel = CHANNEL,                        \
    .Duty = 0.0f                               \
  }

#define SERVO1_HOME_POS 90.0f
#define SERVO2_HOME_POS 60.0f
#define SERVO1_CHECK_POS 70.0f
#define SERVO2_CHECK_POS 38.0f
#define __LINEAR_TRANSFORM(x,amin,amax,bmin,bmax) (((x-amin)/(amax-amin))*(bmax-bmin)+bmin)
#define __SATURATION(x,xmin,xmax) fmaxf(fminf(x, xmax), xmin)

void SERVO_Init(SERVO_Handle_TypeDef* hservo);
void SERVO_WritePosition(SERVO_Handle_TypeDef* hservo, float pos);
float SERVO_ReadPosition(SERVO_Handle_TypeDef* hservo);

void PWM_Init(PWM_Handle_TypeDef* hpwm);
void PWM_WriteDuty(PWM_Handle_TypeDef* hpwm, float duty);
float PWM_ReadDuty(const PWM_Handle_TypeDef* hpwm);

#endif /* INC_SERVO_H_ */
