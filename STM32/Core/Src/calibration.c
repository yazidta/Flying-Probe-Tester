#include "Calibration.h"
uint8_t motorGroup = 0;// 0 for motor[0] and motor[2], 1 for motor[1] and motor[3]
int8_t motor1Cali[2];
int8_t motor2Cali[2];
uint32_t StepsFront[4]={0,0,0,0};
int32_t StepsBack[4]={0,0};
uint32_t LastSteps[3] = {0,0,0,0};

void motorHoming(Motor *motor, GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin, int direction, int speed, int homePositionMM) {
    TMC2209_SetDirection(motor, direction);
    TMC2209_SetSpeed(motor, speed);

    if (IsSensorTriggered(GPIO_Port, GPIO_Pin) == 0) {
        TMC2209_Start(motor);

        while (IsSensorTriggered(GPIO_Port, GPIO_Pin) == 0);

        if (IsSensorTriggered(GPIO_Port, GPIO_Pin) == 1) {
            TMC2209_Stop(motor);
            motor->currentPositionMM = homePositionMM;
            motor->stepsTaken = 0;
            motor->StepsBack = 0;
            motor->StepsFront = 0;
        }
    }

    TMC2209_Stop(motor);
}

bool MotorsHoming(Motor *motor) {
    motorHoming(&motor[0], EndStop1_GPIO_Port, EndStop1_Pin, 1, 8000, 0);
    motorHoming(&motor[1], EndStop2_GPIO_Port, EndStop2_Pin, 0, 14000, 0);
    motorHoming(&motor[2], EndStop3_GPIO_Port, EndStop3_Pin, 0, 10000, 0);
    motorHoming(&motor[3], EndStop4_GPIO_Port, EndStop4_Pin, 0, 8000, 450);

    return true;
}


void MotorControl_ButtonHandler(Axis *axes,Motor *motors) {
	static uint8_t CtrPressedFlag = 0; // Flag to detect button press edge
	    // StepsFront[0] = 0;
	//TMC2209_SetSpeed(&motors[0],10000);
	//TMC2209_SetSpeed(&motors[2],10000);
	    uint32_t pressStartTime = 0;
	    uint32_t debounceTime = 50;
	    uint32_t currentTime = HAL_GetTick();
	    static uint32_t lastPressTime = 0; // Last valid press timestamp

	    if (HAL_GPIO_ReadPin(BtnCtr_GPIO_Port, BtnCtr_Pin) == GPIO_PIN_SET) {
	        if (CtrPressedFlag == 0) { // Only increment on first press
	            pressStartTime = currentTime;
	            CtrPressedFlag = 1; // Set flag to avoid multiple increments
	        }
	    } else {
	        if (CtrPressedFlag == 1 && (currentTime - pressStartTime) >= debounceTime) {
	            Pressed += 1;
	            lastPressTime = currentTime; // Update the last valid press time
	              }
	        CtrPressedFlag = 0; // Reset flag when button is released
        switch (Pressed) {
            case 1:
                // Save calibration for first press
                motors[motorGroup * 2].currentPositionMM =
                    abs(motors[motorGroup * 2].StepsFront - motors[motorGroup * 2].StepsBack) / 160;
                motors[motorGroup * 2 + 1].currentPositionMM =
                    abs(motors[motorGroup * 2 + 1].StepsBack - motors[motorGroup * 2 + 1].StepsFront) / 400;

                motors[motorGroup * 2].calib[0] = motors[motorGroup * 2].currentPositionMM;
                motors[motorGroup * 2 + 1].calib[0] = motors[motorGroup * 2 + 1].currentPositionMM;

                motors[motorGroup * 2].currentPositionMM = 0;
                motors[motorGroup * 2 + 1].currentPositionMM = 0;
                break;

            case 2:
                // Save calibration for second press
                motors[motorGroup * 2].currentPositionMM =
                    abs(motors[motorGroup * 2].StepsFront - motors[motorGroup * 2].StepsBack) / 160.0f;
                motors[motorGroup * 2 + 1].currentPositionMM =
                    abs(motors[motorGroup * 2 + 1].StepsBack - motors[motorGroup * 2 + 1].StepsFront) / 400.0f;

                motors[motorGroup * 2].calib[1] = motors[motorGroup * 2].currentPositionMM;
                motors[motorGroup * 2 + 1].calib[1] = motors[motorGroup * 2 + 1].currentPositionMM;
                if (motorGroup >= 2) {
                       motorGroup = 0;  // Reset or handle as per your system's requirement
                  }
                // Perform homing for all motors
                  //MotorsHoming(motors);

               // Move all motors to their saved calibrated positions
                if(motorGroup == 0){
                   TMC2209_MoveTo(&axes[0],0,motors[0].calib[1]);
                   TMC2209_MoveTo(&axes[1],0,-motors[1].calib[0]);
//                TMC2209_MoveTo(&axes[1],1,-400);
                }
                else {
                	TMC2209_MoveTo(&axes[0],1,motors[2].calib[1]);
                    TMC2209_MoveTo(&axes[1],1,-motors[3].calib[0]);
                }
                motorGroup += 1;

           // Reset Pressed counter to prevent further calibration steps
                Pressed = 0;
                    break;

                 default:
                                // Handle unexpected Pressed value
                   Pressed = 0;
                   break;
                        }

    }



	if(HAL_GPIO_ReadPin(BtnUp_GPIO_Port, BtnUp_Pin) == GPIO_PIN_RESET){
		    // Send one step for each millisecond the button is pressed
	    //setMicrosteppingResolution(&motors[motorGroup *2], 16);
		//TMC2209_SetSpeed(&motors[motorGroup *2+1],16000);
			//StepsFront[0] = 0;
            //LastSteps[0] += StepsFront[0];
			TMC2209_SetDirection(&motors[motorGroup * 2], GPIO_PIN_SET);
		    TMC2209_Start_C(&motors[motorGroup * 2]);
		    while(HAL_GPIO_ReadPin(BtnUp_GPIO_Port, BtnUp_Pin) == GPIO_PIN_RESET){
    		    //StepsFront[0] = motors[motorGroup *2].stepsTaken + LastSteps[0];
		    }
		    //LastSteps = StepsFront[0];
    	//motors[motorGroup *2].currentPositionMM = StepsFront[0] * 160;
//    	if(StepsFront[0] <= -28000){
//        	TMC2209_Stop(&motors[motorGroup * 2]);
//        	StepsFront[0] = 0;

    	//}
}
    if (HAL_GPIO_ReadPin(BtnUp_GPIO_Port, BtnUp_Pin) == GPIO_PIN_SET ) {
    	TMC2209_Stop(&motors[motorGroup * 2]);
    }


	if(HAL_GPIO_ReadPin(BtnDown_GPIO_Port, BtnDown_Pin) == GPIO_PIN_RESET){
		//motors[motorGroup*2].stepsTaken = 0;
		//StepsBack[0] = 0;
		//StepsBack[0] += motors[motorGroup*2].stepsTaken;
		TMC2209_SetDirection(&motors[motorGroup * 2], GPIO_PIN_RESET);
		TMC2209_Start_C(&motors[motorGroup * 2]);
		while(HAL_GPIO_ReadPin(BtnDown_GPIO_Port, BtnDown_Pin) == GPIO_PIN_RESET){
    	//StepsBack[0] = -(int)(motors[motorGroup *2].stepsTaken);
		}
//    	if(StepsBack[0] >= 28000){
//        	TMC2209_Stop(&motors[motorGroup * 2]);
//        	StepsBack[0] = 0;
//
//
//    	}


}
    if (HAL_GPIO_ReadPin(BtnDown_GPIO_Port, BtnDown_Pin) == GPIO_PIN_SET || StepsBack[0] > 28000) {
        // Button 1 pressed (Step Motor in one direction)
    	TMC2209_Stop(&motors[motorGroup * 2]);

        //TMC2209_CountSteps_C(&motors[motorGroup * 2],StepsBack[0]);
    }


	if(HAL_GPIO_ReadPin(BtnRight_GPIO_Port, BtnRight_Pin) == GPIO_PIN_RESET){
        TMC2209_SetDirection(&motors[motorGroup * 2+1], GPIO_PIN_SET);
        TMC2209_Start_C(&motors[motorGroup * 2+1]);
        while(HAL_GPIO_ReadPin(BtnRight_GPIO_Port, BtnRight_Pin) == GPIO_PIN_RESET);
}
    if (HAL_GPIO_ReadPin(BtnRight_GPIO_Port, BtnRight_Pin) == GPIO_PIN_SET) {
        // Button 1 pressed (Step Motor in one direction)
        TMC2209_Stop(&motors[motorGroup * 2 + 1]);
    }


	if(HAL_GPIO_ReadPin(BtnLeft_GPIO_Port, BtnLeft_Pin) == GPIO_PIN_RESET){
        TMC2209_SetDirection(&motors[motorGroup * 2+1], GPIO_PIN_RESET);
        TMC2209_Start_C(&motors[motorGroup * 2+1]);
        while(HAL_GPIO_ReadPin(BtnLeft_GPIO_Port, BtnLeft_Pin) == GPIO_PIN_RESET);
}
    if (HAL_GPIO_ReadPin(BtnLeft_GPIO_Port, BtnLeft_Pin) == GPIO_PIN_SET) {
        // Button 1 pressed (Step Motor in one direction)
        TMC2209_Stop(&motors[motorGroup * 2+1]);
    }
}



