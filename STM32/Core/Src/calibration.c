#include "Calibration.h"
uint8_t motorGroup = 0;// 0 for motor[0] and motor[2], 1 for motor[1] and motor[3]
uint8_t Pressed = 0;
int8_t motor1Cali[2];
int8_t motor2Cali[2];
uint32_t StepsFront[4]={0,0,0,0};
int32_t StepsBack[4]={0,0};
uint32_t LastSteps[3] = {0,0,0,0};
uint8_t x = 0;
bool MotorsHoming(Motor *motor) {
    // These flags indicate whether each motor has finished homing.
    bool homed[4] = { false, false, false, false };

    // Start Motor 0 (using EndStop2, direction = 1, home = 0)
    if (IsSensorTriggered(EndStop2_GPIO_Port, EndStop2_Pin) == 0) {
        TMC2209_SetDirection(&motor[0], 1);
        TMC2209_SetSpeed(&motor[0], 8000);
        TMC2209_Start(&motor[0]);
    } else {
        homed[0] = true;  // already at home
        motor[0].currentPositionMM = 0;
        motor[0].stepsTaken = 0;
        motor[0].StepsBack = 0;
        motor[0].StepsFront = 0;
    }

    // Start Motor 1 (using EndStop4, direction = 0, home = 450)
    if (IsSensorTriggered(EndStop4_GPIO_Port, EndStop4_Pin) == 0) {
        TMC2209_SetDirection(&motor[1], 0);
        TMC2209_SetSpeed(&motor[1], 8000);
        TMC2209_Start(&motor[1]);
    } else {
        homed[1] = true;
        motor[1].currentPositionMM = 450;
        motor[1].stepsTaken = 0;
        motor[1].StepsBack = 0;
        motor[1].StepsFront = 0;
    }

    // Start Motor 2 (using EndStop1, direction = 0, home = 0)
    if (IsSensorTriggered(EndStop1_GPIO_Port, EndStop1_Pin) == 0) {
        TMC2209_SetDirection(&motor[2], 0);
        TMC2209_SetSpeed(&motor[2], 8000);
        TMC2209_Start(&motor[2]);
    } else {
        homed[2] = true;
        motor[2].currentPositionMM = 0;
        motor[2].stepsTaken = 0;
        motor[2].StepsBack = 0;
        motor[2].StepsFront = 0;
    }

    // Start Motor 3 (using EndStop3, direction = 1, home = 0)
    if (IsSensorTriggered(EndStop3_GPIO_Port, EndStop3_Pin) == 0) {
        TMC2209_SetDirection(&motor[3], 1);
        TMC2209_SetSpeed(&motor[3], 8000);
        TMC2209_Start(&motor[3]);
    } else {
        homed[3] = true;
        motor[3].currentPositionMM = 0;
        motor[3].stepsTaken = 0;
        motor[3].StepsBack = 0;
        motor[3].StepsFront = 0;
    }

    // Now poll all sensors until each motor has reached its home position.
    while (!(homed[0] && homed[1] && homed[2] && homed[3])) {
        if (!homed[0] && (IsSensorTriggered(EndStop2_GPIO_Port, EndStop2_Pin) == 1)) {
            TMC2209_Stop(&motor[0]);
            motor[0].currentPositionMM = 0;
            motor[0].stepsTaken = 0;
            motor[0].StepsBack = 0;
            motor[0].StepsFront = 0;
            homed[0] = true;
        }

        if (!homed[1] && (IsSensorTriggered(EndStop4_GPIO_Port, EndStop4_Pin) == 1)) {
            TMC2209_Stop(&motor[1]);
            motor[1].currentPositionMM = 0;
            motor[1].stepsTaken = 0;
            motor[1].StepsBack = 0;
            motor[1].StepsFront = 0;
            homed[1] = true;
        }

        if (!homed[2] && (IsSensorTriggered(EndStop1_GPIO_Port, EndStop1_Pin) == 1)) {
            TMC2209_Stop(&motor[2]);
            motor[2].currentPositionMM = 0;
            motor[2].stepsTaken = 0;
            motor[2].StepsBack = 0;
            motor[2].StepsFront = 0;
            homed[2] = true;
        }

        if (!homed[3] && (IsSensorTriggered(EndStop3_GPIO_Port, EndStop3_Pin) == 1)) {
            TMC2209_Stop(&motor[3]);
            motor[3].currentPositionMM = 0;
            motor[3].stepsTaken = 0;
            motor[3].StepsBack = 0;
            motor[3].StepsFront = 0;
            homed[3] = true;
        }

        // Small delay to prevent a tight loop hogging the CPU.
        HAL_Delay(1);
    }

    return true;
}


void MotorControl_ButtonHandler(Axis *axes,Motor *motors) {
	if (calibrationState()) {
	        return;
	    }

	static uint8_t CtrPressedFlag = 0; // Flag to detect button press edge
	    // StepsFront[0] = 0;
	//TMC2209_SetSpeed(&motors[0],10000);
	//TMC2209_SetSpeed(&motors[2],10000);
	    uint32_t pressStartTime = 0;
	    uint32_t debounceTime = 50;
	    uint32_t currentTime = HAL_GetTick();
	    SERVO_WritePosition(&hservo1, 60);
	    SERVO_WritePosition(&hservo2, 60);

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
                motors[motorGroup ].currentPositionMM =
                    abs(motors[motorGroup].StepsFront - motors[motorGroup].StepsBack) / 400;
                motors[motorGroup + 2].currentPositionMM =
                    abs(motors[motorGroup+ 2].StepsBack - motors[motorGroup+ 2].StepsFront) / 160;

                motors[motorGroup].calib[0] = motors[motorGroup].currentPositionMM;
                motors[motorGroup + 2].calib[0] = motors[motorGroup + 2].currentPositionMM;

                //motors[motorGroup * 2].currentPositionMM = 0;
                //motors[motorGroup * 2 + 1].currentPositionMM = 0;
                break;

            case 2:
                // Save calibration for second press
                motors[motorGroup].currentPositionMM =
                    abs(motors[motorGroup].StepsFront - motors[motorGroup].StepsBack) / 400.0f;
                motors[motorGroup+ 2].currentPositionMM =
                    abs(motors[motorGroup  + 2].StepsBack - motors[motorGroup+ 2].StepsFront) / 160.0f;

                motors[motorGroup].calib[1] = motors[motorGroup].currentPositionMM;
                motors[motorGroup+ 2].calib[1] = motors[motorGroup+ 2].currentPositionMM;
                motorGroup += 1;
//
//
                if (motorGroup >= 2) {
                       motorGroup = 0;  // Reset or handle as per your system's requirement
                  }
                // Perform homing for all motors




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
			TMC2209_SetDirection(&motors[motorGroup], GPIO_PIN_SET);
		    TMC2209_Start(&motors[motorGroup ]);
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
    	TMC2209_Stop(&motors[motorGroup]);
    }


	if(HAL_GPIO_ReadPin(BtnDown_GPIO_Port, BtnDown_Pin) == GPIO_PIN_RESET){
		//motors[motorGroup*2].stepsTaken = 0;
		//StepsBack[0] = 0;
		//StepsBack[0] += motors[motorGroup*2].stepsTaken;
		TMC2209_SetDirection(&motors[motorGroup], GPIO_PIN_RESET);
		TMC2209_Start(&motors[motorGroup]);
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
    	TMC2209_Stop(&motors[motorGroup]);

        //TMC2209_CountSteps_C(&motors[motorGroup * 2],StepsBack[0]);
    }


	if(HAL_GPIO_ReadPin(BtnRight_GPIO_Port, BtnRight_Pin) == GPIO_PIN_RESET){
        TMC2209_SetDirection(&motors[motorGroup +2], GPIO_PIN_SET);
        TMC2209_Start(&motors[motorGroup +2]);
        while(HAL_GPIO_ReadPin(BtnRight_GPIO_Port, BtnRight_Pin) == GPIO_PIN_RESET);
}
    if (HAL_GPIO_ReadPin(BtnRight_GPIO_Port, BtnRight_Pin) == GPIO_PIN_SET) {
        // Button 1 pressed (Step Motor in one direction)
        TMC2209_Stop(&motors[motorGroup +2]);
    }


	if(HAL_GPIO_ReadPin(BtnLeft_GPIO_Port, BtnLeft_Pin) == GPIO_PIN_RESET){
        TMC2209_SetDirection(&motors[motorGroup+2], GPIO_PIN_RESET);
        TMC2209_Start(&motors[motorGroup +2]);
        while(HAL_GPIO_ReadPin(BtnLeft_GPIO_Port, BtnLeft_Pin) == GPIO_PIN_RESET);
}
    if (HAL_GPIO_ReadPin(BtnLeft_GPIO_Port, BtnLeft_Pin) == GPIO_PIN_SET) {
        // Button 1 pressed (Step Motor in one direction)
        TMC2209_Stop(&motors[motorGroup +2]);
    }
}
bool calibrationState(){
	if(motors[0].calib[1] !=0 && motors[2].calib[1] !=0 && motors[2].calib[1] !=0 && motors[3].calib[1] !=0 ){
		    	 MotorsHoming(motors);
		    	 SERVO_WritePosition(&hservo1,80);
		    	 SERVO_WritePosition(&hservo2,80);

		       // Move all motors to their saved calibrated positions
		        TMC2209_MoveTo(&axes[0],0,motors[0].calib[0]+0.33);
		    	TMC2209_MoveTo(&axes[1],0,-(motors[2].calib[1]+0.5));
		        TMC2209_MoveTo(&axes[0],1,-(motors[1].calib[0]+0.33));
		    	TMC2209_MoveTo(&axes[1],1,motors[3].calib[1]+0.5);
		    	motors[0].currentPositionMM = 0;
		    	motors[1].currentPositionMM = 100;
		    	motors[2].currentPositionMM = 0;
		    	motors[3].currentPositionMM = 0;
		    	HAL_Delay(10000);
		    	TMC2209_MoveTo(&axes[0],0,20.5995);
		    	TMC2209_MoveTo(&axes[1],0,-37.5995);
		    	TMC2209_MoveTo(&axes[0],1,44.5995);
		        TMC2209_MoveTo(&axes[1],1,20.5995);

		    	CheckConnection(&hservo2,&hservo1);
		    	if(CheckConnection(&hservo2,&hservo1)){
		    		x = 1;
		    	}
		    	 return 1;
		     }
		     else{
		    	 return 0;
		     }
}


