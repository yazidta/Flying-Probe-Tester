/******************************************************************************
 * @file       TMC2209.cpp
 * @author     Ahmed Bouras
 * @date       05/12/2024
 * @version    0.6
 *
 * @copyright  Copyright (c) [2024] Ahmed Bouras
 *
 * @license    Permission is hereby granted, free of charge, to any person
 *             obtaining a copy of this software and associated documentation
 *             files (the "Software"), to deal in the Software without
 *             restriction, including without limitation the rights to use,
 *             copy, modify, merge, publish, distribute, sublicense, and/or
 *             sell copies of the Software, and to permit persons to whom
 *             the Software is furnished to do so, subject to the following
 *             conditions:
 *
 *             The above copyright notice and this permission notice shall
 *             be included in all copies or substantial portions of the
 *             Software.
 *
 *             THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
 *             KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 *             WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 *             PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
 *             OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 *             OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 *             OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 *             SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * @note       This library provides an interface for controlling the TMC2209
 *             stepper motor driver over STEP, DIR, ENN & UART.
 ******************************************************************************
 */

#ifndef INC_TMC2209_REGISTER_H_
#define INC_TMC2209_REGISTER_H_


// Note that there might be some missing registers please refer to the datasheet if you couldn't find something.


// Register addresses
#define TMC2209_REG_GCONF        0x00
#define TMC2209_REG_GSTAT        0x01
#define TMC2209_REG_IFCNT        0x02
#define TMC2209_REG_SLAVECONF    0x03
#define TMC2209_REG_OTP_PROG     0x04
#define TMC2209_REG_OTP_READ     0x05
#define TMC2209_REG_IOIN         0x06
#define TMC2209_REG_FACTORY_CONF 0x07

#define TMC2209_REG_IHOLD_IRUN   0x10  // Bits 0–4: IHOLD (current for hold mode), Bits 8–12: IRUN (current for motor operation. max 31), Bits 16–23: IHOLDDELAY (duration of the transition from IRUN to IHOLD)
#define TMC2209_REG_TPOWERDOWN   0x11
#define TMC2209_REG_TSTEP        0x12
#define TMC2209_REG_TPWMTHRS     0x13
#define TMC2209_REG_VACTUAL      0x22

#define TMC2209_REG_TCOOLTHRS    0x14
#define TMC2209_REG_SGTHRS       0x40
#define TMC2209_REG_SG_RESULT    0x41
#define TMC2209_REG_COOLCONF     0x42
#define TMC_REG_SENDDELAY 		 0x03
#define TMC_REG_GCONF 			 0x00
#define TMC2209_REG_MSCNT        0x6A
#define TMC2209_REG_MSCURACT     0x6B
#define TMC2209_REG_CHOPCONF     0x6C
#define TMC2209_REG_DRVSTATUS    0x6F
#define TMC2209_REG_PWMCONF      0x70
#define TMC2209_REG_PWM_SCALE    0x71
#define TMC2209_REG_PWM_AUTO     0x72


// GCONF register bit positions
#define TMC2209_I_SCALE_ANALOG_POS   0
#define TMC2209_INTERNAL_RSENSE_POS  1
#define TMC2209_EN_SPREADCYCLE_POS   2
#define TMC2209_SHAFT_POS            3
#define TMC2209_INDEX_OTPW_POS       4
#define TMC2209_INDEX_STEP_POS       5
#define TMC2209_PDN_DISABLE_POS      6
#define TMC2209_MSTEP_REG_SELECT_POS 7
#define TMC2209_MULTISTEP_FILT_POS   8
#define TMC2209_TEST_MODE_POS        9

#endif /* INC_TMC2209_REGISTER_H_ */
