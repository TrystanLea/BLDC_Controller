/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Motor control header file.
 *
 *      This file contains all #defines, typedefs and prototypes related to
 *      the motor control.
 *
 * \par Application note:
 *      AVR447: Sinusoidal driving of three-phase permanent motor using
 *      ATmega48/88/168
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * $Name: RELEASE_1_0 $
 * $Revision: 1.9 $
 * $RCSfile: PMSM.h,v $
 * $Date: 2006/03/27 07:20:51 $  \n
 ******************************************************************************/

#ifndef _PMSM_H_
#define _PMSM_H_


//! FALSE constant value.
#define FALSE   0

//! TRUE constant value, defined to be compatible with comparisons.
#define TRUE    (!FALSE)

/*! This value specifies half the dead time in number of clock cycles. Divide by frequency to get duration.
 *
 *  \todo Specify the dead time.
 */
#define DEAD_TIME_HALF    4

//! The number of elements in the sine modulation table per phase.
#define SINE_TABLE_LENGTH 192U

#if SINE_TABLE_LENGTH != 192U
#warning "Changing sine modulation table length can have unwanted side effects. Consult the documentation for more information."
#endif

//! The number of elements in the sine modulation table for each phase per commutation sector.
#define TABLE_ELEMENTS_PER_COMMUTATION_SECTOR   (SINE_TABLE_LENGTH / 6)

//! Bit pattern of PWM pins placed on PORTB.
#define PWM_PATTERN_PORTB   ((1 << PB1) | (1 << PB2) | (1 << PB3))

//! Bit pattern of PWM pins placed on PORTD.
#define PWM_PATTERN_PORTD   ((1 << PD3) | (1 << PD5) | (1 << PD6))

//! Forward direction flag value.
#define DIRECTION_FORWARD       0

//! Reverse direction flag value.
#define DIRECTION_REVERSE       1

//! Unknown direction flag value.
#define DIRECTION_UNKNOWN       3

/*! Uncomment this to disable internal pullup resistors on hall sensor inputs.
 *
 *  \todo Select whether internal pull-ups on hall sensor inputs should be
 *  enabled.
 */
#define HALL_PULLUP_ENABLE      FALSE

//! PIN register for Hall sensor input.
#define HALL_PIN      PINC

//! Pin where H1 is connected.
#define H1_PIN        PC0

//! Pin where H2 is connected.
#define H2_PIN        PC1

//! Pin where H3 is connected.
#define H3_PIN        PC2

//! The ADC channel where the analog speed reference is connected.
#define ADC_CHANNEL_SPEED_REF   3

//! The ADC channel where the motor current shunt resistor is connected.
#define ADC_CHANNEL_CURRENT     4

//! ADC clock prescaler 8 setting.
#define ADC_PRESCALER_8        ((0 << ADPS2) | (1 << ADPS1) | (0 << ADPS0))

//! ADC clock prescaler 64 setting.
#define ADC_PRESCALER_64        ((1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0))

//! ADC clock prescaler used in this application.
#define ADC_PRESCALER           ADC_PRESCALER_8

//! ADC internal voltage reference channel value.
#define ADC_REFERENCE_VOLTAGE_INTERNAL      ((1 << REFS1) | (1 << REFS0))

//! ADC VCC voltage reference channel value.
#define ADC_REFERENCE_VOLTAGE_VCC           ((0 << REFS1) | (1 << REFS0))

//! ADC AREF voltage reference channel value.
#define ADC_REFERENCE_VOLTAGE_AREF          ((0 << REFS1) | (0 << REFS0))

/*! ADC voltage reference used in this application.
 *
 *  \todo Select ADC voltage reference channel.
 */
#define ADC_REFERENCE_VOLTAGE               ADC_REFERENCE_VOLTAGE_INTERNAL

//! ADMUX settings for analog speed reference measurement.
#define ADMUX_SPEED_REF   (ADC_REFERENCE_VOLTAGE | (1 << ADLAR) | (ADC_CHANNEL_SPEED_REF << MUX0))

//! ADMUX settings for motor current measurement.
#define ADMUX_CURRENT     (ADC_REFERENCE_VOLTAGE | (1 << ADLAR) | (ADC_CHANNEL_CURRENT << MUX0))

/*! Pin where direction command input is located.
 *
 *  Do not change to pin on a different PORT!
 */
#define DIRECTION_COMMAND_PIN   PD2

/*! Pin where reverse rotation signal output is located.
 *
 *  Do not change to pin on a different PORT!
 */
#define REV_ROTATION_PIN        PD4

/*! Pin where tacho signal output is located.
 *
 *  Do not change to pin on a different PORT!
 */
#define TACHO_OUTPUT_PIN        PD7

/*! Emergency shutdown input pin
 *
 *  Do not change to pin on a different PORT!
 */
#define EMERGENCY_SHUTDOWN_PIN  PB5

//! Waveform constant for block commutation. Used as status flag.
#define WAVEFORM_BLOCK_COMMUTATION    0

//! Waveform status flag for sinusoidal driving.
#define WAVEFORM_SINUSOIDAL           1

//! Waveform status flag for braking.
#define WAVEFORM_BRAKING              2

//! Waveform status flag used in transitions between different types of driving.
#define WAVEFORM_UNDEFINED            3

/*! The number of commutation 'ticks' that must pass without any hall changes
 *  before the motor is considered to be stopped.
 *
 *  \todo Adjust the motor stopped limit.
 */
#define COMMUTATION_TICKS_STOPPED     6000

/*! Enables/disables the tacho signal output. TRUE = enabled, FALSE = disabled.
 *
 *  \todo Select whether the Tacho output signal should be enabled.
 */
#define TACHO_OUTPUT_ENABLED              TRUE

/*! Enables/disables the reverse rotation signal output. TRUE = enabled, FALSE = disabled.
 *
 *  \todo Select whether the 'Reverse rotation' signal should be enabled.
 */
#define REVERSE_ROTATION_SIGNAL_ENABLE    TRUE

//! TURN_MODE value for coasting (disabled drivers).
#define TURN_MODE_COAST               0

//! TURN_MODE value for braking (drivers connected to ground).
#define TURN_MODE_BRAKE               1

/*! Turn mode. Set to either TURN_MODE_COAST or TURN_MODE_BRAKE.
 *
 *  \todo Select turn mode.
 */
#define TURN_MODE                     TURN_MODE_BRAKE

//! The number to multiply speed input with to produce duty cycle compare value (0-255).
#define BLOCK_COMMUTATION_DUTY_MULTIPLIER   3

/*! This constant specifies the number of subsequent detections of correct
 *  direction of rotation needed before the firmware is considered synchronized
 *  with the motor. (SYNCHRONIZATION_COUNT + 1) hall sensor changes in the
 *  same direction are needed.
 */
#define SYNCHRONIZATION_COUNT       2

//! Speed control selection for open loop control.
#define SPEED_CONTROL_OPEN_LOOP     0

//! Speed control selection for closed loop control.
#define SPEED_CONTROL_CLOSED_LOOP   1

/*! Type of speed control. select either SPEED_CONTROL_OPEN_LOOP or
 *  SPEED_CONTROL_CLOSED_LOOP.
 *  \todo Select speed control method.
 */
#define SPEED_CONTROL_METHOD        SPEED_CONTROL_OPEN_LOOP

/*! The number of ticks between each iteration of the speed loop.
 *  \todo Adjust speed control loop time base.
 */
#define SPEED_CONTROLLER_TIME_BASE   150

/*! PID controller proportional gain constant.
 *  \todo Adjust PID controller proportional gain. (Only for closed loop)
 */
#define PID_K_P    50

/*! PID controller integral gain constant.
 *  \todo Adjust PID controller integral gain. (Only for closed loop)
 */
#define PID_K_I    5

/*! PID controller derivative gain constant.
 *  \todo Adjust PID controller derivative gain. (Only for closed loop)
 */
#define PID_K_D    10

/*! The maximum increment (maximum speed) to use as setpoint when the maximum
 *  speed reference value is input.
 *
 *  \todo Adjust maximum increment. (Maximum speed, used by speed controller)
 */
#define SPEED_CONTROLLER_MAX_INCREMENT      550

/*! Max speed reference input. (Rounded up to closest power of 2 in this case,
 *  which is recommended to speed up division.
 *
 *  \todo Adjust Maximum speed reference input value.
 */
#define SPEED_CONTROLLER_MAX_INPUT          256


//Typedefs
/*! \brief Collection of all motor control flags.
 *
 *  This struct contains all motor control flags used in this implementation.
 */
typedef struct PMSMflags
{
  uint8_t motorStopped : 1;     //! Is motor stopped?
  uint8_t motorSynchronized: 1; //! Is motor synchronized? Does not have any meaning when motorStopped is TRUE.
  uint8_t actualDirection : 2;  //! The actual direction of rotation.
  uint8_t desiredDirection : 1; //! The desired direction of rotation.
  uint8_t driveWaveform : 2;    //! The current waveform that should be produced.
} PMSMflags_t;


//Function prototypes
static void TimersInit(void);
static void TimersSetModeSinusoidal(void);
static void TimersSetModeBlockCommutation(void);
static void TimersSetModeBrake(void);
static void TimersWaitForNextPWMCycle(void);
static void BlockCommutationSetDuty(const uint8_t duty);
static void PortsInit(void);
static void PinChangeIntInit(void);
static void ADCInit(void);
static void CheckEmergencyShutdown(void);
static void SpeedController(void);
static uint8_t GetDesiredDirection(void);
static uint8_t GetActualDirection(void);
static void BlockCommutate(const uint8_t direction, uint8_t hall);
static uint8_t GetHall(void);
static void DesiredDirectionUpdate(void);
static uint8_t CalculateActualDirection(const uint8_t lastHall, const uint8_t newHall);
static void ActualDirectionUpdate(uint8_t lastHall, const uint8_t newHall);
static void ReverseRotationSignalUpdate(void);
static void InsertDeadband(const uint8_t compareValue, uint8_t * compareHighPtr, uint8_t * compareLowPtr);
static void AdjustSineTableIndex(const uint16_t increment);
static uint16_t SineTableIncrementCalculate(const uint16_t ticks);
static void SetAdvanceCommutation(const uint8_t leadAngle);
static void TachoOutputUpdate(const uint8_t hall);
static void EnablePWMOutputs(void);
static void DisablePWMOutputs(void);
static void CommutationTicksUpdate(void);
static void MotorSynchronizedUpdate(void);
static uint8_t IsMotorSynchronized(void);

/*! \mainpage
* \section intro Introduction
* This documents data structures, functions, variables, defines, enums, and
* typedefs in the software for application note AVR447.
*
* \section compinfo Compilation Info
* This software was written for the IAR Embedded Workbench, 4.11B.\n
* To make project:\n
* Add the main.c and pid.c files to project. Use device --cpu=m48, enable bit
* definitions in I/O include files, optimization should be high on speed for
* best performance but might need to be lowered for easier debugging, output
* format: ubrof8 for Debug and intel_extended for Release.
* Under "C/C++ compiler->Code" chose 9 registers locked for global variables.
*
* \section deviceinfo Device Info
* ATmega48/ATmega88/ATmega168 can be used. The example is
* written for ATmega48.
*
* \section note Note
*
* Most of the source code in this application note is placed in one file (main.c)
* to be able to take full advantage of the compiler's optimization.
*
* Please see the \ref todo for a list of changes that should be considered
* before running this firmware.
*
* \section contactinfo Contact Info
* For more info about Atmel AVR visit
* <A href="http://www.atmel.com/products/AVR/" >Atmel AVR</A> \n
* <A href="http://www.atmel.com/dyn/products/app_notes.asp?family_id=607"
  >AVR Application Notes</A>\n
* Support mail: avr@atmel.com
*/



#endif
