/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Motor control implementation.
 *
 *      This file contains the full implementation of the motor control,
 *      except the PID-controller.
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
 * $Revision: 1.8 $
 * $RCSfile: main.c,v $
 * $Date: 2006/03/27 07:20:51 $  \n
 ******************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "PMSM.h"
#include "PMSMtables.h"

#if (SPEED_CONTROL_METHOD == SPEED_CONTROL_CLOSED_LOOP)
#include "pid.h"
#endif

/*! \brief Motor control flags placed in I/O space for fast access.
 *
 *  This variable contains all the flags used for motor control.
 *  It is placed in GPIOR0 register, which allows usage of several
 *  fast bit manipulation/branch instructions.
 */
// volatile PMSMflags_t fastFlags @0x1e;
#define fastFlags (*(volatile PMSMflags_t*)_SFR_MEM_ADDR(GPIOR0))

/*! \brief Increment used for sine table iteration.
 *
 *  This variable holds the increment used for sine table iteration.
 *  It is stored in an 8.8 fixed point format.
 */
register volatile uint16_t sineTableIncrement asm("r14");


/*! \brief Index into sine table
 *
 *  This variable is used as an index into the sine table. It is
 *  stored in a 8.8 fixed point format, so it can be directly incremented by
 *  sineTableIncrement. Only the high byte is used as table index.
 */
register volatile uint16_t sineTableIndex asm("r12");


/*! \brief The number of 'ticks' between two hall sensor changes.
 *
 *  This variable is used to count the number of 'ticks' between each hall
 *  sensor change. One 'tick' is one PWM period, 510 CPU clock cycles. This is
 *  used to calculate sine table increment. The speed of the motor is inversely
 *  proportional to this value.
 */
register volatile uint16_t commutationTicks asm("r10");


/*! \brief The amplitude of the generated sine waves.
 *
 *  This variable controls the amplitude of the generated sine waves.
 *  The range is 0-255.
 */
register volatile uint8_t amplitude asm("r9");


/*! \brief The lead angle or advance commutation angle
 *
 *  This variable specifies a shift in the sine table that will
 *  generate advance commutaiton. For a 192 element sine table, an increase
 *  of one in advanceCommutationSteps will result in 1.875 degrees lead
 *  angle.
 */
register volatile uint8_t advanceCommutationSteps asm("r8");


/*! \brief Index to the end of the current commutation sector
 *
 *  This variable holds the index where the next commutation sector starts,
 *  including the advance commutation angle.
 *
 *  It is used to prevent the output from going further until the hall sensors
 *  changes.
 */
register volatile uint8_t sineTableNextSectorStart asm("r7");


/*! \brief Current measurement
 *
 *  The most recent current measurement is stored in this variable. It is not
 *  used for any purpose in this implementation, but the measurement is
 *  updated.
 */
volatile uint8_t current;


//! The most recent speed input measurement.
volatile uint8_t speedInput;

/*! \brief Speed controller run flag.
 *
 *  This variable is set to TRUE every time the speed controller should be run.
 */
volatile uint8_t SpeedControllerRun = FALSE;


#if (SPEED_CONTROL_METHOD == SPEED_CONTROL_CLOSED_LOOP)
//! Struct used to hold PID controller parameters and variables.
pidData_t pidParameters;
#endif


/*! \brief Main function / initialization
 *
 *  The main function initializes all subsystems needed for motor control
 *  and enables interrupts, which kicks off the fully interrupt-driven
 *  motor control. The main function goes into an eternal loop where it
 *  does nothing.
 */
void main(void)
{
  //Initialize peripherals.
  PortsInit();
  TimersInit();
  PinChangeIntInit();
  ADCInit();

  SetAdvanceCommutation(0);

#if (SPEED_CONTROL_METHOD == SPEED_CONTROL_CLOSED_LOOP)
  PID_Init(PID_K_P, PID_K_I, PID_K_D, &pidParameters);
#endif

  //Do not start until everything is ready.
  CheckEmergencyShutdown();

  //Initialize fastflags. Use temporary variable to avoid several accesses to
  //volatile variable.
  {
    PMSMflags_t fastFlagsInitial;

    // Set motorStopped to FALSE at startup. This will make sure that the motor
    // is not started if it is not really stopped. If it is stopped, this variable
    // will quickly be updated.
    fastFlagsInitial.motorStopped = FALSE;

    // Set motorSyncronized to FALSE at startup. This will prevent the motor from being
    // driven until the motor is in synch or stopped.
    fastFlagsInitial.motorSynchronized = FALSE;
    fastFlagsInitial.actualDirection = DIRECTION_UNKNOWN;
    fastFlagsInitial.desiredDirection = 0;
    fastFlagsInitial.driveWaveform = WAVEFORM_UNDEFINED;

    fastFlags = fastFlagsInitial;
  }

  DesiredDirectionUpdate();

  // Enable Timer1 capture event interrupt.
  TIMSK1 |= (1 << ICIE1);

  //Enable interrupts globally and let motor driver take over.
  sei();
  for(;;)
  {
    if (SpeedControllerRun)
    {
      SpeedController();
      SpeedControllerRun = FALSE;
    }
  }
}


/*! \brief Initializes I/O port directions and pull-up resistors
 *
 *  This function initializes all I/O ports with correct
 *  direction and pull-up resistors, if needed.
 */
static void PortsInit(void)
{
#if (HALL_PULLUP_ENABLE == TRUE)
  // Set hall sensor pins as input, pullups enabled.
  PORTC = (1 << H1_PIN) | (1 << H2_PIN) | (1 << H3_PIN);
#endif

  // PORTD outputs
  DDRD = (1 << REV_ROTATION_PIN) | (1 << TACHO_OUTPUT_PIN);

  //Enable pull-up on direction signal.
  PORTD |= (1 << DIRECTION_COMMAND_PIN);
}


/*! \brief Initializes and synchronizes Timers
 *
 *  This function sets the correct prescaler and starts all
 *  three timers. The timers are synchronized to ensure that
 *  all PWM signals are aligned.
 */
static void TimersInit(void)
{
  //Set all timers in "Phase correct mode". Do not enable outputs yet.
  TCCR0A = (1 << WGM00);
  TCCR1A = (1 << WGM11);
  TCCR2A = (1 << WGM20);

  //Set top value of Timer/counter1.
  ICR1 = 0xff;

  //Synchronize timers
  TCNT0 = 0;
  TCNT1 = 2;
  TCNT2 = 4;

  // Start all 3 timers.
  TCCR0B = (0 << CS01) | (1 << CS00);
  TCCR1B = (1 << WGM13) | (0 << CS11) | (1 << CS10);
  TCCR2B = (0 << CS21) | (1 << CS20);
}


/*! \brief Initialize pin change interrupts.
 *
 *  This function initializes pin change interrupt on hall
 *  sensor input pins input, emergency shutdown
 *  input and motor direction control input.
 */
static void PinChangeIntInit(void)
{
  // Initialize pin change interrupt on emergency shutdown pin.
  PCMSK0 = (1 << PCINT5);

  // Initialize pin change interrupt on hall sensor inputs.
  PCMSK1 = (1 << PCINT10) | (1 << PCINT9) | (1 << PCINT8);

  // Initialize pin change interrupt on direction input pin.
  PCMSK2 = (1 << PCINT18);

  // Enable pin change interrupt on ports with pin change signals
  PCICR = (1 << PCIE2) | (1 << PCIE1) | (1 << PCIE0);
}


/*! \brief Initializes the ADC
 *
 *  This function initializes the ADC for speed reference measurements.
 *  The ADC will operate in free-running mode, and reading is done in the
 *  ADC complete interrupt service routine.
 */
static void ADCInit(void)
{
  //Select initial AD conversion channel.
  ADMUX = ADMUX_SPEED_REF;

  //Initialize ADC.
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIF) | (1 << ADIE) | ADC_PRESCALER;

  //Set trigger source to Timer/Counter0 overflow.
  ADCSRB = (1 << ADTS2) | (0 << ADTS1) | (0 << ADTS0);
}


/*! \brief Checks whether emergency shutdown is set at startup.
 *
 *  This function checks whether emergency shut-off signal is
 *  set at startup and takes action accordingly. This function is only needed
 *  at startup, since pin change interrupts are enabled for these signals.
 *
 *  \note In this version, nothing is done in these situations.
 *
 *  \todo Write code to handle a situation where either emergency shut-off
 *  set at startup.
 */
static void CheckEmergencyShutdown(void)
{
  if ( (PINB & (1 << EMERGENCY_SHUTDOWN_PIN)) != 0)
  {
    //Insert code here to handle emercency shutdown signal at startup.
  }
}


/*! \brief  Speed regulator loop.
 *
 *  This function is called every SPEED_REGULATOR_TIME_BASE ticks. In this
 *  implementation, a simple PID controller loop is called, but this function
 *  could be replaced by any speed (or other regulator).
 */
static void SpeedController(void)
{
#if (SPEED_CONTROL_METHOD == SPEED_CONTROL_CLOSED_LOOP)

  //Calculate an increment setpoint from the analog speed input.
  int16_t incrementSetpoint = ((uint32_t)speedInput * SPEED_CONTROLLER_MAX_INCREMENT) / SPEED_CONTROLLER_MAX_INPUT;

  //PID regulator with feed forward from speed input.
  int32_t outputValue;
  outputValue = (uint32_t)speedInput;
  outputValue += PID_Controller(incrementSetpoint, (int16_t)sineTableIncrement, &pidParameters);

  if (outputValue < 0)
  {
    outputValue = 0;
  }
  else if (outputValue > 255)
  {
    outputValue = 255;
  }

  amplitude = outputValue;
#else
  amplitude = speedInput;
#endif
}


/*! \brief Configures timers for sine wave generation
 *
 *  This function is called every time sine wave generation is
 *  needed. PWM outputs are safely disabled while configuration
 *  registers are changed to avoid unintended driving or shoot-
 *  through.
 */
#pragma inline=forced
static void TimersSetModeSinusoidal(void)
{
  //Set PWM pins to input (Hi-Z) while changing modes.
  DisablePWMOutputs();

  //Sets all 3 timers in inverted pair mode.
  TCCR0A = (1 << COM0A1) | (0 << COM0A0) | (1 << COM0B1) | (1 << COM0B0) | (1 << WGM00);
  TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (1 << COM1B1) | (1 << COM1B0) | (1 << WGM11);
  TCCR2A = (1 << COM2A1) | (0 << COM2A0) | (1 << COM2B1) | (1 << COM2B0) | (1 << WGM20);

  //Make sure all outputs are turned off before PWM outputs are enabled.
  OCR0A = OCR1AL = OCR2A = 0;
  OCR0B = OCR1BL = OCR2B = 0xff;

  //Wait for next PWM cycle to ensure that all outputs are updated.
  TimersWaitForNextPWMCycle();

  fastFlags.driveWaveform = WAVEFORM_SINUSOIDAL;

  //Change PWM pins to output again to allow PWM control.
  EnablePWMOutputs();
}


/*! \brief Configures timers for block commutation.
 *
 *  This function is called every time block commutation is
 *  needed. PWM outputs are safely disabled while configuration
 *  registers are changed to avoid unintended driving or shoot-
 *  through.
 */
#pragma inline=forced
static void TimersSetModeBlockCommutation(void)
{
  //Set PWM pins to input (Hi-Z) while changing modes.
  DisablePWMOutputs();

  //Sets both outputs of all 3 timers in "clear on up-counting, set on down-counting" mode.
  TCCR0A = (1 << COM0A1) | (0 << COM0A0) | (1 << COM0B1) | (0 << COM0B0) | (1 << WGM00);
  TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (1 << COM1B1) | (0 << COM1B0) | (1 << WGM11);
  TCCR2A = (1 << COM2A1) | (0 << COM2A0) | (1 << COM2B1) | (0 << COM2B0) | (1 << WGM20);

  //Set output duty cycle to zero for now.
  BlockCommutationSetDuty(0);

  //Wait for next PWM cycle to ensure that all outputs are updated.
  TimersWaitForNextPWMCycle();

  fastFlags.driveWaveform = WAVEFORM_BLOCK_COMMUTATION;

  //Change PWM pins to output again to allow PWM control.
  EnablePWMOutputs();
}


/*! \brief Configures timers for braking and starts braking.
 *
 *  This function configures the timers for braking and starts braking. Please
 *  note that braking when turning can produce too much heat for the drivers to
 *  handle. Use with care!
 */
#if (TURN_MODE == TURN_MODE_BRAKE)
#pragma inline=forced
static void TimersSetModeBrake(void)
{
  //Set PWM pins to input (Hi-Z) while changing modes.
  DisablePWMOutputs();

  //Sets both outputs of all 3 timers in "clear on up-counting, set on down-counting" mode.
  TCCR0A = (1 << COM0A1) | (0 << COM0A0) | (1 << COM0B1) | (0 << COM0B0) | (1 << WGM00);
  TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (1 << COM1B1) | (0 << COM1B0) | (1 << WGM11);
  TCCR2A = (1 << COM2A1) | (0 << COM2A0) | (1 << COM2B1) | (0 << COM2B0) | (1 << WGM20);

  // High side
  OCR0A = OCR1A = OCR2A = 0x00;

  // Low side
  OCR0B = OCR1B = OCR2B = 0xff;

  //Wait for next PWM cycle to ensure that all outputs are updated.
  TimersWaitForNextPWMCycle();

  fastFlags.driveWaveform = WAVEFORM_BRAKING;

  EnablePWMOutputs();
}
#endif


/*! \brief Waits for the start of the next PWM cycle.
 *
 *  Waits for the start of the next PWM cycle.
 *  Can be used to make sure that a shoot-through
 *  does not occur in the transition between two output waveform generation modes.
 */
#pragma inline=forced
static void TimersWaitForNextPWMCycle(void)
{
  //Clear Timer1 Capture event flag.
  TIFR1 = (1 << ICF1);

  //Wait for new Timer1 Capture event flag.
  while ( !(TIFR1 & (1 << ICF1)) )
  {

  }
}


/*! \brief Sets duty cycle for block commutation
 *
 *  This function sets duty cycle when block commutation is used.
 *  Never call this function when sine wave generation is used.
 *  \note The duty cycle is not in percent, but a value from
 *  0-255. Duty cycle in percent can be calculated as
 *  duty * 100 / 255.
 *
 *  \param duty New duty cycle. Range is 0-255.
 */
#pragma inline=forced
static void BlockCommutationSetDuty(const uint8_t duty)
{
  // Set all compare registers to new duty cycle value.
  OCR0A = OCR0B = OCR1AL = OCR1BL = OCR2A = OCR2B = duty;
}


/*! \brief Returns the desired direction.
 *
 *  This function returns the current desired direction.
 *  \note The direction input is not read at this time. A
 *  separate pin change interrupt is responsible for reading
 *  the input.
 *
 *  \retval DIRECTION_FORWARD Forward direction is requested.
 *  \retval DIRECTION_REVERSE Reverse direction is requested.
 */
#pragma inline=forced
static uint8_t GetDesiredDirection(void)
{
  return (uint8_t)fastFlags.desiredDirection;
}


/*! \brief Returns the actual direction of the motor.
 *
 *  This function returns the actual direction of the motor.
 *
 *  \note Two hall changes in the same direction is needed
 *  before this function returns the correct direction of
 *  rotation.
 *
 *  \return The desired direction.
 *
 *  \retval DIRECTION_FORWARD Motor is running forward
 *  \retval DIRECTION_REVERSE Motor is running in reverse
 *  \retval DIRECTION_UNKNOWN The current direction of the motor
 *  can not be determined (may be stopped or turning), or the
 *  hall sensors output incorrect values.
 */
#pragma inline=forced
static uint8_t GetActualDirection(void)
{
  return fastFlags.actualDirection;
}


/*! \brief Performs block commutation according to running direction and hall sensor input
 *
 *  This function performs a block commutation according to the
 *  specified running direction and hall sensor input.
 *
 *  \note Do not use this function while the timers are configured
 *  for sine wave driving.
 *
 *  \param direction Direction of rotation.
 *  \param hall Hall sensor value at the same form as returned by GetHall().
 */
#pragma inline=forced
static void BlockCommutate(const uint8_t direction, uint8_t hall)
{
  uint8_t const __flash *pattern;

  if (direction == DIRECTION_FORWARD)
  {
    pattern = blockCommutationTableForward;
  }
  else
  {
    pattern = blockCommutationTableReverse;
  }
  pattern += (hall * 2);

  DisablePWMOutputs();
  DDRB |= *pattern++;
  DDRD |= *pattern;
}


/*! \brief Reads the hall sensor inputs.
 *
 *  This function reads the hall sensor inputs and converts it to
 *  a number from 1 to 6 by combining the hall sensors as bits: H3|H2|H1.
 *
 *  \note It is possible to change the physical placement of hall sensor
 *  inputs, but the recommended configuration is the one used in this
 *  example, since it requires very little code to decode the hall values.
 *
 *  \return The decoded hall sensor value.
 *
 *  \retval 0 Illegal hall state. Possible hardware error.
 *  \retval 1-6 Legal hall sensor values.
 *  \retval 7 Illegal hall state. Possible hardware error.
 */
#pragma inline=forced
static uint8_t GetHall(void)
{
  uint8_t hall;

  hall = PINC & ((1 << H3_PIN) | (1 << H2_PIN) | (1 << H1_PIN));
  hall >>= H1_PIN;
  return hall;
}


/*! \brief Updates global desired direction flag.
 *
 *  Running this function triggers a reading of the direction
 *  input pin. The desiredDirection flag is set accordingly.
 */
#pragma inline=forced
static void DesiredDirectionUpdate(void)
{
  if ( (PIND & (1 << DIRECTION_COMMAND_PIN)) != 0 )
  {
    fastFlags.desiredDirection = DIRECTION_REVERSE;
  }
  else
  {
    fastFlags.desiredDirection = DIRECTION_FORWARD;
  }
}


/*! \brief Updates global actual direction flag based on the two latest hall values.
 *
 *  Calling this function with the last two hall sensor values as
 *  parameters triggers an update of the global actualDirection flag.
 *
 *  \param lastHall The last hall value.
 *  \param newHall The current hall value.
 */
#pragma inline=forced
static void ActualDirectionUpdate(uint8_t lastHall, const uint8_t newHall)
{
  //Make sure that lastHall is within bounds of table. If not, set to 0, which is
  //also an illegal hall value, but legal table index.
  if (lastHall > 6)
  {
    lastHall = 0;
  }
  if (expectedHallSequenceForward[lastHall] == newHall)
  {
    fastFlags.actualDirection = DIRECTION_FORWARD;
  }
  else if (expectedHallSequenceReverse[lastHall] == newHall)
  {
   fastFlags.actualDirection = DIRECTION_REVERSE;
  }
  else
  {
    PMSMflags_t tempFlags = fastFlags;
    tempFlags.actualDirection = DIRECTION_UNKNOWN;
    tempFlags.motorSynchronized = FALSE;
    fastFlags = tempFlags;
  }
}


/*! \brief Updates the reverse rotation signal output pin value.
 *
 *  Running this function compares the actual and desired direction
 *  flags to decide if the motor is running in the opposite direction
 *  of what is requested. The Reverse rotation pin will be set to high
 *  when the motor is running in the opposite direction and low when
 *  the motor is running in the correct direction.
 *
 *  \note This function does not update actual and desired direction
 *  flags. The result of this function will therefore represent the
 *  time at which these variables were updated.
 */
#pragma inline=forced
static void ReverseRotationSignalUpdate(void)
{
#if (REVERSE_ROTATION_SIGNAL_ENABLE)
  if (GetActualDirection() == GetDesiredDirection())
  {
    PORTD &= ~(1 << REV_ROTATION_PIN);
  }
  else
  {
    PORTD |= (1 << REV_ROTATION_PIN);
  }
#endif
}


/*! \brief Returns the high and low values with deadband for a given compare value.
 *
 * This function takes as argument a desired compare value and inserts a symmetric
 * deadband. The compare values for high and low side with deadband are returned
 * through the two supplied pointers.
 * The constant \ref DEAD_TIME_HALF is used as deadband, and the resulting deadtime
 * will be DEAD_TIME_HALF clock cycles times 2.
 *
 * \param compareValue desired compare value
 * \param compareHighPtr Pointer used to return high side compare value with dead band.
 * \param compareLowPtr  Pointer used to return low side compare value with dead band.
 */
#pragma inline=forced
static void InsertDeadband(const uint8_t compareValue, uint8_t * compareHighPtr, uint8_t * compareLowPtr)
{
  if (compareValue <= DEAD_TIME_HALF)
  {
    *compareHighPtr = 0x00;
    *compareLowPtr = compareValue;
  }
  else if (compareValue >= (0xff - DEAD_TIME_HALF))
  {
    *compareHighPtr = 0xff - (2 * DEAD_TIME_HALF);
    *compareLowPtr = 0xff;
  }
  else
  {
    *compareHighPtr = compareValue - DEAD_TIME_HALF;
    *compareLowPtr = compareValue + DEAD_TIME_HALF;
  }
}


/*! \brief Calculates the increment for sine table iteration.
 *
 *  This function calculates the increment to be used for sine
 *  table iteration, based on the number of 'ticks' between two
 *  consecutive hall changes. Since a divide is needed, which is
 *  not supported in hardware on the AVR, the divide is done through
 *  a lookup table for values below 256 (when the motor is running
 *  fastest).
 *
 *  \param ticks The number of ticks between two consecutive hall changes.
 *
 *  \return The increment in 8.8 format to be used for sine table iteration.
 */
#pragma inline = forced
static uint16_t SineTableIncrementCalculate(const uint16_t ticks)
{
  if (ticks < 256)
  {
    return divisionTable[(uint8_t)ticks];
  }
  return (uint16_t)(((SINE_TABLE_LENGTH / 6) << 8) / ticks);
}


/*! \brief Adjusts the sine table index according to the current increment.
 *
 *  This function increases the sine table index with the given increment
 *  The index is then adjusted to be within the table length.
 *
 *  \param increment The increment (in 8.8 format) added to the sine table index.
 */
#pragma inline=forced
static void AdjustSineTableIndex(const uint16_t increment)
{
  sineTableIndex += increment;

  // If the table index is out of bounds, wrap the index around the table end
  // to continue from the beginning. Also wrap the next sector start index.
  if ((sineTableIndex >> 8) >= SINE_TABLE_LENGTH)
  {
    sineTableIndex -= (SINE_TABLE_LENGTH << 8);
    sineTableNextSectorStart -= SINE_TABLE_LENGTH;
  }

  //Make copy of sineNextSectorStart to specify order of volatile access.
  uint8_t nextSectorStart = sineTableNextSectorStart;
  if ((sineTableIndex >> 8) > nextSectorStart)
  {
    sineTableIndex = (nextSectorStart << 8);
  }
}


/*! \brief Sets the lead angle.
 *
 *  This function sets the advance commutation angle to be used during sine
 *  wave operation. An increase of one equals 1.875 degrees.
 *
 *  \note There is no checking of the advanceCommutation input parameter, but this
 *  should not be set to a value above 40 to avoid overflow in the sine table
 *  index.
 *
 *  \param advanceCommutation The advance commutation (1 equals 1.875 degrees)
 */
#pragma inline=forced
static void SetAdvanceCommutation(const uint8_t advanceCommutation)
{
  advanceCommutationSteps = advanceCommutation;
}


/*! \brief Updates the tacho output pin signal
 *
 *  This function uses the current hall sensor value to determine the tacho
 *  output signal. The "algorithm" used is based on the fact that this signal
 *  should have a 0 value when the combined hall sensor inputs are a power of 2.
 *
 *  The expression (hall & (hall - 1) computes to 0 for all values of 'hall' that
 *  is a power of 2. This can be used to produce the correct output signal.
 *
 *  \param hall The current hall sensor value.
 */
#pragma inline=forced
static void TachoOutputUpdate(const uint8_t hall)
{
#if (TACHO_OUTPUT_ENABLED)
  if ( (hall & (uint8_t)(hall - 1)) != 0 )
  {
    PORTD &= ~(1 << TACHO_OUTPUT_PIN);
  }
  else
  {
    PORTD |= (1 << TACHO_OUTPUT_PIN);
  }
#endif
}


/*! \brief Enables PWM output pins
 *
 *  This function enables PWM outputs by setting the port direction for
 *  all PWM pins as output. The PWM configuration itself is not altered
 *  in any way by running this function.
 */
#pragma inline=forced
static void EnablePWMOutputs(void)
{
  DDRB |= PWM_PATTERN_PORTB;
  DDRD |= PWM_PATTERN_PORTD;
}


/*! \brief Disables PWM output pins
 *
 *  This function disables PWM outputs by setting the port dierction for
 *  all PWM pins as inputs, thus overriding the PWM. The PWM configuration
 *  itself is not altered in any way by running this function.
 */
#pragma inline=forced
static void DisablePWMOutputs(void)
{
  DDRB &= ~PWM_PATTERN_PORTB;
  DDRD &= ~PWM_PATTERN_PORTD;
}


/*! \brief Updates the 'tick' counter and checks for stopped motor.
 *
 *  This function should be run at every PWM timer overflow to ensure
 *  that all 'ticks' are counted. It increases the 'tick' counter
 *  until it reaches the maximum tick limit that corresponds to what
 *  is considered a stopped or stalled motor. In that case, the global
 *  motor stopped flag is set.
 */
#pragma inline=forced
static void CommutationTicksUpdate(void)
{
  if (commutationTicks < COMMUTATION_TICKS_STOPPED)
  {
    commutationTicks++;
  }
  else
  {
    fastFlags.motorStopped = TRUE;
    fastFlags.motorSynchronized = FALSE;
    if (fastFlags.driveWaveform != WAVEFORM_BLOCK_COMMUTATION)
    {
      TimersSetModeBlockCommutation();
      BlockCommutate(GetDesiredDirection(), GetHall());

#if (SPEED_CONTROL_METHOD == SPEED_CONTROL_CLOSED_LOOP)
      PID_Reset_Integrator(&pidParameters);
#endif
    }
  }
}


/*! \brief Updates the global motor synchronized flag.
 *
 *  This function updates the global motor synchronized flag. The motor control
 *  is considered to be synchronized with the motor when it is not stopped and
 *  the driver has detected a direction of rotation that corresponds to the
 *  desired direction a predefined number of times.
 */
#pragma inline=forced
static void MotorSynchronizedUpdate(void)
{
  static uint8_t synchCount = 0;

  PMSMflags_t tempFlags;

  tempFlags = fastFlags;

  if ((tempFlags.desiredDirection == tempFlags.actualDirection) &&
      (tempFlags.motorStopped == FALSE) && (tempFlags.motorSynchronized == FALSE))
  {
    synchCount++;
    if (synchCount >= SYNCHRONIZATION_COUNT)
    {
      fastFlags.motorSynchronized = TRUE;
    }
  }
  else
  {
    fastFlags.motorSynchronized = FALSE;
    synchCount = 0;
  }
}


/*! \brief Returns the motor synchronized flag.
 *
 *  This function returns the motor synchronized flag.
 *
 *  \return The motor synchronized flag.
 *
 *  \retval TRUE  Motor control is synchronized with motor.
 *  \retval FALSE Motor control is not yet synchronized with motor.
 */
#pragma inline=forced
static uint8_t IsMotorSynchronized(void)
{
  return (uint8_t)(fastFlags.motorSynchronized);
}


/*! \brief Emergency shut-off interrupt service routine.
 *
 *  This ISR is triggered if the emergency shutdown input changes state.
 *
 *  \note In the current implementation, the motor driver outputs are
 *  simply disabled and the program goes into an eternal loop on any
 *  change at this input. Custom code should be made here that handles
 *  such situation in a more intelligent way, adapted to the motor and
 *  driver stage.
 *
 *  \todo Change how emergency shut-off is handled.
 */
 
ISR(PCINT0_vect)
{
  DisablePWMOutputs();
  for (;;)
  {

  }
}


/*! \brief Hall sensor change interrupt service routine.
 *
 *  This interrupt service routine is called every time any of the hall
 *  sensors change. The sine table index is updated to reflect the
 *  position indicated by the hall sensors.
 *
 *  A new increment is calculated, based on the time since last hall
 *  sensor change.
 *
 *  The Tacho output signal is updated.
 *
 *  The actual direction is updated.
 *
 *  The reverse rotation output signal is updated.
 *
 *  The motor stopped flag is set to false, since the motor is obviously not
 *  stopped when there is a hall change.
 */
ISR(PCINT1_vect)
{
  static uint8_t lastHall = 0xff;
  uint8_t hall;

  hall = GetHall();

  MotorSynchronizedUpdate();
  uint8_t synch = IsMotorSynchronized();
  if ((fastFlags.driveWaveform != WAVEFORM_SINUSOIDAL) && (synch))
  {
    TimersSetModeSinusoidal();
  }

  //If sinusoidal driving is used, synchronize sine wave generation to the
  //current hall sensor value. Advance commutation is also
  //added in the process.
  if (fastFlags.driveWaveform == WAVEFORM_SINUSOIDAL)
  {
    uint16_t tempIndex;
    if (GetDesiredDirection() == DIRECTION_FORWARD)
    {
      tempIndex = (CSOffsetsForward[hall] + advanceCommutationSteps) << 8;
    }
    else
    {
      tempIndex = (CSOffsetsReverse[hall] + advanceCommutationSteps) << 8;
    }
    sineTableIndex = tempIndex;

    //Adjust next sector start index. It might be set to a value larger than
    //SINE_TABLE_LENGTH at this point. This is adjusted in AdjustSineTableIndex
    //and should not be done here, as it will cause problems when advance
    //commutation is used.
    sineTableNextSectorStart = (tempIndex >> 8) + TABLE_ELEMENTS_PER_COMMUTATION_SECTOR;
  }

  //If block commutation is used. Commutate according to hall signal.
  else if (fastFlags.driveWaveform == WAVEFORM_BLOCK_COMMUTATION)
  {
    BlockCommutate(GetDesiredDirection(), hall);
  }

  //Update internal and external signals that depend on hall sensor value.
  TachoOutputUpdate(hall);
  ActualDirectionUpdate(lastHall, hall);
  ReverseRotationSignalUpdate();

  lastHall = hall;

  //Calculate new increment for sine wave generation and reset commutation
  //timer.
  sineTableIncrement = SineTableIncrementCalculate(commutationTicks);
  commutationTicks = 0;

  //Since the hall sensors are changing, the motor can not be stopped.
  fastFlags.motorStopped = FALSE;
}


/*! \brief Direction input change interrupt service routine.
 *
 *  This ISR is called every time the direction input pin changes
 *  state. The desired direction flag is updated accordingly. The
 *  motor control then goes into a state where it needs a stopped
 *  motor or a synchronization before any driving of the motor is
 *  performed.
 */
ISR(PCINT2_vect)
{
  //Update desired direction flag.
  DesiredDirectionUpdate();

#if (TURN_MODE == TURN_MODE_COAST)
  //Disable driver signals to let motor coast. The motor will automatically
  //start once it is synchronized or stopped.
  DisablePWMOutputs();
  fastFlags.motorSynchronized = FALSE;
  fastFlags.motorStopped = FALSE;
  fastFlags.driveWaveform = WAVEFORM_UNDEFINED;
#endif

#if (TURN_MODE == TURN_MODE_BRAKE)
  //Set motor in brake mode. The motor will automatically start once it is
  //synchronized or stopped.
  fastFlags.motorSynchronized = FALSE;
  fastFlags.motorStopped = FALSE;
  TimersSetModeBrake(); // Automatically sets driveWaveform.
#endif
}


/*! \brief Timer1 Capture Evente interrupt service routine.
 *
 * This interrupt service routine is run everytime the up-down counting timer0
 * reaches TOP (0xff). New sinusoidal output values are calculated and the
 * timers are updated to reflect the new values.
 */
ISR(TIMER1_CAPT_vect)
{
  if (fastFlags.driveWaveform == WAVEFORM_SINUSOIDAL)
  {
    uint8_t tempU, tempV, tempW;
    {
      uint8_t const __flash * sineTablePtr = sineTable;

      AdjustSineTableIndex(sineTableIncrement);

      //Add sine table offset to pointer. Must be multiplied by 3, since one
      //value for each phase is stored in the table.
      sineTablePtr += (sineTableIndex >> 8) * 3;

      tempU = *sineTablePtr++;
      if (GetDesiredDirection() == DIRECTION_FORWARD)
      {
        tempV = *sineTablePtr++;
        tempW = *sineTablePtr;
      }
      else
      {
        tempW = *sineTablePtr++;
        tempV = *sineTablePtr;
      }
    }

    //Scale sine modulation values to the current amplitude.
    tempU = ((uint16_t)(amplitude * tempU) >> 8);
    tempV = ((uint16_t)(amplitude * tempV) >> 8);
    tempW = ((uint16_t)(amplitude * tempW) >> 8);

    {
      uint8_t compareHigh, compareLow;


      InsertDeadband(tempU, &compareHigh, &compareLow);
      OCR0A = compareHigh;
      OCR0B = compareLow;

      InsertDeadband(tempV, &compareHigh, &compareLow);
      OCR1AL = compareHigh;
      OCR1BL = compareLow;

      InsertDeadband(tempW, &compareHigh, &compareLow);
      OCR2A = compareHigh;
      OCR2B = compareLow;
    }
  }
  else if (fastFlags.driveWaveform == WAVEFORM_BLOCK_COMMUTATION)
  {
    uint16_t blockCommutationDuty = amplitude * BLOCK_COMMUTATION_DUTY_MULTIPLIER;

    if (blockCommutationDuty > 255)
    {
      blockCommutationDuty = 255;
    }

    BlockCommutationSetDuty((uint8_t)blockCommutationDuty);
  }

  CommutationTicksUpdate();

  {
    //Run the speed regulation loop with constant intervals.
    static uint8_t speedRegTicks = 0;
    speedRegTicks++;
    if (speedRegTicks >= SPEED_CONTROLLER_TIME_BASE)
    {
      SpeedControllerRun = TRUE;
      speedRegTicks -= SPEED_CONTROLLER_TIME_BASE;
    }
  }
}


/*! \brief AD conversion complete interrupt service routine
 *
 *  This interrupt service routine is automatically executed every time
 *  an AD conversion is finished and the converted result is available
 *  in the ADC data register.
 *
 *  The switch/case construct makes sure the converted value is stored in the
 *  variable corresponding to the selected channel and changes the channel for
 *  the next ADC measurement.
 *
 *  More ADC measurements can be added to the cycle by extending the switch/
 *  case construct.
 *
 *  Only the 8 most significant bits of the ADC result are used.
 */
ISR(ADC_vect)
{
  switch (ADMUX)
  {
  case ADMUX_SPEED_REF:
    speedInput = ADCH;
    ADMUX = ADMUX_CURRENT;
    break;
  case ADMUX_CURRENT:
    current = ADCH;
    ADMUX = ADMUX_SPEED_REF;
    break;
  default:
    //This is probably an error and should be handled.
    break;
  }

  //Clear Timer/counter0 overflow flag.
  TIFR0 = (1 << TOV0);
}
