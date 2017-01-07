/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      PID controller header file.
 *
 *      This file contains #defines, typedefs and prototypes for the PID
 *      controller.
 *
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
 * $Revision: 1.3 $
 * $RCSfile: pid.h,v $
 * $Date: 2006/03/16 11:57:20 $  \n
 ******************************************************************************/

#ifndef PID_H
#define PID_H

/*! \brief Scaling division factor for PID controller.
 *
 *  Scaling division factor for the PID controller.
 *  The P, I and D gains will be divided by this number.
 */
#define SCALING_FACTOR  256


/*! \brief PID Status
 *
 * Setpoints and data used by the PID control algorithm
 */
typedef struct pidData{
  //! Last process value, used to find derivative of process value.
  int16_t lastProcessValue;
  //! Summation of errors, used for integrate calculations
  int32_t sumError;
  //! The Proportional tuning constant, given in x100
  int16_t P_Factor;
  //! The Integral tuning constant, given in x100
  int16_t I_Factor;
  //! The Derivative tuning constant, given in x100
  int16_t D_Factor;
  //! Maximum allowed error, avoid overflow
  int16_t maxError;
  //! Maximum allowed sumerror, avoid overflow
  int32_t maxSumError;
} pidData_t;


//! Maximum value of integers
#define MAX_INT         32767

//! Maximum value of long integers
#define MAX_LONG        2147483647L

/*! Maximum value of the I-term.
 *
 *  This limits the maximum positive or negative value of the I-term. Changing
 *  this value leads to a different integral anti-windup limit. Do not increase
 *  this value from its default, as it is already at the limit of the
 *  underlying data types.
 */
#define MAX_I_TERM      (MAX_LONG - (2 * (int32_t)MAX_INT))

// Boolean values
//! FALSE constant.
#define FALSE           0

//! TRUE constant.
#define TRUE            (!FALSE)


//Prototypes
void PID_Init(int16_t p_factor, int16_t i_factor, int16_t d_factor, pidData_t *pid);
int16_t PID_Controller(int16_t setPoint, int16_t processValue, pidData_t *pid_st);
void PID_Reset_Integrator(pidData_t *pid_st);

#endif
