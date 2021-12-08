/**************************************************************************
* File Name             PIDcmd.c
* Description           
*				          
* Date				          Name(s)						          Action
* November 17, 2021		  David C. & Jaskaran K.			First Implementation
***************************************************************************/

/**************************************************************************
---------------------------- LIBRARY DEFINITIONS --------------------------
***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>

#include "common.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_pwr.h"
#include "stm32f4xx_hal_rcc_ex.h"
#include "stm32f4xx_hal_pwr_ex.h"
#include "stm32f4xx_hal_cortex.h"

/**************************************************************************
--------------------------- PRECOMPILER DEFINITIONS -----------------------
***************************************************************************/
#define DC_MOTOR_IN1_PIN GPIO_PIN_0
#define DC_MOTOR_IN2_PIN GPIO_PIN_1
#define DC_MOTOR_ENABLE_PIN GPIO_PIN_8

#define DC_MOTOR_PORT GPIOA

#define PID_P 2.0
#define PID_I 2.0
#define PID_D 1.0

/**************************************************************************
------------------------------- VARIABLE TYPES ----------------------------
***************************************************************************/

typedef enum {
  INACTIVE, ACTIVE  
}STATE;

typedef struct {

  double P,I,D;  // constants to use
  int32_t encoderCurrent, encoderPrevious;
  double targetRPM;
  double errorCurrent, errorPrevious, errorI, errorD;
  STATE status; // status of the PID
  double PIDResult;

}PID_CONTEXT;

/**************************************************************************
---------------------------- GLOBAL VARIABLES --------------------------
***************************************************************************/
TIM_HandleTypeDef tim3;

volatile PID_CONTEXT pid;

const uint16_t period = 20000;

const uint16_t periodEncoder = 0xFFFF;
const double timebase = (1.0e-5) * period; 

/*Motor characteristics (BLUE LED) */
const double unitsPerEncoderTurn = 4.0;
const double gearsRatio = 29.47;
const double encoderFullTurn = unitsPerEncoderTurn * gearsRatio;
const double pid2pwm = 1.0; 

volatile uint16_t currentPWM = period/2; 

TIM_HandleTypeDef tim1;  // Timer Handler 

uint16_t pins[] = {DC_MOTOR_IN1_PIN, DC_MOTOR_IN2_PIN, DC_MOTOR_ENABLE_PIN};
GPIO_TypeDef* ports[] = {DC_MOTOR_PORT, DC_MOTOR_PORT, DC_MOTOR_PORT};


/**************************************************************************
------------------------ OWN FUNCTION DEFINITIONS -------------------------
***************************************************************************/

void InitializePID(){

  pid.P = PID_P;
  pid.D = PID_D;
  pid.I = PID_I;

  pid.encoderCurrent = 0;
  pid.encoderPrevious = 0;
  pid.errorCurrent = 0;
  pid.errorPrevious = 0;
  pid.errorI = 0; 
  pid.errorD = 0;

  pid.targetRPM = 0;
  pid.PIDResult = 0;
  pid.status = INACTIVE;

}

/*--------------------------------------------------------------------------
*	Name:			    encoderInit
*	Description:	Initializes Timer 1 and GPIO for DC motor.
*	Parameters:		void
*
*	Returns:		  ret: CmdReturnOk = 0 if Okay.
---------------------------------------------------------------------------*/

ParserReturnVal_t LoopInitialize()
{
  /*Initialize PID structure*/
  InitializePID();

  HAL_StatusTypeDef rc;
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Initialize DC MOTOR direction pins*/
  GPIO_InitTypeDef My_GPIO_InitStructA = {0};
  GPIO_InitTypeDef My_GPIO_InitStructB = {0};
  GPIO_InitTypeDef My_GPIO_InitStructC = {0};  

  for(int i = 0; i< 3; i++){
    if(ports[i] == GPIOA){
      My_GPIO_InitStructA.Pin |= (pins[i]);
    }
    if(ports[i] == GPIOB){
      My_GPIO_InitStructB.Pin |= (pins[i]);
    }
    if(ports[i] == GPIOC){
      My_GPIO_InitStructC.Pin |= (pins[i]);
    }
  }

  if(My_GPIO_InitStructA.Pin != 0){
    My_GPIO_InitStructA.Mode = GPIO_MODE_OUTPUT_PP;
    My_GPIO_InitStructA.Pull = GPIO_NOPULL;
    My_GPIO_InitStructA.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &My_GPIO_InitStructA);
  }
  
  if(My_GPIO_InitStructB.Pin != 0){
    My_GPIO_InitStructB.Mode = GPIO_MODE_OUTPUT_PP;
    My_GPIO_InitStructB.Pull = GPIO_NOPULL;
    My_GPIO_InitStructB.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &My_GPIO_InitStructB);
  }

  if(My_GPIO_InitStructC.Pin != 0){
    My_GPIO_InitStructC.Mode = GPIO_MODE_OUTPUT_PP;
    My_GPIO_InitStructC.Pull = GPIO_NOPULL;
    My_GPIO_InitStructC.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &My_GPIO_InitStructC);
  }
  /* Configure these timer pins for ENCODER */
  // Channel 1
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Channel 2
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
  /*Initialize timer for PWM*/

  __HAL_RCC_TIM1_CLK_ENABLE();
  TIM_OC_InitTypeDef sConfig;
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  tim1.Instance = TIM1;
  tim1.Init.Prescaler = HAL_RCC_GetPCLK2Freq() / 100000 - 1;
  tim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim1.Init.Period = period;
  tim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tim1.Init.RepetitionCounter = 0;
  tim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(&tim1) != HAL_OK)
  {
    printf("Error 1 initializing the timer\n");
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&tim1, &sClockSourceConfig) != HAL_OK)
  {
    printf("Error 2 initializing the timer\n");
  }
  
  HAL_NVIC_SetPriority((IRQn_Type) TIM1_UP_TIM10_IRQn, (uint32_t) 0, (uint32_t) 1);
  HAL_NVIC_EnableIRQ((IRQn_Type) TIM1_UP_TIM10_IRQn);
  // Start timer
  HAL_TIM_Base_Start_IT(&tim1);

  sConfig.OCMode = TIM_OCMODE_PWM1;
  sConfig.Pulse = period/2;
  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfig.OCFastMode = TIM_OCFAST_DISABLE;
  sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfig.OCNIdleState =TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&tim1, &sConfig, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&tim1, &sConfig, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&tim1, &sConfig, TIM_CHANNEL_3);

  GPIO_InitTypeDef GPIO_InitStructPWM = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /* Configure the PWM output pins */
  GPIO_InitStructPWM.Pin = (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);
  GPIO_InitStructPWM.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructPWM.Pull = GPIO_NOPULL;
  GPIO_InitStructPWM.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructPWM.Alternate = 1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructPWM);
  

  /*Initialize timer for ENCODER*/
  __HAL_RCC_TIM3_CLK_ENABLE();
  tim3.Instance = TIM3;
  tim3.Init.Prescaler = 0;
  tim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim3.Init.Period = periodEncoder;
  tim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tim3.Init.RepetitionCounter = 0;
  rc = HAL_TIM_Base_Init(&tim3);
  if (rc != HAL_OK)
  {
      printf("Failed to initialize Timer 3 Base, “ ”rc=%u\n", rc);
      return CmdReturnBadParameter1;
  }

  TIM_Encoder_InitTypeDef encoderConfig;
  encoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  encoderConfig.IC1Polarity = 0;
  encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  encoderConfig.IC1Prescaler = 0;
  encoderConfig.IC1Filter = 3;
  encoderConfig.IC2Polarity = 0;
  encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  encoderConfig.IC2Prescaler = 0;
  encoderConfig.IC2Filter = 3;
  rc = HAL_TIM_Encoder_Init(&tim3, &encoderConfig);
  if (rc != HAL_OK)
  {
      printf("Failed to initialize Timer 3 Encoder, "
              "rc=%u\n",
              rc);
      return CmdReturnBadParameter1;
  }
  rc = HAL_TIM_Encoder_Start(&tim3, TIM_CHANNEL_1);
  if (rc != HAL_OK)
  {
      printf("Failed to start Timer 3 Encoder, "
              "rc=%u\n",
              rc);
      return CmdReturnBadParameter1;
  }
  rc = HAL_TIM_Encoder_Start(&tim3, TIM_CHANNEL_2);
  if (rc != HAL_OK)
  {
      printf("Failed to start Timer 3 Encoder, "
              "rc=%u\n",
              rc);
      return CmdReturnBadParameter1;
  }
  //HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_ENABLE_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_IN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_IN2_PIN, GPIO_PIN_RESET);

  return CmdReturnOk;
}
// MACRO: Add new command to help menu
ADD_CMD("loopinit", LoopInitialize, "\t\tInitializes timer 3 as encoder mode and PID loop")
/*--------------------------------------------------------------------------
*	Name:			    Encoder
*	Description:	Prints the current position of the motor shaft.
*	Parameters:		void
*
*	Returns:		  ret: CmdReturnOk = 0 if Okay.
---------------------------------------------------------------------------*/
ParserReturnVal_t Encoder()
{ 
  uint16_t currentEnc = __HAL_TIM_GET_COUNTER(&tim3); 
  
  printf("%d\n", currentEnc);

  return CmdReturnOk;
}
// MACRO: Add new command to help menu
ADD_CMD("encoder", Encoder, "\t\tGet current encoder value")

/*--------------------------------------------------------------------------
*	Name:			    SetPosition
*	Description:	Takes arguments as a signed raw value for moving motor 
8               to reach desired position  in forward or reverse direction
*	Parameters:		void
*
*	Returns:		  ret: CmdReturnOk = 0 if Okay.
---------------------------------------------------------------------------*/
ParserReturnVal_t PrintSpeed()
{ 
  // Calculate angular distance delta
  int32_t encoderDelta = pid.encoderCurrent - pid.encoderPrevious;
  double turnsDelta = (double)encoderDelta / encoderFullTurn;
  double degreeDelta = turnsDelta*360;

  // Calculate speed
  double speedEncoder = (double)encoderDelta / timebase;
  double speedTurns = turnsDelta / timebase;
  double speedDegree = degreeDelta / timebase;
  double rpms = speedTurns * 60;

  printf("Encoder/s: %.2lf, RPS: %.2lf, RPM: %.2lf, Degrees/s: %.2lf\n", 
    speedEncoder, speedTurns, rpms, speedDegree);

  return CmdReturnOk;
}
// MACRO: Add new command to help menu
ADD_CMD("speed", PrintSpeed, "\t\tPrint current speed of DC Motor")

void CalculateSpeedsAndErrors(){

  // Calculate angular distance delta
  int32_t encoderDelta = pid.encoderCurrent - pid.encoderPrevious;
  double RPSDelta = (double)encoderDelta / encoderFullTurn;

  // Calculate speed
  double RPM = (RPSDelta / timebase)*60;
  
  // Calculate and cycle error samples
  pid.errorPrevious = pid.errorCurrent;
  pid.errorCurrent = pid.targetRPM - RPM;

  // Calculate error derivative
  double errorDelta = (pid.errorCurrent - pid.errorPrevious);
  pid.errorD = errorDelta / timebase;

  // Calculate error integral
  if(fabs(pid.errorCurrent) > fabs(pid.errorPrevious)){
    pid.errorI += (timebase * pid.errorPrevious) + (errorDelta * timebase)/2;
  }
  else {
    pid.errorI += (timebase * pid.errorCurrent) + (errorDelta * timebase)/2;
  }    
}

/*--------------------------------------------------------------------------
*	Name:			    SetSpeed
*	Description:	Takes arguments as a signed raw value for moving motor 
8               to reach desired position  in forward or reverse direction
*	Parameters:		void
*
*	Returns:		  ret: CmdReturnOk = 0 if Okay.
---------------------------------------------------------------------------*/
ParserReturnVal_t SetSpeed()
{ 
  double desiredSpeed;

  if(fetch_double_arg(&desiredSpeed)){
    printf("No argument."
    "Please enter desired speed of the DC motor\n");
  }
  else{
    pid.targetRPM = desiredSpeed;
    pid.status = ACTIVE;
    HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_IN2_PIN, GPIO_PIN_RESET);
    HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_1);
  }

  return CmdReturnOk;
}
// MACRO: Add new command to help menu
ADD_CMD("setspeed", SetSpeed, "\t\tSet desired speed")

/*--------------------------------------------------------------------------
*	Name:			    SetSpeed
*	Description:	Takes arguments as a signed raw value for moving motor 
8               to reach desired position  in forward or reverse direction
*	Parameters:		void
*
*	Returns:		  ret: CmdReturnOk = 0 if Okay.
---------------------------------------------------------------------------*/
ParserReturnVal_t StopMotor()
{ 
  HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_IN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DC_MOTOR_PORT, DC_MOTOR_IN2_PIN, GPIO_PIN_RESET);
  pid.targetRPM = 0;
  pid.status = INACTIVE;
  HAL_TIM_PWM_Stop(&tim1, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&tim1, TIM_CHANNEL_1, 0);

  return CmdReturnOk;
}
// MACRO: Add new command to help menu
ADD_CMD("stop", StopMotor, "\t\tStop control loop and DC motor")

/*--------------------------------------------------------------------------
*	Name:			    SetSpeed
*	Description:	Takes arguments as a signed raw value for moving motor 
8               to reach desired position  in forward or reverse direction
*	Parameters:		void
*
*	Returns:		  ret: CmdReturnOk = 0 if Okay.
---------------------------------------------------------------------------*/
ParserReturnVal_t PrintfPID()
{ 
  // Print PID
  printf("Kp: %.2lf, Ki: %.2lf, Kd: %.2lf, \n P: %.2lf, I: %.2lf, D: %.2lf, \n R: %.2lf \n CR: %d\n",
    pid.P, pid.I, pid.D, pid.errorCurrent, pid.errorI, pid.errorD, pid.PIDResult, currentPWM);

  return CmdReturnOk;
}
// MACRO: Add new command to help menu
ADD_CMD("pid", PrintfPID, "\t\tPrint PID information.")

/*--------------------------------------------------------------------------
*	Name:			    TIM1_UP_TIM10_IRQHandler
*	Description:	Timer 1 interrupt handler
*	Parameters:		void
*
*	Returns:		  void
---------------------------------------------------------------------------*/
void TIM1_UP_TIM10_IRQHandler(void)
{
  // This will call for "HAL_TIM_PeriodElapsedCallback()" on timer update 
  HAL_TIM_IRQHandler(&tim1);
}

/*--------------------------------------------------------------------------
*	Name:			    HAL_TIM_PeriodElapsedCallback
*	Description:	Callbacks for timer overflow/update event
*                - Stops DC motor when desired position is reached
*	Parameters:		*htim - pointer to timer handler
*
*	Returns:		  void
---------------------------------------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

  if (pid.status == ACTIVE){

    // Cycle samples of encoder
    pid.encoderPrevious = pid.encoderCurrent;
    pid.encoderCurrent = (int32_t)__HAL_TIM_GET_COUNTER(&tim3); 

    // Calculate speeds and errors
    CalculateSpeedsAndErrors();

    // Calculate PID
    pid.PIDResult = (pid.P * pid.errorCurrent) + (pid.I * pid.errorI) + (pid.D * pid.errorD);

    // Affect our plant
    currentPWM = __HAL_TIM_GET_COUNTER(&tim1); 
    currentPWM += ((uint16_t)(pid.PIDResult * pid2pwm));
 
    // Update PWM value
    __HAL_TIM_SET_COMPARE(&tim1, TIM_CHANNEL_1, currentPWM);
    //__HAL_TIM_SET_COMPARE(&tim1, TIM_CHANNEL_1, period);
    //printf("%.2lf\n", pid.PIDResult);
    }
  
}

