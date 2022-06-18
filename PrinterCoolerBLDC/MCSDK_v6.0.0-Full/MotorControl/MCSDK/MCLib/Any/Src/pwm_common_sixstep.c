/**
  ******************************************************************************
  * @file    pwm_common_sixstep.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the six-step PWM component of the Motor Control SDK:
  *
  *           * ADC triggering for sensorless bemf acquisition
  *           * translation of the electrical angle in step sequence
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup pwm_curr_fdbk_6s
  */

/* Includes ------------------------------------------------------------------*/
#include "pwm_common_sixstep.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @defgroup pwm_curr_fdbk_6s Six-Step PWM generation
  *
  * @brief PWM components for Six Step drive
  *
  * @todo Complete documentation for the pwm_curr_fdbk_6s module 
  * 
  * @{
  */

/**
  * @brief  It is used to clear the variable in CPWMC.
  * @param  this related object of class CPWMC
  * @retval none
  */
void PWMC_Clear(PWMC_Handle_t *pHandle)
{
}

/**
  * @brief  Switches PWM generation off, inactivating the outputs.
  * @param  pHandle Handle on the target instance of the PWMC component
  */
__weak void PWMC_SwitchOffPWM( PWMC_Handle_t * pHandle )
{
  pHandle->pFctSwitchOffPwm( pHandle );
}

/**
  * @brief  Switches PWM generation on
  * @param  pHandle Handle on the target instance of the PWMC component
  */
__weak void PWMC_SwitchOnPWM( PWMC_Handle_t * pHandle )
{
  pHandle->pFctSwitchOnPwm( pHandle );
}

/**
  * @brief  Set the ADC trigger point for bemf acquisition.
  * @param  pHandle Handle on the target instance of the PWMC component
  * @param  SamplingPoint pulse value of the timer channel used for ADC triggering
  */
__weak void PWMC_SetADCTriggerChannel( PWMC_Handle_t * pHandle, uint16_t SamplingPoint )
{
  pHandle->pFctSetADCTriggerChannel( pHandle, SamplingPoint );
}

/**
  * @brief  Switches power stage Low Sides transistors on.
  *
  * This function is meant for charging boot capacitors of the driving
  * section. It has to be called on each motor start-up when using high
  * voltage drivers.
  *
  * @param  pHandle: handle on the target instance of the PWMC component
  */
__weak void PWMC_TurnOnLowSides( PWMC_Handle_t * pHandle )
{
  pHandle->pFctTurnOnLowSides( pHandle );
}

/** @brief Returns #MC_BREAK_IN if an over current condition was detected on the power stage
 *         controlled by the PWMC component pointed by  @p pHandle, since the last call to this function;
 *         returns #MC_NO_FAULTS otherwise. */
__weak uint16_t PWMC_CheckOverCurrent( PWMC_Handle_t * pHandle )
{
  return pHandle->pFctIsOverCurrentOccurred( pHandle );
}

/**
  * @brief  It is used to retrieve the satus of TurnOnLowSides action.
  * @param  pHandle: handler of the current instance of the PWMC component
  * @retval bool It returns the state of TurnOnLowSides action:
  *         true if TurnOnLowSides action is active, false otherwise.
  */
/** @brief Returns the status of the "TurnOnLowSide" action on the power stage
 *         controlled by the @p pHandle PWMC component: true if it
 *         is active, false otherwise*/
__weak bool PWMC_GetTurnOnLowSidesAction( PWMC_Handle_t * pHandle )
{
  return pHandle->TurnOnLowSidesAction;
}

/**
* @brief  It is used to set the align motor flag.
* @param  this related object of class CPWMC
* @param  flag to be applied in uint8_t, 1: motor is in align stage, 2: motor is not in align stage
* @retval none
*/
void PWMC_SetAlignFlag(PWMC_Handle_t *pHandle, int16_t flag)
{
  pHandle->AlignFlag = flag;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to switch PWM
 *        generation off.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void PWMC_RegisterSwitchOffPwmCallBack( PWMC_Generic_Cb_t pCallBack,
                                        PWMC_Handle_t * pHandle )
{
  pHandle->pFctSwitchOffPwm = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to switch PWM
 *        generation on.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void PWMC_RegisterSwitchonPwmCallBack( PWMC_Generic_Cb_t pCallBack,
                                       PWMC_Handle_t * pHandle )
{
  pHandle->pFctSwitchOnPwm = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to turn low sides on.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void PWMC_RegisterTurnOnLowSidesCallBack( PWMC_Generic_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctTurnOnLowSides = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to the overcurrent status
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void PWMC_RegisterIsOverCurrentOccurredCallBack( PWMC_OverCurr_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctIsOverCurrentOccurred = pCallBack;
}


/**
 * @brief Converts the motor electrical angle to the corresponding step in the six-step sequence
 * @param pHandle pointer on the handle structure of the PWMC instance
 * @retval calculated step
 */
__weak uint8_t  PWMC_ElAngleToStep( PWMC_Handle_t * pHandle )
{
  uint8_t Step;
  if ((pHandle->hElAngle >= (int16_t)( S16_60_PHASE_SHIFT / 2)) && (pHandle->hElAngle < (int16_t)( S16_60_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2))) Step = STEP_1;
  else if ((pHandle->hElAngle >= (int16_t)( S16_60_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2)) && (pHandle->hElAngle < (int16_t)( S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2))) Step = STEP_2;
  else if ((pHandle->hElAngle >= (int16_t)( S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2)) || (pHandle->hElAngle < (int16_t)( - S16_120_PHASE_SHIFT - S16_60_PHASE_SHIFT / 2))) Step = STEP_3;
  else if ((pHandle->hElAngle >= (int16_t)( - S16_120_PHASE_SHIFT - S16_60_PHASE_SHIFT / 2)) && (pHandle->hElAngle < (int16_t)( - S16_60_PHASE_SHIFT - S16_60_PHASE_SHIFT / 2))) Step = STEP_4;
  else if ((pHandle->hElAngle >= (int16_t)( - S16_60_PHASE_SHIFT - S16_60_PHASE_SHIFT / 2)) && (pHandle->hElAngle < (int16_t)( - S16_60_PHASE_SHIFT / 2))) Step = STEP_5;
  else if ((pHandle->hElAngle >= (int16_t)( - S16_60_PHASE_SHIFT / 2)) && (pHandle->hElAngle < (int16_t)( S16_60_PHASE_SHIFT / 2))) Step = STEP_6;
  else {}
  return Step;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
