/**
  ******************************************************************************
  * @file    speed_regulator.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the Temperature Sensor component of the Motor Control SDK.
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
  */

/* Includes ------------------------------------------------------------------*/
#include "speed_regulator_potentiometer.h"

/** @addtogroup MCSDK
  * @{
  */


/* Functions ---------------------------------------------------- */

/**
 * @brief Initializes speed regulator conversions
 *
 *  @p pHandle : Pointer on Handle structure of SpeedRegulator component
 *
 */
__weak void SRP_Init(SRP_Handle_t *pHandle, SpeednTorqCtrl_Handle_t *pSTC)
{

  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    /* Need to be register with RegularConvManager */
    pHandle->convHandle = RCM_RegisterRegConv(&pHandle->SpeedRegConv);
    pHandle->pSTC = pSTC;
    SRP_Clear(pHandle);
  }
}

/**
 * @brief Initializes internal speed regulator computed value
 *
 *  @p pHandle : Pointer on Handle structure of speed regulator
 */
__weak void SRP_Clear(SRP_Handle_t *pHandle )
{
  if (MC_NULL == pHandle)
  {
    /* nothing to do */
  }
  else
  {
    uint16_t aux;
    uint16_t index;
    aux = ( pHandle->MaximumSpeedRange + pHandle->MinimumSpeedRange ) / 2u;
    for ( index = 0u; index < pHandle->LowPassFilterBW; index++ )
    {
      pHandle->aBuffer[index] = aux;
    }
    pHandle->LatestConv = aux;
    pHandle->AvSpeedReg_d = aux;
    pHandle->index = 0;
  }
}

static uint16_t SRP_ConvertSpeedRegFiltered( SRP_Handle_t *pHandle )
{
  uint16_t hAux;
  uint8_t vindex;
  uint16_t max = 0, min = 0;
  uint32_t tot = 0u;

  for ( vindex = 0; vindex < pHandle->LowPassFilterBW; )
  {
    hAux = RCM_ExecRegularConv(pHandle->convHandle);

    if ( hAux != 0xFFFFu )
    {
      if ( vindex == 0 )
      {
        min = hAux;
        max = hAux;
      }
      else
      {
        if ( hAux < min )
        {
          min = hAux;
        }
        if ( hAux > max )
        {
          max = hAux;
        }
      }
      vindex++;

      tot += hAux;
    }
  }

  tot -= max;
  tot -= min;
  return ( uint16_t )( tot / ( pHandle->LowPassFilterBW - 2u ) );
}

static uint16_t SRP_SpeedDigitToSpeedUnit( SRP_Handle_t *pHandle, uint16_t DigitValue )
{
  uint16_t hAux;
  hAux = DigitValue / pHandle->ConversionFactor + pHandle->MinimumSpeedRange;
  return hAux;
}

/**
  * @brief  It actually performes the Speed regulator ADC conversion and updates average
  *         value
  * @param  pHandle related SRP_Handle_t
  * @retval bool OutOfSynch signal
  */
__weak bool SRP_CalcAvSpeedRegFilt( SRP_Handle_t *pHandle )
{
  uint32_t wtemp;
  uint16_t hAux;
  uint8_t i;

  hAux = SRP_ConvertSpeedRegFiltered( pHandle );

  if ( hAux != 0xFFFF )
  {
    pHandle->aBuffer[pHandle->index] = hAux;
    wtemp = 0;
    for ( i = 0; i < pHandle->LowPassFilterBW; i++ )
    {
      wtemp += pHandle->aBuffer[i];
    }
    wtemp /= pHandle->LowPassFilterBW;
    pHandle->AvSpeedReg_d = ( uint16_t )wtemp;
    pHandle->LatestConv = hAux;

    if ( pHandle->index < pHandle->LowPassFilterBW - 1 )
    {
      pHandle->index++;
    }
    else
    {
      pHandle->index = 0;
    }
  }
  pHandle->OutOfSynchro = SRP_CheckSpeedRegSync( pHandle );

  return ( pHandle->OutOfSynchro );
}

/**
  * @brief  It actually performes the speed regulator ADC conversion and updates average
  *         value
  * @param  pHandle related SRP_Handle_t
  * @retval bool OutOfSynch signal
  */
__weak bool SRP_CalcAvSpeedReg( SRP_Handle_t * pHandle )
{
  uint32_t wtemp;
  uint16_t hAux;
  uint8_t i;

  hAux = RCM_ExecRegularConv(pHandle->convHandle);

  if ( hAux != 0xFFFF )
  {
    pHandle->aBuffer[pHandle->index] = hAux;
    wtemp = 0;
    for ( i = 0; i < pHandle->LowPassFilterBW; i++ )
    {
      wtemp += pHandle->aBuffer[i];
    }
    wtemp /= pHandle->LowPassFilterBW;
    pHandle->AvSpeedReg_d = ( uint16_t )wtemp;
    pHandle->LatestConv = hAux;

    if ( pHandle->index < pHandle->LowPassFilterBW - 1 )
    {
      pHandle->index++;
    }
    else
    {
      pHandle->index = 0;
    }
  }
  pHandle->OutOfSynchro = SRP_CheckSpeedRegSync( pHandle );

  return ( pHandle->OutOfSynchro );
}

/**
  * @brief  It returns OutOfSync check between current potentiometer setting and measured speed
  * @param  pHandle related SRP_Handle_t
  * @retval bool OutOfSynch signal
  */
__weak bool SRP_CheckSpeedRegSync( SRP_Handle_t * pHandle )
{
  bool hAux = false;
  uint16_t speedRegValue = SRP_SpeedDigitToSpeedUnit(pHandle, pHandle->AvSpeedReg_d );
  uint16_t speedRegTol = SRP_SpeedDigitToSpeedUnit(pHandle, pHandle->SpeedAdjustmentRange);
  SpeednPosFdbk_Handle_t *speedHandle;
  speedHandle = STC_GetSpeedSensor(pHandle->pSTC);
  if ((speedHandle->hAvrMecSpeedUnit > (speedRegValue +  speedRegTol)) || (speedHandle->hAvrMecSpeedUnit < (speedRegValue -  speedRegTol)))
  {
    hAux = true;
  }
  return hAux;
}

/**
  * @brief  Executes speed ramp and applies external speed regulator new setpoint if ajustment range is violated
  * @param  pHandle related SRP_Handle_t
  * @retval bool final speed is equal to measured speed
  */
__weak bool SRP_ExecPotRamp( SRP_Handle_t * pHandle )
{
  bool hAux = SRP_CheckSpeedRegSync(pHandle);
  uint16_t PotentiometerReqSpeed = pHandle->LatestConv;
  uint16_t PotentiometerCurrentSpeed = pHandle->AvSpeedReg_d;

  SpeednPosFdbk_Handle_t *speedHandle;
  speedHandle = STC_GetSpeedSensor(pHandle->pSTC);
  int16_t CurrentSpeed = speedHandle->hAvrMecSpeedUnit;
      
  if ((PotentiometerReqSpeed <= PotentiometerCurrentSpeed - pHandle->SpeedAdjustmentRange) ||   // Requested speed must be different from previous one
      (PotentiometerReqSpeed >= PotentiometerCurrentSpeed + pHandle->SpeedAdjustmentRange))
  {
    int16_t deltaSpeed;
    uint16_t RequestedSpeed = SRP_SpeedDigitToSpeedUnit(pHandle, PotentiometerReqSpeed );
    uint32_t hDuration;
    deltaSpeed = (int16_t) RequestedSpeed - CurrentSpeed;
    if (deltaSpeed > 0) hDuration = ((uint32_t) deltaSpeed) * 1000 / pHandle->RampSlope;
    else hDuration = ((uint32_t) (- deltaSpeed)) * 1000 / pHandle->RampSlope;
    if (CurrentSpeed < 0)
    {
      STC_ExecRamp(pHandle->pSTC, (int16_t) (- RequestedSpeed), hDuration);
    }
    else
    {
      STC_ExecRamp(pHandle->pSTC, (int16_t) RequestedSpeed, hDuration);
    }
  }
  return hAux;
}

/**
  * @brief  Executes speed ramp and applies external speed regulator new setpoint if ajustment range is violated
  * @param  pHandle related SRP_Handle_t
  * @retval bool final speed is equal to measured speed
  */
__weak uint16_t SRP_GetSpeedReg_d( SRP_Handle_t * pHandle )
{
  return ( pHandle->LatestConv );
}
__weak uint16_t SRP_GetAvSpeedReg_d( SRP_Handle_t * pHandle )
{
  return ( pHandle->AvSpeedReg_d );
}
__weak uint16_t SRP_GetAvSpeedReg_SU( SRP_Handle_t * pHandle )
{
  uint16_t temp = SRP_SpeedDigitToSpeedUnit(pHandle, pHandle->AvSpeedReg_d );
  return ( temp );
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
