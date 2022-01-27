/**
  ****************************************************************************
  * @file           : AR-3603HB_api.c
  * @brief          : Servo AR-3603HB library c file
  * @author			: Konrad Kacper Domian
  ****************************************************************************
  * @attention
  *
  *	Library was made for AR-3603HB servo, if u want use it firstly
  *	change parametters
  *
  *
  ****************************************************************************
  */

#include "AR-3603HB_api.h"
#include "tim.h"
#include "string.h"
#include <stdio.h>
#include <string.h>

struct
{
#ifdef TIM_CH_1
unsigned int tim_ch_1:1;
#endif /*TIM_CH_1 */

#ifdef TIM_CH_2
	unsigned int tim_ch_2:1;
#endif /*TIM_CH_2 */

#ifdef TIM_CH_3
	unsigned int tim_ch_3:1;
#endif /*TIM_CH_3 */

#ifdef TIM_CH_4
	unsigned int tim_ch_4:1;
#endif /*TIM_CH_4 */
}Timers;




void InitAR3603HB(void)
{
#ifdef TIM_CH_1
	HAL_TIM_PWM_Start(&TIM_NO, TIM_CH_1);

#endif /*TIM_CH_1 */

#ifdef TIM_CH_2
	HAL_TIM_PWM_Start(&TIM_NO, TIM_CH_2);
#endif /*TIM_CH_2 */

#ifdef TIM_CH_3
	HAL_TIM_PWM_Start(&TIM_NO, TIM_CH_3);
#endif /*TIM_CH_3 */

#ifdef TIM_CH_4
	HAL_TIM_PWM_Start(&TIM_NO, TIM_CH_4);
#endif /*TIM_CH_4 */


}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim10 )
  {
#ifdef TIM_CH_1
	if(Timers.tim_ch_1)
		__HAL_TIM_SET_COMPARE(&TIM_NO, TIM_CH_1, 0);
#endif /*TIM_CH_1 */

#ifdef TIM_CH_2
	if(Timers.tim_ch_2)
			__HAL_TIM_SET_COMPARE(&TIM_NO, TIM_CH_2, 0);
#endif /*TIM_CH_2 */

#ifdef TIM_CH_3
	if(Timers.tim_ch_3)
				__HAL_TIM_SET_COMPARE(&TIM_NO, TIM_CH_3, 0);
#endif /*TIM_CH_3 */

#ifdef TIM_CH_4
	if(Timers.tim_ch_4)
				__HAL_TIM_SET_COMPARE(&TIM_NO, TIM_CH_4, 0);
#endif /*TIM_CH_4 */
  }
}

void MoveAR3603HB(int angle, int ServoNumber, int FLAG)
{
#ifdef TIM_CH_1
	if(ServoNumber == 1)
	{
		if(FLAG == 0)
		{
			__HAL_TIM_SET_COMPARE(&TIM_NO, TIM_CH_1, 1400);
			TIMER((TIME_IN_MS_TO_ANGLE_60/60)*angle);
			__HAL_TIM_SET_COMPARE(&TIM_NO, TIM_CH_1, 0);
			TIMER(10);
		}
		else
		{
			HAL_TIM_Base_Start_IT(&htim10);
			Timers.tim_ch_1=1;
			__HAL_TIM_SET_COMPARE(&TIM_NO, TIM_CH_1, 1400);
		}
	}
#endif /*TIM_CH_1 */


#ifdef TIM_CH_2
	if(ServoNumber == 2)
		{
			if(FLAG == 0)
			{
				__HAL_TIM_SET_COMPARE(&TIM_NO, TIM_CH_2, 1400);
				TIMER((TIME_IN_MS_TO_ANGLE_60/60)*angle);
				__HAL_TIM_SET_COMPARE(&TIM_NO, TIM_CH_2, 0);
				TIMER(10);
			}
			else
			{

				HAL_TIM_Base_Start_IT(&htim10);
				Timers.tim_ch_2=1;
				__HAL_TIM_SET_COMPARE(&TIM_NO, TIM_CH_2, 1400);
			}
		}
#endif /*TIM_CH_2 */

#ifdef TIM_CH_3
	if(ServoNumber == 3)
			{
				if(FLAG == 0)
				{
					__HAL_TIM_SET_COMPARE(&TIM_NO, TIM_CH_3, 1400);
					HAL_Delay((TIME_IN_MS_TO_ANGLE_60/60)*angle);
					__HAL_TIM_SET_COMPARE(&TIM_NO, TIM_CH_3, 0);
					HAL_Delay(100);
				}


					HAL_TIM_Base_Start_IT(&htim10);
					Timers.tim_ch_3=1;
					__HAL_TIM_SET_COMPARE(&TIM_NO, TIM_CH_3, 1400);
			}
#endif /*TIM_CH_3 */

#ifdef TIM_CH_4
	if(ServoNumber == 4)
			{
				if(FLAG == 0)
				{
					__HAL_TIM_SET_COMPARE(&TIM_NO, TIM_CH_4, 1400);
					HAL_Delay((TIME_IN_MS_TO_ANGLE_60/60)*angle);
					__HAL_TIM_SET_COMPARE(&TIM_NO, TIM_CH_4, 0);
					HAL_Delay(100);
				}


					HAL_TIM_Base_Start_IT(&htim10);
					Timers.tim_ch_1=1;
					__HAL_TIM_SET_COMPARE(&TIM_NO, TIM_CH_4, 1400);
			}
#endif /*TIM_CH_4 */
}

