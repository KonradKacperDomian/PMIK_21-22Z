/**
  ****************************************************************************
  * @file           : AR-3603HB_api.h
  * @brief          : Servo AR-3603HB library header file
  * @author			: Konrad Kacper Domian
  ****************************************************************************
  * @attention
  *
  *	Library was made for AR-3603HB servo, if u want use it firstly
  *	change define
  *	TIM_NO
  *	TIM_CH_X
  *	TIME_IN_MS_TO_ANGLE_60 140
  *
  *
  ****************************************************************************
  */



#ifndef INC_AR_3603HB_API_H_
	#define INC_AR_3603HB_API_H_
	#include "tim.h"

	#define TIM_NO htim3 //Set witch timer u use
	/**
	 * Set timers Channels
	 */
	#define TIM_CH_1 TIM_CHANNEL_1
	#define TIM_CH_2 TIM_CHANNEL_2
	//#define TIM_CH_3 TIM_CHANNEL_3
	//#define TIM_CH_4 TIM_CHANNEL_4

	#define TIME_IN_MS_TO_ANGLE_60 140 //Set time of move servo to 60 degres



	/**
	 * @brief Initialize AR3603HB servo
	 * @retval None
	 */
	void InitAR3603HB(void);
	/**
	  * @brief Blocking funtion for move servo AR3603HB, Flags: 0-Blocking ~0-NonBlocking
	  * @retval None
	  */
	void MoveAR3603HB(int angle, int ServoNumber, int FLAG);
	/**
		  * @brief Timer For not blocking MoveAR3603HB fun.
		  * @retval None
	*/
	void SysTick_Handler();
	/**
		* @brief Timer For not blocking MoveAR3603HB fun.
		* @retval None
	*/
	void delay_ms(int time);

#endif /* INC_AR_3603HB_API_H_ */
