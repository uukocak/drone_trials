/**
 *	PID Library for multiple instance in multiple formats
 *
 *	@author 	Umut Utku Kocak
 *	@email		umutkocak@hacettepe.edu.tr
 *      @company    
 *	@version 	v1.0
 *	@ide		Keil uVision
 *
 **/

#ifndef _MD_PID_H
#define _MD_PID_H

#include "stdint.h" 
/*included for uint32_t and int32_t types */

typedef float float32_t;

/**
 * Buffer type selected for integration buffer
 */
typedef enum{
	PID_iBuf_Ring,
	PID_iBuf_Single
}PID_iBuf_type;

/**
  * @brief Instance structure for the float32_t PID Control.
  */
typedef struct{

    float32_t pGain;                /**< The proportional gain. */
    float32_t iGain;                /**< The integrational gain. */
    float32_t dGain;                /**< The derivational gain. */

    float32_t pError;               /**< Previous error. */
	float32_t *iBuf;                /**< Integration buffer pointer. */
	uint32_t iBuf_len_f;            /**< Integration buffer length */
	PID_iBuf_type iBuf_type_f;      /**< Integration buffer type*/

	float32_t RangeMin;             /**< Saturation minimum range. */
	float32_t RangeMax;	            /**< Saturation maximum range. */

    float32_t iMin;                 /**< Intagretion saturation minimum range. */
	float32_t iMax;	                /**< Intagretion saturation maximum range. */

	float32_t action;				/**< Last calculated PID result */
} PID_f32_t;

 /**
   * @brief Instance structure for the int32_t PID Control.
   */
typedef struct{

    float32_t pGain;                /**< The proportional gain. */
    float32_t iGain;                /**< The integrational gain. */
    float32_t dGain;                /**< The derivational gain. */

    int32_t pError;                 /**< Previous error. */
	int32_t *iBuf;                  /**< Integration buffer pointer. */
	uint32_t iBuf_len_f;            /**< Integration buffer length */
	PID_iBuf_type iBuf_type_f;      /**< Integration buffer type*/

	int32_t RangeMin;               /**< Saturation minimum range. */
	int32_t RangeMax;	            /**< Saturation maximum range. */

    int32_t iMin;                   /**< Intagretion saturation minimum range. */
	int32_t iMax;	                /**< Intagretion saturation maximum range. */

	int32_t action;				    /**< Last calculated PID result */
} PID_i32_t;


/**
 * Initialize specific PID for 32bit float variable calculation
 *
 * Parameters:
 * 	- PID_f32_t* pid_s
 * 		Pointer to selected pid,  you want to init
 * 	- PID_iBuf_type iBuf_type
 * 		Select integration buffer type .
 * 		    Single variable : Sums errors in one variable (  PID_iBuf_Single  )
 *       (x)Ring buffer     : Records error values in a ring buffer (  PID_iBuf_Ring  )
 * 	- uint32_t iBuf_len
 * 		Buffer length for ring buffer , bigger values result longer calculations
 *  - float32_t Kp
 * 		Proportional gain for pid
 *  - float32_t Ki
 * 		Integrational gain for pid
 *  - float32_t Kd
 * 		Derivative gain for pid
 */
void PID_f32_Init(PID_f32_t * pid_s , PID_iBuf_type iBuf_type , uint32_t iBuf_len , float32_t Kp , float32_t Ki , float32_t Kd );

/**
 * Calculate PID result for given position and setpoint values (32bit float )
 *
 * Parameters:
 * 	- PID_f32_t* pid_s
 * 		Pointer to selected pid,  you want to use for calculation
 *  -float32_t positon
 *      Current position
 *  -float32_t setpoint
 *      Desired point or command
 *
 *  Returns:
 *  -Value of action with respect to error
 *  -Last return value also stored at PID_s.action variable
 *
 *  Must be called in iteration of constant frequancy
 *
 */
float32_t PID_f32_Update(PID_f32_t * pid_s , float32_t positon , float32_t setpoint);

/**
 * Set saturation range for specific PID (32bit float )
 * Parameters:
 *	- PID_f32_t* pid_s
 * 		Pointer to selected pid you want to set
 *  - float32_t rMin
 *      Minimum value for PID result
 *  - float32_t rMax
 *      Maximum value for PID result
 *
 *   Default : rMin = rMax = -1 ( Deactivated )
 */
void PID_f32_SetRange(PID_f32_t * pid_s , float32_t rMin , float32_t rMax);

/**
 * Set or update PID gains for specific PID (32bit float  )
 * Parameters:
 *	- PID_f32_t* pid_s
 * 		Pointer to selected pid you want to tune
 *  - float32_t Kp
 * 		Proportional gain for pid
 *  - float32_t Ki
 * 		Integrational gain for pid
 *  - float32_t Kd
 * 		Derivative gain for pid
 */
void PID_f32_Tune(PID_f32_t * pid_s , float32_t Kp , float32_t Ki , float32_t Kd );




/**
 * Initialize specific PID for 32bit integer variable calculation
 *
 * Parameters:
 * 	- PID_i32_t* pid_s
 * 		Pointer to selected pid,  you want to init
 * 	- PID_iBuf_type iBuf_type
 * 		Select integration buffer type .
 * 		    Single variable : Sums errors in one variable (  PID_iBuf_Single  )
 *       (x)Ring buffer     : Records error values in a ring buffer (  PID_iBuf_Ring  )
 * 	- uint32_t iBuf_len
 * 		Buffer length for ring buffer , bigger values result longer calculations
 *  - float32_t Kp
 * 		Proportional gain for pid
 *  - float32_t Ki
 * 		Integrational gain for pid
 *  - float32_t Kd
 * 		Derivative gain for pid
 */
void PID_i32_Init(PID_i32_t * pid_s , PID_iBuf_type iBuf_type , uint32_t iBuf_len , float32_t Kp , float32_t Ki , float32_t Kd );


/**
 * Calculate PID result for given position and setpoint values (32bit integer )
 *
 * Parameters:
 * 	- PID_i32_t* pid_s
 * 		Pointer to selected pid,  you want to use for calculation
 *  -int32_t positon
 *      Current position
 *  -int32_t setpoint
 *      Desired point or command
 *
 *  Returns:
 *  -Value of action with respect to error
 *  -Last return value also stored at PID_s.action variable
 *
 *  Must be called in iteration of constant frequancy
 *
 */
int32_t PID_i32_Update(PID_i32_t * pid_s , int32_t positon , int32_t setpoint);

/**
 * Set saturation range for specific PID (32bit integer )
 * Parameters:
 *	- PID_i32_t* pid_s
 * 		Pointer to selected pid you want to set
 *  - int32_t rMin
 *      Minimum value for PID result
 *  - int32_t rMax
 *      Maximum value for PID result
 *
 *   Default : rMin = rMax = -1 ( Deactivated )
 */
void PID_i32_SetRange(PID_i32_t * pid_s , int32_t rMin , int32_t rMax);

/**
 * Set or update PID gains for specific PID (32bit integer  )
 * Parameters:
 *	- PID_i32_t* pid_s
 * 		Pointer to selected pid you want to tune
 *  - float32_t Kp
 * 		Proportional gain for pid
 *  - float32_t Ki
 * 		Integrational gain for pid
 *  - float32_t Kd
 * 		Derivative gain for pid
 */
void PID_i32_Tune(PID_i32_t * pid_s , float32_t Kp , float32_t Ki , float32_t Kd );

#endif
