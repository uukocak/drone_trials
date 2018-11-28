#include <md_pid.h>

void PID_f32_Init(PID_f32_t * pid_s , PID_iBuf_type iBuf_type , uint32_t iBuf_len , float32_t Kp , float32_t Ki , float32_t Kd )
{
	if(iBuf_type == PID_iBuf_Ring )
	{
        pid_s -> iBuf = malloc(iBuf_len * sizeof(float32_t));
	    /* Allocate memory for integration buffer (ring type) */
	}
	else if(iBuf_type == PID_iBuf_Single )
	{
        pid_s -> iBuf = malloc(1 * sizeof(float32_t));
	    /* Allocate memory for integration buffer (single variable type) */
	}
	
	/* Set and store initial PID Gains */
	pid_s -> pGain = Kp; 
	pid_s -> iGain = Ki;
	pid_s -> dGain = Kd;
	
    /* Set and store buffer type and length */
	pid_s -> iBuf_type_f =  iBuf_type;
	pid_s -> iBuf_len_f =  iBuf_len;
    
	/* Set initial saturation ranges ( Saturation deactivated)  */
	pid_s -> RangeMin = -1;
	pid_s -> RangeMax = -1;	
    
    /* Set initial integration saturation ranges  */
	pid_s -> iMin = 0;
	pid_s -> iMax = 0;	
}

float32_t PID_f32_Update(PID_f32_t * pid_s , float32_t positon , float32_t setpoint)
{
	float32_t  pTerm , iTerm , dTerm, action ;
	float32_t  error = position - setpoint;
	
    /* P term calculation */
 	pTerm = pid_s -> pGain * error;
 	
    /* D term calculation */
    dTerm = error - pid_s -> pError;
    dTerm *= pid_s -> dGain ;
	
    /* Filling integration buffer */
	pid_s -> *iBuf += error;
	
    /* Saturation integration buffer */
	if(*iBuf > pid_s->iMax ) 
        *iBuf = pid_s->iMax ;
	else if (*iBuf < pid_s->iMin ) 
        *iBuf = pid_s->iMin;
    
    /* I term calculation */
	iTerm = *iBuf * pid_s -> iGain;
	
    /* Store last error */
	pid_s -> pError = error;
	
    /* Final calculation */
	action = pid_s -> pTerm + pid_s -> iTerm + pid_s -> dTerm;
	
     /* Final saturation */
	if( pid_s-> RangeMin != pid_s -> RangeMax )
	{
		if( action > pid_s -> RangeMax  ) 
            action = pid_s -> RangeMax;
		else if ( action < pid_s-> RangeMin ) 
            action = pid_s-> RangeMin;
	}
	
    /* Store last calculated value */
	pid_s -> action = action;
	
	return action;
}

void PID_i32_Init(PID_i32_t * pid_s , PID_iBuf_type iBuf_type , uint32_t iBuf_len , float32_t Kp , float32_t Ki , float32_t Kd )
{
	if(iBuf_type == PID_iBuf_Ring )
        pid_s -> iBuf = malloc(iBuf_len * sizeof(uint32_t));
	    /* Allocate memory for integration buffer (ring type) */
	
	else if(iBuf_type == PID_iBuf_Single )
        pid_s -> iBuf = malloc(1 * sizeof(uint32_t));
	    /* Allocate memory for integration buffer (single variable type) */
	
	
	
	/* Set and store initial PID Gains */
	pid_s -> pGain = Kp; 
	pid_s -> iGain = Ki;
	pid_s -> dGain = Kd;
	
    /* Set and store buffer type and length */
	pid_s -> iBuf_type_f =  iBuf_type;
	pid_s -> iBuf_len_f =  iBuf_len;
    
	/* Set initial saturation ranges ( Saturation deactivated)  */
	pid_s -> RangeMin = -1;
	pid_s -> RangeMax = -1;	
	
     /* Set initial integration saturation ranges  */
	pid_s -> iMin = 0;
	pid_s -> iMax = 0;	
}

int32_t PID_i32_Update(PID_i32_t * pid_s , int32_t positon , int32_t setpoint)
{
	int32_t  pTerm , iTerm , dTerm , action ;
	int32_t  error = position - setpoint;
	
     /* P term calculation */
 	pTerm = (int32_t)pid_s -> pGain * error;      
 	
     /* D term calculation */
    dTerm = error - pid_s -> pError;
    dTerm = (int32_t) pid_s -> dGain * dTerm ;
	
    /* Filling integration buffer */
	pid_s -> *iBuf += error;
	
     /* Saturation integration buffer */
	if(*iBuf > pid_s->iMax ) 
        *iBuf = pid_s->iMax ;
	else if (*iBuf < pid_s->iMin ) 
        *iBuf = pid_s->iMin;
    
      /* I term calculation */
	iTerm = (int32_t) *iBuf * pid_s -> iGain;
	
      /* Store last error */
	pid_s -> pError = error;
	
     /* Final calculation */
	action = pid_s -> pTerm + pid_s -> iTerm + pid_s -> dTerm;
	
     /* Final saturation */
	if( pid_s-> RangeMin != pid_s -> RangeMax )
	{
		if( action > pid_s -> RangeMax  ) 
            action = pid_s -> RangeMax;
		else if ( action < pid_s-> RangeMin ) 
            action = pid_s-> RangeMin;
	}
	
     /* Store last calculated value */
	pid_s -> action = action;
	
	return action;
}

void PID_i32_SetRange(PID_i32_t * pid_s , int32_t rMin , int32_t rMax)
{
	pid_s -> RangeMin = rMin;
	pid_s -> RangeMax = rMax;	
}

void PID_f32_SetRange(PID_f32_t * pid_s , float32_t rMin , float32_t rMax)
{
	pid_s -> RangeMin = rMin;
	pid_s -> RangeMax = rMax;	
}	

void PID_i32_Tune(PID_i32_t * pid_s , float32_t Kp , float32_t Ki , float32_t Kd )
{
	if ( Kp < 0 || Ki < 0 || Kd < 0 ) return; 
	pid_s -> pGain = Kp;
	pid_s -> iGain = Ki;
	pid_s -> dGain = Kd;	
}

void PID_f32_Tune(PID_f32_t * pid_s , float32_t Kp , float32_t Ki , float32_t Kd )
{
    if ( Kp < 0 || Ki < 0 || Kd < 0 ) return;
	pid_s -> pGain = Kp;
	pid_s -> iGain = Ki;
	pid_s -> dGain = Kd;	
}
