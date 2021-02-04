//this is for debugging purposes 

#ifndef ENCODER_H
#define ENCODER_H

#ifndef INLINE
#define INLINE extern inline
#endif

typedef struct encoder{
	int32_t encoder_position; //position of virtual encoder
	int16_t encoder_run;		//steps run per segment
	int8_t encoder_sign;		//direction of encoder
	//has 1 char that won't affect size
}Encoder_t;


typedef struct encoders{
	Encoder_t en[MOTORS];
}Encoders_t;

extern Encoders_t en;

#define EN_SET_STEP_SIGN(motor,sign) en.en[motor].encoder_sign = sign
#define EN_INCREMENT(motor) en.en[motor].encoder_run += en.en[motor].encoder_sign
#define EN_ACCUMULATE(motor) en.en[motor].encoder_position += en.en[motor].encoder_run; en.en[motor].encoder_run = 0


INLINE void en_set_step_sign(uint8_t motor, int8_t sign) __attribute__((always_inline));
INLINE void en_increment(uint8_t motor) __attribute__((always_inline));
INLINE void en_accumulate(uint8_t motor) __attribute__((always_inline));
INLINE int32_t en_read_encoder(uint8_t motor) __attribute__((always_inline));
void encoder_init(void);
void en_set_encoder_steps(uint8_t motor, float steps);

inline void en_set_step_sign(uint8_t motor, int8_t sign){
	en.en[motor].encoder_sign = sign;
}

inline void en_increment(uint8_t motor){
	en.en[motor].encoder_run += en.en[motor].encoder_sign;
}

inline void en_accumulate(uint8_t motor){
	en.en[motor].encoder_position += en.en[motor].encoder_run;
	en.en[motor].encoder_run = 0;
}

inline int32_t en_read_encoder(uint8_t motor){
	return en.en[motor].encoder_position;
}




#endif

