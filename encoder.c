
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "system.h"
#include "encoder.h"

Encoders_t en;

void encoder_init(void)
{
	memset(&en, 0, sizeof(en));		// clear all values, pointers and status
}

void en_set_encoder_steps(uint8_t motor, float steps)
{
	en.en[motor].encoder_position = (int32_t)roundf(steps);
}
