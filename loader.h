#ifndef LOADER_H
#define LOADER_H



enum prepBufferState {
	PREP_BUFFER_OWNED_BY_LOADER = 0,	// staging buffer is ready for load
	PREP_BUFFER_OWNED_BY_EXEC			// staging buffer is being loaded
};

typedef struct  loader{
	void(*load_move)(void);
	void(*get_target_units)(float* , float*);
	stat_t(*prep_line)(float , float*, float*);
	uint8_t (*actuator_runtime_isbusy)(void);
	uint8_t buffer_state;
	uint8_t move_type;
}load_t;

extern load_t ld;

void ld_init(void);
void ld_request_load(void);
void ld_request_exe(void);

#endif

