#include <stdint.h>
#include <string.h>

#include "actuator.h"
#include "stm32f4xx_hal.h"

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define SET_UNDEFINE_VAL(x) ((x) = (__typeof__(x))0xFFFFFFFF)
#define IS_UNDEFINE_VAL(x) ((__typeof__(x))(x) == (__typeof__(x))0xFFFFFFFF)
#define IN_RANGE(value, min, max) (((value) >= (min)) && ((value) <= (max)))

#define ACT_SIGNAL_POSITION_UNDEF_Pos      (0U)
#define ACT_SIGNAL_POSITION_UNDEF_Msk      (0x1UL << ACT_SIGNAL_POSITION_UNDEF_Pos)
#define ACT_SIGNAL_POSITION_UNDEF 		    ACT_SIGNAL_POSITION_UNDEF_Msk

#define ACT_SIGNAL_CALIBRATED_Pos          (1U)
#define ACT_SIGNAL_CALIBRATED_Msk          (0x1UL << ACT_SIGNAL_CALIBRATED_Pos)
#define ACT_SIGNAL_CALIBRATED               ACT_SIGNAL_CALIBRATED_Msk

#define ACT_SIGNAL_POSITION_SETTING_Pos    (2U)
#define ACT_SIGNAL_POSITION_SETTING_Msk    (0x1UL << ACT_SIGNAL_POSITION_SETTING_Pos)
#define ACT_SIGNAL_POSITION_SETTING         ACT_SIGNAL_POSITION_SETTING_Msk


#define IS_SIGNAL_SET(act, signal)        (((act->fsm_signals) & (signal)) != 0)
#define SET_SIGNAL(act, signal)           ((act->fsm_signals) |= (signal))
#define CLEAR_SIGNAL(act, signal)         ((act->fsm_signals) &= ~(signal))

typedef enum {
	STATE_IDLE = 0,
	STATE_START_CALB,
	STATE_END_CALB,
	STATE_MOV,
	STATE_ERROR
} fsm_name_states;

typedef enum {
	LEFT = 0,
	RIGHT = 1
} moving_direction;

typedef struct {
	switch_t limited;
	switch_t ctrl;
} side_hw_t;

typedef struct {
	fixed_point_t cur;
	fixed_point_t new;
	fixed_point_t safety_low;
	fixed_point_t safety_hi;
} position_t;

#pragma pack(push, 1)
typedef struct {
	uint8_t iter;
	/* left len in ticks*/
	uint32_t l_len_avr;
	uint8_t l_len_dispersion;
	/* right len in ticks*/
	uint32_t r_len_avr;
	uint8_t r_len_dispersion;
} calibration_t;
#pragma pack(pop)

#pragma pack(push, 1)
struct actuator_t {
    side_hw_t sides[2];
    position_t position;
    calibration_t calibration;
    /* FSM */
    fsm_name_states cur_state;
    uint8_t fsm_signals;

    /* action */
    uint32_t needed_tick;
    uint32_t start_time_stamp;
    uint32_t stop_time_stamp;
};
#pragma pack(pop)


static void start_moving(actuator_ptr_t act, moving_direction dir);
static void stop_moving(actuator_ptr_t act);
static uint32_t time_span(uint32_t t0, uint32_t t1);
static uint8_t is_limit(actuator_ptr_t act, moving_direction dir);
static actuator_status actuator_get_status(actuator_ptr_t);

void actuator_init(actuator_ptr_t act, actuator_conf_hw* conf_hw) {
	memset(act, 0, sizeof(struct actuator_t));
	/* -------------init HW --------------------*/
	act->sides[LEFT].limited = *conf_hw->left_limited_switch;
	act->sides[LEFT].ctrl = *conf_hw->left_moved_switch;
	act->sides[RIGHT].limited = *conf_hw->left_limited_switch;
	act->sides[RIGHT].ctrl = *conf_hw->left_moved_switch;
	/*--------------position--------------------*/
	act->position.safety_low = ACTUATOR_SAFETY_INTERVAL;
	act->position.safety_hi = 0xFF - ACTUATOR_SAFETY_INTERVAL;
	/* -------------init FSM ------------------ */
	act->cur_state = STATE_IDLE;
	CLEAR_SIGNAL(act, ACT_SIGNAL_POSITION_SETTING);
	CLEAR_SIGNAL(act, ACT_SIGNAL_CALIBRATED);
	SET_SIGNAL(act, ACT_SIGNAL_POSITION_UNDEF);
	/*-----------init calibration --------------*/
	act->calibration.iter = ACTUATOR_CALIBRATION_ITERATION;
	act->calibration.l_len_avr = act->calibration.r_len_avr = 0;
	act->calibration.l_len_dispersion = act->calibration.r_len_dispersion = 1;

	/*---------------------------*/
	SET_UNDEFINE_VAL(act->needed_tick);

	stop_moving(act);
}


const int sizeof_struct_actuator_t() {
    return sizeof(struct actuator_t);
}

/*TODO move to utile */
static inline uint32_t int_multiply_fixed_point(int32_t integer, fixed_point_t fixed_point) {
    return (integer * (int32_t)fixed_point) >> 8;
}

/*TODO move to utile */
static inline uint32_t time_span(uint32_t t0, uint32_t t1) {
	if (t1 >= t0) return t1 - t0;
	else return ((uint32_t)-1 - t0) + t1 + 1;
}

//============================================================================
//							HW
//============================================================================
static void start_moving(actuator_ptr_t act, moving_direction dir)
{
	switch_t* ctrl_switch = &act->sides[dir].ctrl;
	HAL_GPIO_WritePin(ctrl_switch->GPIOx, ctrl_switch->GPIO_Pin, ctrl_switch->active_lvl);
	/*save time*/
	act->start_time_stamp = HAL_GetTick();
}

static void stop_moving(actuator_ptr_t act) {
	/*stop left*/
	switch_t* ctrl_switch = &act->sides[LEFT].ctrl;
	HAL_GPIO_WritePin(ctrl_switch->GPIOx, ctrl_switch->GPIO_Pin, !ctrl_switch->active_lvl);
	/*stop right*/
	ctrl_switch = &act->sides[RIGHT].ctrl;
	HAL_GPIO_WritePin(ctrl_switch->GPIOx, ctrl_switch->GPIO_Pin, !ctrl_switch->active_lvl);
	/*save time*/
	act->stop_time_stamp = HAL_GetTick();
}

static uint8_t is_limit(actuator_ptr_t act, moving_direction dir){
	switch_t* limited_switch = &act->sides[LEFT].limited;
	return limited_switch->active_lvl == HAL_GPIO_ReadPin(limited_switch->GPIOx, limited_switch->GPIO_Pin);
}

//=========================================================================
// 							FSM
//========================================================================
struct fsm_state {
	fsm_name_states (*fn_worker)(actuator_ptr_t);
};

static fsm_name_states IDLE_handler(actuator_ptr_t);
static fsm_name_states START_CALB_handler(actuator_ptr_t);
static fsm_name_states END_CALB_handler(actuator_ptr_t);
static fsm_name_states MOV_handler(actuator_ptr_t);
static fsm_name_states ERROR_handler(actuator_ptr_t);

/* TODO rewrite to using signals like fsm[state][signals] = {fn_work, next_state} */
struct fsm_state fsm[] = {
		[STATE_IDLE ... STATE_ERROR] = {ERROR_handler},
		[STATE_IDLE] = {IDLE_handler},
		[STATE_START_CALB] = {START_CALB_handler},
		[STATE_END_CALB] = {END_CALB_handler},
		[STATE_MOV] = {MOV_handler}
};
void updating_FSM(actuator_ptr_t act)
{
	struct fsm_state* cur_state = &fsm[act->cur_state];
	act->cur_state = cur_state->fn_worker(act);
}

//==============================================================
//						IDLE
//==============================================================
static fsm_name_states IDLE_handler(actuator_ptr_t act)
{
	/* checking moving into IDLE state*/
	if(is_limit(act, LEFT) || is_limit(act, RIGHT)) {
		stop_moving(act);
		return STATE_ERROR;
	}

	/*check calibration*/
	if( !IS_SIGNAL_SET(act, ACT_SIGNAL_CALIBRATED) ) {return STATE_START_CALB;}
	/* TODO add new stage for it */
	if(IS_SIGNAL_SET(act, ACT_SIGNAL_POSITION_SETTING)) {

		/* check range */
		if(IN_RANGE(act->position.new, act->position.cur - 0x04, act->position.cur + 0x04)) {
			CLEAR_SIGNAL(act, ACT_SIGNAL_POSITION_SETTING);
			return STATE_IDLE;
		}

		/* calc action params */
		uint8_t delta_position = (act->position.cur > act->position.new) ? (act->position.cur - act->position.new) : (act->position.new - act->position.cur);
		uint8_t direction = (act->position.cur > act->position.new) ? LEFT : RIGHT;
		act->needed_tick = HAL_GetTick() + int_multiply_fixed_point((direction == LEFT) ? act->calibration.l_len_avr : act->calibration.r_len_avr, delta_position);

		start_moving(act, direction);
		return STATE_MOV;
	}
	return STATE_IDLE;
}

//==============================================================
//						CALIBRATION
//==============================================================
static fsm_name_states START_CALB_handler(actuator_ptr_t act) {
	if(IS_SIGNAL_SET(act, ACT_SIGNAL_POSITION_UNDEF)) {
		start_moving(act, RIGHT);
		return STATE_MOV;
	}

	if(act->calibration.iter) {
		start_moving(act, act->calibration.iter%2);
		return STATE_MOV;
	}
	return STATE_IDLE;
}

static fsm_name_states END_CALB_handler(actuator_ptr_t act) {
	if(IS_SIGNAL_SET(act, ACT_SIGNAL_POSITION_UNDEF)) {
		CLEAR_SIGNAL(act, ACT_SIGNAL_POSITION_UNDEF);
		return STATE_START_CALB;
	}

	/*accumulate lens*/
	if(act->calibration.iter) {
		uint32_t span = time_span(act->start_time_stamp, act->stop_time_stamp);
		if(act->calibration.iter%2) act->calibration.l_len_avr += span;
		else act->calibration.r_len_avr += span;
	}
	if(act->calibration.iter--) return STATE_START_CALB;

	/*calculate average values*/
	act->calibration.r_len_avr = (act->calibration.r_len_avr/ACTUATOR_CALIBRATION_ITERATION) >> 1;
	act->calibration.l_len_avr = (act->calibration.l_len_avr/ACTUATOR_CALIBRATION_ITERATION) >> 1;

	SET_SIGNAL(act,ACT_SIGNAL_CALIBRATED);
	return STATE_IDLE;
}

//==============================================================
//						MOVING
//==============================================================
static fsm_name_states MOV_handler(actuator_ptr_t act) {
	/* check left limited switch */
	if(is_limit(act, LEFT)) {
		stop_moving(act);
		act->position.cur = 0;

		if(!IS_SIGNAL_SET(act, ACT_SIGNAL_CALIBRATED))
			return STATE_END_CALB;
		else
			return STATE_IDLE;
	}

	/* check right limited switch */
	if(is_limit(act, RIGHT)) {
		stop_moving(act);
		act->position.cur = 0xFF;

		if(!IS_SIGNAL_SET(act, ACT_SIGNAL_CALIBRATED))
			return STATE_END_CALB;
		else
			return STATE_IDLE;
	}


	if(!IS_UNDEFINE_VAL(act->needed_tick)) {
		uint32_t current_tick = HAL_GetTick();
		if(current_tick >= act->needed_tick){
			stop_moving(act);
			act->position.cur = act->position.new;

			return STATE_IDLE;
		}
	}

	return STATE_MOV;
}
//=========================================================================
//						 ERROR
//=========================================================================
static fsm_name_states ERROR_handler(actuator_ptr_t) {
	return STATE_IDLE;
}



//=========================================================================
//						 SET POSITION
//=========================================================================
actuator_status actuator_set_position(actuator_ptr_t act, fixed_point_t position) {

	if(act->cur_state == STATE_IDLE) {
		SET_SIGNAL(act, ACT_SIGNAL_POSITION_SETTING);
		act->position.new = MIN(MAX(position, act->position.safety_low), act->position.safety_hi);
	}
	updating_FSM(act);
	return actuator_get_status(act);
}


static actuator_status actuator_get_status(actuator_ptr_t act)
{
	switch (act->cur_state) {
		case STATE_IDLE: return idle;
		/*TODO add a check of signals*/
		case STATE_ERROR: return error_general;
		default:
				return busy;
			break;
	}
}

