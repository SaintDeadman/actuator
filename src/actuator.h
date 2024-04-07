#ifndef _ACTUATOR_H_
#define _ACTUATOR_H_
#include "stm32f4xx_hal.h"

#define ACTUATOR_BEHAVIOR_IRQ     0 /* not implemented. (EXTI,TIM) */
#define ACTUATOR_BEHAVIOR_POLLING 1

#define ACTUATOR_CALIBRATION_STATISTICS 0
#define ACTUATOR_CALIBRATION_FILTER     1 /* not implemented */
#define ACTUATOR_CALIBRATION_NO         2
#define ACTUATOR_CALIBRATION_YES        3

#define ACTUATOR_INIT_HW_NO  0
#define ACTUATOR_INIT_HW_YES 1 /* not implemented */

/* Configuration */
#define ACTUATOR_INIT_HW						(ACTUATOR_INIT_HW_NO)
#define ACTUATOR_BEHAVIOR_MODE 					(ACTUATOR_BEHAVIOR_POLLING)
#define ACTUATOR_CALIBRATION_MODE				(ACTUATOR_CALIBRATION_STATISTICS)
#define ACTUATOR_CALIBRATION_ERROR_ESTIMATION	(ACTUATOR_CALIBRATION_NO)
#define ACTUATOR_CALIBRATION_ITERATION  		(4)
#define ACTUATOR_CALIBRATION_SAVE				(ACTUATOR_CALIBRATION_NO)
#define ACTUATOR_SAFETY_INTERVAL				(0x02)

typedef struct actuator_t* actuator_ptr_t;
typedef uint8_t fixed_point_t;

typedef struct {
	GPIO_TypeDef* GPIOx;
	uint16_t 	  GPIO_Pin;
	GPIO_PinState active_lvl;
} switch_t;

typedef struct {
	/* left  side */
	switch_t* left_limited_switch;
	switch_t* left_moved_switch;
	/* right side */
	switch_t* right_moved_switch;
	switch_t* right_limited_switch;
} actuator_conf_hw;

typedef enum  {
	idle,
	busy,
    error_general,
	error_setting_position,
	error_calibration,
} actuator_status;


/**
 * @brief Allocate memory for an actuator structure on the stack.
 *
 * This macro is used to allocate memory for an actuator structure on the stack.
 *
 * @param act Pointer to the actuator structure variable.
 */
#define actuator_new(act) \
	uint8_t act##_buf[sizeof_struct_actuator_t()];\
	act=(struct actuator_t*)act##_buf;

/**
 * @brief Get the size of the actuator structure.
 *
 * This function is a helper function intended to hide the implementation details
 * of the actuator structure. It returns the size of the actuator structure in bytes.
 *
 * @return The size of the actuator structure in bytes.
 */
const int sizeof_struct_actuator_t();

/**
 * @brief Initialize the actuator.
 *
 * This function initializes the actuator using the provided hardware configuration.
 *
 * @param act Pointer to the actuator structure.
 * @param conf_hw Pointer to the hardware configuration structure.
 */
void actuator_init(actuator_ptr_t act, actuator_conf_hw* conf_hw);

/**
 * @brief Set the position of the actuator.
 *
 * @param act Pointer to the actuator structure.
 * @param position Position of the actuator, represented in fixed-point format without integer part,
 *                 where 0x00 corresponds to 0, and 0xFF corresponds to 0.99 (99%).
 *                 Note: The value 0xFF represents the maximum possible position of the actuator.
 */
actuator_status actuator_set_position(actuator_ptr_t act, fixed_point_t position);
#define actuator_homing(act) actuator_set_position(act, 0x80)



/*TODO*/
//void EXTI_handler(actuator_ptr_t);
//void TIM_handler(actuator_ptr_t);

#endif //_ACTUATOR_H_
