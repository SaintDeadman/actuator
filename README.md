# actuator

This code is part of a test task for an Firmware Developer position.
The task description can be found in the doc.


# Key Points

- Calculating the distance between two extreme points: Calibration of the device is required before using the main movement function. Calibration involves measuring the average time to travel between two extreme points. The speeds of movement may vary in different directions. It is also necessary to calculate the squares of errors in changes.

- Accuracy of setting the position: It is also important to consider the accumulated installation error (error square).

# Features

- Using a state machine for device control:
   A finite state machine is applied for device control, providing convenience in programming and state tracking.

- Calibration launch if device is not calibrated:
   Automatic launch of the calibration process if the device has not been calibrated.

- Data hiding:
   Data hiding mechanism is used to ensure code security and modularity.

- Safety interval for position setting:
   Checking the position for safety interval before performing movement.

# TODO List

Although the following list outlines various features and enhancements, due to time constraints, not all of them were implemented:

- Add EXTI interrupts for motion limitation:
   Use external interrupts for motion limitation of the device.

- Use a timer interrupts for position setting:
   Apply a timer for position setting with control over time intervals.

- Add saving of calibration information to non-volatile memory:
   Implement saving of calibration results to non-volatile memory to preserve data between reboots.

- Add signals to the finite state machine:
   Expand the finite state machine by adding signals for more flexible device control.

- Calculate variance and accumulated error to trigger recalibration:
   Implement functionality to calculate variance and accumulated error for automatic triggering of recalibration process when necessary.

- One degree of freedom (compression-extension) as a special case:
   Analyze and discuss the possibility of extending the functionality to consider multi-dimensional control options.


# How To Use It
```C
...
static actuator_ptr_t actuator;
...

int main()
{
    ...
    /**/
    actuator_new(actuator);

    actuator_conf_hw conf_hw = {
        .left_limited_switch  = &(switch_t){.GPIOx=GPIOA, .GPIO_Pin= GPIO_PIN_0, .active_lvl = GPIO_PIN_RESET},
        .left_moved_switch    = &(switch_t){.GPIOx=GPIOC, .GPIO_Pin= GPIO_PIN_13, .active_lvl = GPIO_PIN_RESET},
        .right_limited_switch = &(switch_t){.GPIOx=GPIOA, .GPIO_Pin= GPIO_PIN_0, .active_lvl = GPIO_PIN_RESET},
        .right_moved_switch   = &(switch_t){.GPIOx=GPIOC, .GPIO_Pin= GPIO_PIN_13, .active_lvl = GPIO_PIN_RESET} 
    };

    actuator_init(actuator, &conf_hw);
    ...
    while(42)
    {
        ...
        actuator_homing(actuator);
        ...
    }    
}
```