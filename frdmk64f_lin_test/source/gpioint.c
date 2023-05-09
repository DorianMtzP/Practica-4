#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_common.h"

#include "pin_mux.h"
#include "clock_config.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

volatile bool button2_pressed;
volatile bool button1_pressed;


/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Interrupt service function of switch SW3.
 *
 */
void BOARD_SW2_IRQ_HANDLER(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	button1_pressed = 1;
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BOARD_SW2_GPIO, 1U << BOARD_SW2_GPIO_PIN);

    /* Now the message was sent, and the interrupt source has been cleared, a context
	switch should be performed if xHigherPriorityTaskWoken is equal to pdTRUE.
	NOTE: The syntax required to perform a context switch from an ISR varies from
	port to port, and from compiler to compiler. Check the web documentation and
	examples for the port being used to find the syntax required for your
	application. */
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

/*!
 * @brief Interrupt service function of switch SW3.
 *
 */
void BOARD_SW3_IRQ_HANDLER(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	button2_pressed = 1;
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BOARD_SW3_GPIO, 1U << BOARD_SW3_GPIO_PIN);

    /* Now the message was sent, and the interrupt source has been cleared, a context
	switch should be performed if xHigherPriorityTaskWoken is equal to pdTRUE.
	NOTE: The syntax required to perform a context switch from an ISR varies from
	port to port, and from compiler to compiler. Check the web documentation and
	examples for the port being used to find the syntax required for your
	application. */
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void BOARD_InitGPIOInterrupts (void)
{
	int result = 0;

    /* Define the init structure for the input switch pin */
    gpio_pin_config_t sw3_config = {
        kGPIO_DigitalInput,
        0,
    };
    gpio_pin_config_t sw2_config = {
           kGPIO_DigitalInput,
           0,
       };

    PORT_SetPinInterruptConfig(BOARD_SW3_PORT, BOARD_SW3_GPIO_PIN, kPORT_InterruptFallingEdge);
	PORT_SetPinInterruptConfig(BOARD_SW2_PORT, BOARD_SW2_GPIO_PIN, kPORT_InterruptFallingEdge);

	EnableIRQ(BOARD_SW3_IRQ);
	EnableIRQ(BOARD_SW2_IRQ);

	GPIO_PinInit(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN, &sw3_config);
	GPIO_PinInit(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN, &sw2_config);


    //Set PORTA Interrupt level to 3 (higher than SYSCALL), configMAX_SYSCALL_INTERRUPT_PRIORITY priority is 2.
    result = NVIC_GetPriority(PORTA_IRQn);
	NVIC_SetPriority(PORTA_IRQn,3);        //PORTA vector is 5
	result = NVIC_GetPriority(PORTC_IRQn);
	NVIC_SetPriority(PORTC_IRQn,4);

	/* Define the init structure for the output LED pin*/
	gpio_pin_config_t led_config = {
		kGPIO_DigitalOutput,
		0,
	};
	/* PORTB2 is configured as PTB2 */
	PORT_SetPinMux(PORTB, 2U, kPORT_MuxAsGpio);
	/* PORTB3 is configured as PTB3 */
	PORT_SetPinMux(PORTB, 3U, kPORT_MuxAsGpio);
	/* PORTB10 is configured as PTB10 */
	PORT_SetPinMux(PORTB, 10U, kPORT_MuxAsGpio);
	/* PORTB11 is configured as PTB11 */
	PORT_SetPinMux(PORTB, 11U, kPORT_MuxAsGpio);

	GPIO_PinInit(GPIOB, 2,  &led_config);
	GPIO_PinInit(GPIOB, 3,  &led_config);
	GPIO_PinInit(GPIOB, 10, &led_config);
	GPIO_PinInit(GPIOB, 11, &led_config);

}



/***********************************************************
 *  LEDs
 * ********************************************************/
void BOARD_InitLEDsPins(void)
{
    /* Port B Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* Port E Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortE);

    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput,
        0,
    };

    /* Port B Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortB);

    /* RED*/
    PORT_SetPinMux(PORTB, 22U, kPORT_MuxAsGpio);
    /* GREEN */
	PORT_SetPinMux(PORTE, 26U, kPORT_MuxAsGpio);
    /* BLUE */
    PORT_SetPinMux(PORTB, 21U, kPORT_MuxAsGpio);


    /* RED*/
    GPIO_PinInit(GPIOB, 22U,  &led_config);
    /* GREEN */
    GPIO_PinInit(GPIOE, 26U, &led_config);
    /* BLUE */
    GPIO_PinInit(GPIOB, 21U,  &led_config);

    GPIO_PortSet(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
    GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1u <<BOARD_LED_GREEN_GPIO_PIN);
    GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_GPIO_PIN);


}


