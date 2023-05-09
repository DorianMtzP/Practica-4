/*
 * lin_driver_test_main.c
 * Created on: Sep 15, 2018
 *     Author: Nico
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "fsl_uart_freertos.h"
#include "fsl_uart.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "driver_lin/lin1d3_driver.h"
#include "FreeRTOSConfig.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
//#define MASTER
//#define SLAVE_A
#define SLAVE_B
//#define SLAVE_C


/* UART instance and clock */
#define MASTER_UART UART3
#define MASTER_UART_CLKSRC UART3_CLK_SRC
#define MASTER_UART_CLK_FREQ CLOCK_GetFreq(UART3_CLK_SRC)
#define MASTER_UART_RX_TX_IRQn UART3_RX_TX_IRQn

/* UART instance and clock */
#define LOCAL_SLAVE_UART UART3
#define LOCAL_SLAVE_UART_CLKSRC UART3_CLK_SRC
#define LOCAL_SLAVE_UART_CLK_FREQ CLOCK_GetFreq(UART3_CLK_SRC)
#define LOCAL_SLAVE_UART_RX_TX_IRQn UART3_RX_TX_IRQn

///* UART instance and clock */
//#define SLAVE_UART UART4
//#define SLAVE_UART_CLKSRC UART4_CLK_SRC
//#define SLAVE_UART_CLK_FREQ CLOCK_GetFreq(UART4_CLK_SRC)
//#define SLAVE_UART_RX_TX_IRQn UART4_RX_TX_IRQn

/* Task priorities. */
#define init_task_PRIORITY (configMAX_PRIORITIES - 2)
#define test_task_heap_size_d	(192)
															 //ID  	par	= hex
#define app_message_id_1_d (0x01<<2|message_size_2_bytes_d)	 //0101  00 =0x14 -> 0b 4bits For the 3LED and the OFF status
#define app_message_id_2_d (0x02<<2|message_size_2_bytes_d)	 //1001  11 =0x27 -> 2bits one per SW
#define app_message_id_3_d (0x03<<2|message_size_2_bytes_d)	 //1101  10 =0x36 ->// 2bits one per SW
//#define app_message_id_4_d (0x04<<2|message_size_2_bytes_d)
//#define app_message_id_5_d (0x05<<2|message_size_4_bytes_d)
//#define app_message_id_6_d (0x06<<2|message_size_8_bytes_d)

/* GPIO pin configuration. */
#define BOARD_LED_GPIO       BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN   BOARD_LED_RED_GPIO_PIN
#define BOARD_SW_GPIO        BOARD_SW3_GPIO
#define BOARD_SW_GPIO_PIN    BOARD_SW3_GPIO_PIN
#define BOARD_SW_PORT        BOARD_SW3_PORT
#define BOARD_SW_IRQ         BOARD_SW3_IRQ
#define BOARD_SW_IRQ_HANDLER BOARD_SW3_IRQ_HANDLER


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void test_task(void *pvParameters);

//static void	message_4_callback_local_slave(void* message);
//static void	message_5_callback_local_slave(void* message);
//static void	message_6_callback_local_slave(void* message);
static void	message_1_callback_local_slave(void* message);
static void	message_2_callback_local_slave(void* message);

static void	message_1_callback_slave(void* message);
static void	message_2_callback_slave(void* message);
//static void	message_3_callback_slave(void* message);
/*******************************************************************************
 * Variables
 ******************************************************************************/
extern volatile bool button2_pressed;
extern volatile bool button1_pressed;
static uint8_t ledStatus = 0;
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitGPIOInterrupts();
    BOARD_InitLEDsPins();
    NVIC_SetPriority(MASTER_UART_RX_TX_IRQn, 5);
  //  NVIC_SetPriority(SLAVE_UART_RX_TX_IRQn, 5);


    if (xTaskCreate(test_task, "test_task", test_task_heap_size_d*2, NULL, init_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("Init Task creation failed!.\r\n");
        while (1)
            ;
    }
    PRINTF(" *** LIN driver demo ***\r\n");
#if defined(MASTER)
    PRINTF(" *** MASTER ***\r\n");
#endif
#if defined(SLAVE_A)
    PRINTF(" *** SLAVE A ***\r\n");
#endif
#if defined(SLAVE_B)
    PRINTF(" *** SLAVE B ***\r\n");
#endif
#if	defined(SLAVE_C)
    PRINTF(" *** SLAVE C ***\r\n");
#endif

    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Task responsible for loopback.
 */
static void test_task(void *pvParameters)
{
	int error;
	lin1d3_nodeConfig_t node_config;
	lin1d3_handle_t* master_handle;
	lin1d3_handle_t* slave_handle;
	lin1d3_handle_t* local_slave_handle;

#if defined(MASTER)
	/* Set Master Config */
	node_config.type = lin1d3_master_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = MASTER_UART;
	node_config.srcclk = MASTER_UART_CLK_FREQ;
	node_config.irq = MASTER_UART_RX_TX_IRQn;
	node_config.skip_uart_init = 0;
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));

	/* Init Master node */
	master_handle = lin1d3_InitNode(node_config);
#endif
#if defined(SLAVE_B) || defined(SLAVE_C)
	/* Set Slave Config */
	node_config.type = lin1d3_slave_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = LOCAL_SLAVE_UART;
	node_config.srcclk = LOCAL_SLAVE_UART_CLK_FREQ;
	node_config.irq = LOCAL_SLAVE_UART_RX_TX_IRQn;
	node_config.skip_uart_init = 0;
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));
	/* Comment this lines if not using a transiver */
	node_config.messageTable[0].ID = app_message_id_1_d;
	node_config.messageTable[0].rx = 1;
	node_config.messageTable[0].handler = message_1_callback_slave;
#if defined(SLAVE_B)
	node_config.messageTable[1].ID = app_message_id_2_d;
	node_config.messageTable[1].rx = 0;
	node_config.messageTable[1].handler = message_2_callback_slave;
#endif
#if defined(SLAVE_C)
	node_config.messageTable[2].ID = app_message_id_3_d;
	node_config.messageTable[2].rx = 0;
	node_config.messageTable[2].handler = message_2_callback_slave;
#endif
	/* Init Slave Node*/
	slave_handle = lin1d3_InitNode(node_config);
#endif
#if defined(SLAVE_A)
	/* Set local Slave Config */
	node_config.type = lin1d3_slave_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = LOCAL_SLAVE_UART;
	node_config.srcclk = LOCAL_SLAVE_UART_CLK_FREQ;
	node_config.irq = LOCAL_SLAVE_UART_RX_TX_IRQn;
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));
	node_config.messageTable[0].ID = app_message_id_1_d;
	node_config.messageTable[0].rx = 0;
	node_config.messageTable[0].handler = message_1_callback_local_slave;
	node_config.messageTable[1].ID = app_message_id_2_d;
	node_config.messageTable[1].rx = 1;
	node_config.messageTable[1].handler = message_2_callback_local_slave;
	/* Comment this lines if only using 2 boards */
	node_config.messageTable[2].ID = app_message_id_3_d;
	node_config.messageTable[2].rx = 1;
	node_config.messageTable[2].handler = message_2_callback_local_slave;
	node_config.skip_uart_init = 1;
	node_config.uart_rtos_handle = master_handle->uart_rtos_handle;
	/* Init local Slave Node*/
	local_slave_handle = lin1d3_InitNode(node_config);
#endif



	if((NULL == master_handle)
#if !defined(JUST_MASTER)
		|| (NULL == slave_handle)
		/*|| (NULL == local_slave_handle)*/
#endif
	   ){
		PRINTF(" Init failed!! \r\n");
		error = kStatus_Fail;
	}
	else {
		error = kStatus_Success;
	}
	int conta = 0;
	while (kStatus_Success == error)
    {
#if defined(MASTER)
		if (conta>9){
			conta =0 ;
    		lin1d3_masterSendMessage(master_handle, app_message_id_1_d);
    		vTaskDelay(100);
    		conta++;
    	}
    	lin1d3_masterSendMessage(master_handle, app_message_id_2_d);
    	vTaskDelay(100);
    	conta++;
    	lin1d3_masterSendMessage(master_handle, app_message_id_3_d);
    	conta++;
#endif
    	vTaskDelay(100);
    }

    vTaskSuspend(NULL);
}



static void	message_1_callback_local_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;

	PRINTF("Slave A got ID 1 request\r\n");
	//Transmit LED status
	switch(ledStatus){
		case 0: //OFF
			GPIO_PortSet(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
			GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1u <<BOARD_LED_GREEN_GPIO_PIN);
			GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_GPIO_PIN);
			message_data[0] = 0x00;
			break;
		case 1:	//RED
			GPIO_PortClear(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
			GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1u <<BOARD_LED_GREEN_GPIO_PIN);
			GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_GPIO_PIN);
			message_data[0] = 0x01;
			break;
		case 2: 	//GREEN
			GPIO_PortSet(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
			GPIO_PortClear(BOARD_LED_GREEN_GPIO, 1u <<BOARD_LED_GREEN_GPIO_PIN);
			GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_GPIO_PIN);
			message_data[0] = 0x02;
			break;
		case 3:		//blue
			GPIO_PortSet(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
			GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1u <<BOARD_LED_GREEN_GPIO_PIN);
			GPIO_PortClear(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_GPIO_PIN);
			message_data[0] = 0x03;
			break;
	}
	ledStatus++;
	if (ledStatus >3)
		ledStatus = 0;

//	PRINTF(" %b " , app_message_id_1_d);
	//message_data[0] = 79;
	message_data[1] = 0xf0;
}

static void	message_2_callback_local_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave A ID2 or ID 3 callback \r\n");
	if(message_data[0] == 0x00){				//0b00
		PRINTF("SW2 & SW3 Libres \r\n");
	}else if(message_data[0] == 0x01){			//0b10
		PRINTF("SW2 Presionado & SW3 Libre \r\n");
	}else if(message_data[0] == 0x02){			//0b01
		PRINTF("SW2 Libre & SW3 Presionado  \r\n");
	}else if(message_data[0] == 0x03){			//0b11
		PRINTF("SW2 & SW3 Presionados  \r\n");
	}else{
		PRINTF("Slave reciived wrong data %d,%d\r\n", message_data[0], message_data[1]);
	}
	//PRINTF("Local Slave got response to message 1 %d,%d\r\n", message_data[0], message_data[1]);
}

static void	message_1_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave B or C ID1 callback \r\n");
	if(message_data[0] == 0x00){
		PRINTF("LED OFF \r\n");
	}else if(message_data[0] == 0x01){
		PRINTF("RED LED ON \r\n");
	}else if(message_data[0] == 0x02){
		PRINTF("GREEN LED ON \r\n");
	}else if(message_data[0] == 0x03){
		PRINTF("BLUE LED ON \r\n");
	}else{
		PRINTF("Slave a wrong data %d,%d\r\n", message_data[0], message_data[1]);
	}
	//PRINTF("Local Slave got response to message 1 %d,%d\r\n", message_data[0], message_data[1]);
}
static void	message_2_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave B or C ID2 or 3 callback \r\n");

	if(button1_pressed == 0 && button2_pressed == 0 ){
		message_data[0] = 0x00;		//0b00
		PRINTF("SW2 & SW3 Libres \r\n");
	}
	else if(button1_pressed == 1 && button2_pressed == 0 ){
		message_data[0] = 0x02;		//0b10
		button1_pressed = 0;
		PRINTF("SW2 Presionado & SW3 Libre \r\n");
		GPIO_PortClear(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
	}
	else if(button1_pressed == 0 && button2_pressed == 1 ){
		message_data[0] = 0x01;		//0b01
		PRINTF("SW2 Libre & SW3 Presionado \r\n");
		button2_pressed = 0;
		GPIO_PortClear(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
	}
	else if(button1_pressed == 1 && button2_pressed == 1 ){
		message_data[0] = 0x03;		//0b11
		button1_pressed = 0;
		button2_pressed = 0;
		PRINTF("SW2 & SW3 Presionados \r\n");
		GPIO_PortClear(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
	}
	else{
		PRINTF("Slave got wrong data %d,%d\r\n", message_data[0], message_data[1]);
	}
	message_data[1]= 0x0f;
	//PRINTF("Local Slave got response to message 1 %d,%d\r\n", message_data[0], message_data[1]);
}

