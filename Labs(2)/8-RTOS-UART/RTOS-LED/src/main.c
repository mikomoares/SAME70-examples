/*
* Example RTOS Atmel Studio
*/

#include <asf.h>
#include <string.h>

#include "conf_board.h"

#define LED1_PIO			PIOA
#define LED1_PIO_ID			ID_PIOA
#define LED1_PIO_IDX		0
#define LED1_PIO_IDX_MASK	(1 << LED1_PIO_IDX)

#define LED2_PIO PIOC
#define LED2_PIO_ID ID_PIOC
#define LED2_IDX 30
#define LED2_PIO_IDX_MASK (1 << LED2_IDX)

#define LED3_PIO PIOB
#define LED3_PIO_ID ID_PIOB
#define LED3_IDX 2
#define LED3_PIO_IDX_MASK (1 << LED3_IDX)

#define BUT1_PIO            PIOD
#define BUT1_PIO_ID         16
#define BUT1_PIO_IDX        28
#define BUT1_PIO_IDX_MASK   (1u << BUT1_PIO_IDX)

#define BUT2_PIO PIOC
#define BUT2_PIO_ID ID_PIOC
#define BUT2_PIO_IDX 31
#define BUT2_PIO_IDX_MASK (1u << BUT2_PIO_IDX)

#define BUT3_PIO PIOA
#define BUT3_PIO_ID ID_PIOA
#define BUT3_IDX 19
#define BUT3_PIO_IDX_MASK (1 << BUT3_IDX)


#define TASK_MONITOR_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY)
#define TASK_LED_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY            (tskIDLE_PRIORITY)
#define TASK_LED1_STACK_SIZE (1024/sizeof(portSTACK_TYPE))
#define TASK_LED1_STACK_PRIORITY (tskIDLE_PRIORITY)

SemaphoreHandle_t xSemaphore;
SemaphoreHandle_t xSemaphore2;
SemaphoreHandle_t xSemaphore3;
QueueHandle_t xQueueRx;
QueueHandle_t xQueueExecuta;
QueueHandle_t xQueueLed3;


extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

void but1_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	printf("but_callback 1 \n");
	xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
	printf("semafaro tx \n");
}

void but2_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	printf("but_callback 2\n");
	xSemaphoreGiveFromISR(xSemaphore2, &xHigherPriorityTaskWoken);
	printf("semafaro tx 2\n");
}

void but3_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	printf("but_callback 3\n");
	xSemaphoreGiveFromISR(xSemaphore3, &xHigherPriorityTaskWoken);
	printf("semafaro tx 3\n");
}
void USART1_Handler(void){
	uint32_t ret = usart_get_status(USART1);

	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	char c;

	// Verifica por qual motivo entrou na interrup�cao?
	// RXRDY ou TXRDY

	//  Dados dispon�vel para leitura
	if(ret & US_IER_RXRDY){
		usart_serial_getchar(USART1, &c);
		xQueueSendFromISR(xQueueRx, &c, &xHigherPriorityTaskWoken);

		// -  Transmissoa finalizada
		} else if(ret & US_IER_TXRDY){

	}
}


/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}
uint32_t usart1_puts(uint8_t *pstring){
	uint32_t i ;

	while(*(pstring + i))
	if(uart_is_tx_empty(USART1))
	usart_serial_putchar(USART1, *(pstring+i++));
}

static void task_UartRx(void *pvParameters){

	char rxMSG;
	char msgBuffer[64] = {0};
	int i = 0;
	
	xQueueRx = xQueueCreate(32, sizeof(char));
	
	for (;;){
		if (xQueueReceive(xQueueRx, &rxMSG, (TickType_t)500)){
			if (rxMSG == '\n'){
				msgBuffer[i] = 0;
				xQueueSend(xQueueExecuta, &msgBuffer, 0);
				i = 0;
			}else {
				msgBuffer[i] = rxMSG;
				i ++;
			}
		}
	}
}

static void task_executa(void *pvParameters){
	char buffer[64];
	xQueueExecuta = xQueueCreate(5, sizeof(char[64]));
	int inst;
	
	for (;;){
		if (xQueueReceive(xQueueExecuta, &buffer,(TickType_t)500)){
			printf("cu %s \n",buffer);
			
			if (strcmp(buffer, "led 1 on") == 0){
				inst = 1;
				xQueueSend(xQueueLed3, &inst, 0);
			}if (strcmp(buffer, "led 1 on") == 0){
				inst = 0;
				xQueueSend(xQueueLed3, &inst, 0);
		}
		}
	}
}

/**
 * \brief This task, when activated, send every ten seconds on debug UART
 * the whole report of free heap and total tasks status
 */
static void task_monitor(void *pvParameters)
{
	static portCHAR szList[256];
	//UNUSED(pvParameters);
    const TickType_t xDelay = 3000 / portTICK_PERIOD_MS;

	

	for (;;) {
		printf("--- task ## %u\n", (unsigned int)uxTaskGetNumberOfTasks());
		vTaskList((signed portCHAR *)szList);
		printf(szList);
		vTaskDelay(xDelay);
	}
}

/**
 * \brief This task, when activated, make LED blink at a fixed rate
 */
static void task_led(void *pvParameters)
{
	xSemaphore = xSemaphoreCreateBinary();
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP);
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but1_callback);
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4);
	
	//UNUSED(pvParameters);
    const TickType_t xDelay = 2000 / portTICK_PERIOD_MS;
	
	if (xSemaphore == NULL)
	  printf("falha em criar o semaforo \n");

	for (;;) {
		if( xSemaphoreTake(xSemaphore, ( TickType_t ) 500) == pdTRUE ){
			LED_Toggle(LED0);
		}
	}
}

static void task_led1(void *pvParameters)
{
	
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_PIO_IDX_MASK, PIO_DEFAULT);
	//UNUSED(pvParameters);
	const TickType_t xDelay = 3000 / portTICK_PERIOD_MS;
	const TickType_t xDelayLed = 200 / portTICK_PERIOD_MS;
	
	for (;;) {
		for (uint i=0; i<5; i++){
		    pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
		    vTaskDelay(xDelayLed);
		    pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
		    vTaskDelay(xDelayLed);
	    }
	    vTaskDelay(xDelay);
	}
}

static void task_led2(void *pvParameters)
{
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_PIO_IDX_MASK, PIO_DEFAULT);
	xSemaphore2 = xSemaphoreCreateBinary();
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but2_callback);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 4);
	
	//UNUSED(pvParameters);
	const TickType_t xDelay = 2000 / portTICK_PERIOD_MS;
	
	if (xSemaphore2 == NULL)
	printf("falha em criar o semaforo \n");

	for (;;) {
		if( xSemaphoreTake(xSemaphore2, ( TickType_t ) 500) == pdTRUE ){
			for (uint i=0; i<5; i++){
				pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
				vTaskDelay(200 / portTICK_PERIOD_MS);
				pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
				vTaskDelay(200 / portTICK_PERIOD_MS);
			}
			vTaskDelay(xDelay);
		}
	}
}
static void task_led3(void *pvParameters)
{
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_configure(LED3_PIO, PIO_OUTPUT_0, LED3_PIO_IDX_MASK, PIO_DEFAULT);
	xSemaphore3 = xSemaphoreCreateBinary();
	pmc_enable_periph_clk(BUT3_PIO_ID);
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but3_callback);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 4);
	
	//UNUSED(pvParameters);
	const TickType_t xDelay = 2000 / portTICK_PERIOD_MS;
	
	if (xSemaphore3 == NULL)
	printf("falha em criar o semaforo \n");
	
	int liga;
	xQueueLed3 = xQueueCreate(5, sizeof(int));

	for (;;) {
		
		if (xQueueReceive(xQueueLed3, &liga, (TickType_t)500)){
			if (liga){
				pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
			} else {
				pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
			}
		}
		
		if( xSemaphoreTake(xSemaphore3, ( TickType_t ) 500) == pdTRUE ){
			for (uint i=0; i<5; i++){
				pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
				vTaskDelay(200 / portTICK_PERIOD_MS);
				pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
				vTaskDelay(200 / portTICK_PERIOD_MS);
			}
			vTaskDelay(xDelay);
		}
	}
}

/**
 * \brief Configure the console UART.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
#if defined(__GNUC__)
	setbuf(stdout, NULL);
#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	 * emits one character at a time.
	 */
#endif
}

static void USART1_init(void){
	/* Configura USART1 Pinos */
	sysclk_enable_peripheral_clock(ID_PIOB);
	sysclk_enable_peripheral_clock(ID_PIOA);
	pio_set_peripheral(PIOB, PIO_PERIPH_D, PIO_PB4); // RX
	pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA21); // TX
	MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;

	/* Configura opcoes USART */
	const sam_usart_opt_t usart_settings = {
		.baudrate       = 115200,
		.char_length    = US_MR_CHRL_8_BIT,
		.parity_type    = US_MR_PAR_NO,
		.stop_bits    = US_MR_NBSTOP_1_BIT    ,
		.channel_mode   = US_MR_CHMODE_NORMAL
	};

	/* Ativa Clock periferico USART0 */
	sysclk_enable_peripheral_clock(ID_USART1);

	stdio_serial_init(CONF_UART, &usart_settings);

	/* Enable the receiver and transmitter. */
	usart_enable_tx(USART1);
	usart_enable_rx(USART1);

	/* map printf to usart */
	ptr_put = (int (*)(void volatile*,char))&usart_serial_putchar;
	ptr_get = (void (*)(void volatile*,char*))&usart_serial_getchar;

	/* ativando interrupcao */
	usart_enable_interrupt(USART1, US_IER_RXRDY);
	NVIC_SetPriority(ID_USART1, 4);
	NVIC_EnableIRQ(ID_USART1);
}


/**
 *  \brief FreeRTOS Real Time Kernel example entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	USART1_init();

	/* Output demo information. */
	printf("-- Freertos Example --\n\r");
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);


	/* Create task to monitor processor activity */
	if (xTaskCreate(task_monitor, "Monitor", TASK_MONITOR_STACK_SIZE, NULL,
			TASK_MONITOR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Monitor task\r\n");
	}

	/* Create task to make led blink */
	if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL,
			TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	if 	(xTaskCreate(task_led1, "Led1", TASK_LED1_STACK_SIZE, NULL, TASK_LED1_STACK_PRIORITY, NULL)
 != pdPASS) {
		printf("Failed to create test led task 1\r\n");
	}
	if 	(xTaskCreate(task_led2, "Led2", TASK_LED1_STACK_SIZE, NULL, TASK_LED1_STACK_PRIORITY, NULL)
	!= pdPASS) {
		printf("Failed to create test led task 2\r\n");
	}
	if 	(xTaskCreate(task_led3, "Led3", TASK_LED1_STACK_SIZE, NULL, TASK_LED1_STACK_PRIORITY, NULL)
	!= pdPASS) {
		printf("Failed to create test led task 3\r\n");
	}
	if 	(xTaskCreate(task_UartRx, "UartRx", TASK_LED1_STACK_SIZE, NULL, TASK_LED1_STACK_PRIORITY, NULL)
	!= pdPASS) {
		printf("Failed to create test led task 3\r\n");
	}
	if 	(xTaskCreate(task_UartRx, "UartRx", TASK_LED1_STACK_SIZE, NULL, TASK_LED1_STACK_PRIORITY, NULL)
	!= pdPASS) {
		printf("Failed to create test led task 3\r\n");
	}
	if 	(xTaskCreate(task_executa, "Executa", TASK_LED1_STACK_SIZE, NULL, TASK_LED1_STACK_PRIORITY, NULL)
	!= pdPASS) {
		printf("Failed to create test led task 3\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
