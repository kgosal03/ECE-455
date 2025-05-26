/*
    FreeRTOS-based traffic control system simulation for STM32F4 Discovery board.
    This application manages traffic flow, traffic light states, and displays the system state.
*/

/* Standard includes */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "stm32f4_discovery.h"

/* FreeRTOS includes */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

/* Queue length definition */
#define mainQUEUE_LENGTH 100

/* Function prototypes */
static void prvSetupHardware(void);  // Hardware setup function
static void Traffic_Flow_Adjustment(void *pvParameters);  // Task to adjust traffic flow based on ADC readings
static void Traffic_Generator(void *pvParameters);  // Task to generate traffic based on flow adjustment
static void Traffic_Light_State(void *pvParameters);  // Task to manage traffic light states
static void System_Display(void *pvParameters);  // Task to display system state (lights and traffic)

/* GPIO and ADC initialization functions */
void GPIOC_ADC_Init(void);  // Initialize GPIO for ADC
void GPIOC_SPC_Init(void);  // Initialize GPIO for traffic simulation
void GPIOC_Lights_Init(void);  // Initialize GPIO for traffic lights
void myADC_Init(void);  // Initialize ADC for potentiometer reading

/* Global queues for inter-task communication */
xQueueHandle xQueue_pot_value = 0;  // Queue for potentiometer values
xQueueHandle xQueue_light = 0;  // Queue for traffic light states
xQueueHandle xQueue_generate = 0;  // Queue for traffic generation signals

/*-----------------------------------------------------------*/

int main(void)
{
    prvSetupHardware();  // Initialize hardware

    // Enable clocks for GPIOC and ADC1
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // Initialize GPIO and ADC
    GPIOC_SPC_Init();  // Initialize GPIO for traffic simulation
    GPIOC_Lights_Init();  // Initialize GPIO for traffic lights
    GPIOC_ADC_Init();  // Initialize GPIO for ADC
    myADC_Init();  // Initialize ADC for potentiometer reading

    // Create queues for inter-task communication
    xQueue_pot_value = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));  // Queue for potentiometer values
    xQueue_light = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint16_t));  // Queue for traffic light states
    xQueue_generate = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint16_t));  // Queue for traffic generation signals

    // Add the main queue to the registry for debugging
    vQueueAddToRegistry(xQueue_pot_value, "MainQueue");

    // Create tasks
    xTaskCreate(Traffic_Flow_Adjustment, "Traffic_Flow_Adjustment", configMINIMAL_STACK_SIZE, NULL, 1, NULL);  // Task to adjust traffic flow
    xTaskCreate(Traffic_Generator, "Traffic_Generator", configMINIMAL_STACK_SIZE, NULL, 1, NULL);  // Task to generate traffic
    xTaskCreate(Traffic_Light_State, "Traffic_Light_State", configMINIMAL_STACK_SIZE, NULL, 1, NULL);  // Task to manage traffic lights
    xTaskCreate(System_Display, "System_Display", configMINIMAL_STACK_SIZE, NULL, 1, NULL);  // Task to display system state

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    return 0;
}

/*-----------------------------------------------------------*/

// Initialize GPIO for ADC (potentiometer input)
void GPIOC_ADC_Init() {
    GPIO_InitTypeDef GPIO_InitStruct_ADC;

    GPIO_InitStruct_ADC.GPIO_Pin = GPIO_Pin_3;  // Use pin 3 for ADC
    GPIO_InitStruct_ADC.GPIO_Mode = GPIO_Mode_AN;  // Set pin to analog mode
    GPIO_InitStruct_ADC.GPIO_Speed = GPIO_Speed_50MHz;  // Set speed
    GPIO_InitStruct_ADC.GPIO_OType = GPIO_OType_PP;  // Set output type to push-pull
    GPIO_InitStruct_ADC.GPIO_PuPd = GPIO_PuPd_NOPULL;  // No pull-up/pull-down
    GPIO_Init(GPIOC, &GPIO_InitStruct_ADC);  // Initialize GPIO
}

// Initialize ADC for potentiometer reading
void myADC_Init() {
    ADC_InitTypeDef ADC_InitStruct1;

    ADC_InitStruct1.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;  // No external trigger
    ADC_InitStruct1.ADC_DataAlign = ADC_DataAlign_Right;  // Right-align data
    ADC_InitStruct1.ADC_Resolution = ADC_Resolution_12b;  // 12-bit resolution
    ADC_InitStruct1.ADC_ExternalTrigConv = DISABLE;  // Disable external trigger
    ADC_InitStruct1.ADC_ScanConvMode = DISABLE;  // Disable scan mode
    ADC_InitStruct1.ADC_ContinuousConvMode = ENABLE;  // Enable continuous conversion
    ADC_InitStruct1.ADC_NbrOfConversion = 1;  // Single conversion

    ADC_Init(ADC1, &ADC_InitStruct1);  // Initialize ADC
    ADC_Cmd(ADC1, ENABLE);  // Enable ADC
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_84Cycles);  // Configure ADC channel
}

// Initialize GPIO for traffic simulation (SPC: Simulation of Traffic)
void GPIOC_SPC_Init() {
    GPIO_InitTypeDef GPIO_InitStruct_SPC;

    GPIO_InitStruct_SPC.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;  // Use pins 6, 7, and 8
    GPIO_InitStruct_SPC.GPIO_Mode = GPIO_Mode_OUT;  // Set pins to output mode
    GPIO_InitStruct_SPC.GPIO_Speed = GPIO_Speed_25MHz;  // Set speed
    GPIO_InitStruct_SPC.GPIO_OType = GPIO_OType_PP;  // Set output type to push-pull
    GPIO_InitStruct_SPC.GPIO_PuPd = GPIO_PuPd_NOPULL;  // No pull-up/pull-down
    GPIO_Init(GPIOC, &GPIO_InitStruct_SPC);  // Initialize GPIO
}

// Initialize GPIO for traffic lights
void GPIOC_Lights_Init() {
    GPIO_InitTypeDef GPIO_InitStruct_Lights;

    GPIO_InitStruct_Lights.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;  // Use pins 0, 1, and 2
    GPIO_InitStruct_Lights.GPIO_Mode = GPIO_Mode_OUT;  // Set pins to output mode
    GPIO_InitStruct_Lights.GPIO_Speed = GPIO_Speed_25MHz;  // Set speed
    GPIO_InitStruct_Lights.GPIO_OType = GPIO_OType_PP;  // Set output type to push-pull
    GPIO_InitStruct_Lights.GPIO_PuPd = GPIO_PuPd_NOPULL;  // No pull-up/pull-down
    GPIO_Init(GPIOC, &GPIO_InitStruct_Lights);  // Initialize GPIO
}

/*-----------------------------------------------------------*/

// Task to adjust traffic flow based on ADC readings
static void Traffic_Flow_Adjustment(void *pvParameters) {
    uint32_t adc_value, pot_value;
    while (1) {
        ADC_SoftwareStartConv(ADC1);  // Start ADC conversion
        while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0);  // Wait for conversion to complete
        adc_value = ADC_GetConversionValue(ADC1);  // Get ADC value

        // Cap ADC value to avoid overflow
        if (adc_value > 4000)
            adc_value = 4000;

        // Convert ADC value to a range of 0-100
        pot_value = (uint32_t)((adc_value - 50) * 100) / 3950;

        // Send the potentiometer value to the queue
        if (xQueueSend(xQueue_pot_value, &pot_value, 1000)) {
            printf("POT:%d\n", pot_value);  // Print potentiometer value
            vTaskDelay(1000);  // Delay for 1 second
        } else {
            printf("Traffic Flow Adjustment Failed!\n");  // Print error if queue send fails
        }
    }
}

// Task to generate traffic based on flow adjustment
static void Traffic_Generator(void *pvParameters) {
    uint32_t pot_value;
    uint16_t generate = 1;  // Signal to generate traffic
    int max, min, delay;

    while (1) {
        if (xQueueReceive(xQueue_pot_value, &pot_value, 500)) {  // Receive potentiometer value
            xQueueSend(xQueue_generate, &generate, 1000);  // Send traffic generation signal

            // Set delay range based on potentiometer value
            if (pot_value >= 0 && pot_value < 26) {
                max = 6000;
                min = 4500;
            } else if (pot_value > 25 && pot_value < 51) {
                max = 4500;
                min = 3000;
            } else if (pot_value > 50 && pot_value < 76) {
                max = 3000;
                min = 1500;
            } else {
                max = 1500;
                min = 500;
            }

            // Generate a random delay within the range
            delay = rand() % (max - min + 1) + min;
            printf("Delay:%d\n", delay);  // Print the delay
            vTaskDelay(delay);  // Delay for the generated time
        }
    }
}

// Timer callback for traffic light control
static void trafficTimerCallback(xTimerHandle xTimer) {
    printf("Traffic timer callback\n");  // Print timer callback message
    xTimerStop(xTimer, 0);  // Stop the timer
}

// Task to manage traffic light states
static void Traffic_Light_State(void *pvParameters) {
    uint32_t pot_value, green_duration, red_duration, yellow_duration = 3;  // Light durations
    TimerHandle_t xTraffic = xTimerCreate("TrafficLight", 5000 / portTICK_PERIOD_MS, pdFALSE, (void *)0, trafficTimerCallback);  // Create traffic light timer
    uint16_t light_on = 0;  // Current light state

    while (1) {
        if (xQueueReceive(xQueue_pot_value, &pot_value, 500)) {  // Receive potentiometer value
            // Set light durations based on potentiometer value
            if (pot_value >= 0 && pot_value < 26) {
                green_duration = 3;
                red_duration = 6;
            } else if (pot_value > 25 && pot_value < 51) {
                green_duration = 4;
                red_duration = 5;
            } else if (pot_value > 50 && pot_value < 76) {
                green_duration = 5;
                red_duration = 4;
            } else {
                green_duration = 6;
                red_duration = 3;
            }

            // Change traffic light state and update timer
            if (!xTimerIsTimerActive(xTraffic)) {
                xQueueSend(xQueue_light, &light_on, 500);  // Send current light state to queue

                if (light_on == 0) {  // Red light on
                    GPIO_ResetBits(GPIOC, GPIO_Pin_1);  // Turn off yellow
                    GPIO_ResetBits(GPIOC, GPIO_Pin_2);  // Turn off green
                    GPIO_SetBits(GPIOC, GPIO_Pin_0);  // Turn on red

                    xTimerChangePeriod(xTraffic, (red_duration * 1000) / portTICK_PERIOD_MS, 0);  // Set timer for red duration
                    xTimerStart(xTraffic, 0);  // Start timer
                    light_on = 2;  // Next state: green
                } else if (light_on == 1) {  // Yellow light on
                    GPIO_ResetBits(GPIOC, GPIO_Pin_0);  // Turn off red
                    GPIO_ResetBits(GPIOC, GPIO_Pin_2);  // Turn off green
                    GPIO_SetBits(GPIOC, GPIO_Pin_1);  // Turn on yellow

                    xTimerChangePeriod(xTraffic, (yellow_duration * 1000) / portTICK_PERIOD_MS, 0);  // Set timer for yellow duration
                    xTimerStart(xTraffic, 0);  // Start timer
                    light_on = 0;  // Next state: red
                } else if (light_on == 2) {  // Green light on
                    GPIO_ResetBits(GPIOC, GPIO_Pin_0);  // Turn off red
                    GPIO_ResetBits(GPIOC, GPIO_Pin_1);  // Turn off yellow
                    GPIO_SetBits(GPIOC, GPIO_Pin_2);  // Turn on green

                    xTimerChangePeriod(xTraffic, (green_duration * 1000) / portTICK_PERIOD_MS, 0);  // Set timer for green duration
                    xTimerStart(xTraffic, 0);  // Start timer
                    light_on = 1;  // Next state: yellow
                }
            }
        } else {
            if (xQueueSend(xQueue_pot_value, &pot_value, 1000)) {  // Retry sending potentiometer value
                vTaskDelay(500);  // Delay for 500ms
            }
        }
    }
}

// Task to display system state (lights and traffic)
static void System_Display(void *pvParameters) {
    uint16_t light, new_car = 0;  // Light state and new car signal
    int cars[20] = {0};  // Array to simulate traffic

    GPIO_SetBits(GPIOC, GPIO_Pin_8);  // Turn on display

    while (1) {
        if (xQueueReceive(xQueue_light, &light, 500)) {  // Receive light state
            printf("Light On: %d\n", light);  // Print light state
        }

        // Reset and update traffic display
        for (int i = 20; i > 0; i--) {
            if (cars[i])
                GPIO_SetBits(GPIOC, GPIO_Pin_6);  // Turn on LED if car is present
            else
                GPIO_ResetBits(GPIOC, GPIO_Pin_6);  // Turn off LED if no car

            GPIO_SetBits(GPIOC, GPIO_Pin_7);  // Clock signal for display
            GPIO_ResetBits(GPIOC, GPIO_Pin_7);
        }

        // Move cars forward based on light state
        if (light == 2) {  // Green light
            for (int i = 19; i > 0; i--)
                cars[i] = cars[i - 1];  // Move cars forward
        } else {  // Red or yellow light
            for (int i = 8; i > 0; i--) {
                if (cars[i] == 0) {
                    cars[i] = cars[i - 1];  // Move cars forward if space is available
                    cars[i - 1] = 0;
                }
            }
            for (int i = 19; i > 9; i--) {
                cars[i] = cars[i - 1];  // Move cars forward in the second half
                cars[i - 1] = 0;
            }
        }

        cars[0] = 0;  // Clear the first position
        if (xQueueReceive(xQueue_generate, &new_car, 100)) {  // Receive new car signal
            if (new_car) {
                cars[0] = 1;  // Add a new car to the first position
            }
        }

        vTaskDelay(500);  // Delay for 500ms
    }
}

/*-----------------------------------------------------------*/

// Hook function called when malloc fails
void vApplicationMallocFailedHook(void) {
    for (;;);  // Infinite loop if malloc fails
}

// Hook function called when a stack overflow is detected
void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName) {
    for (;;);  // Infinite loop if stack overflow occurs
}

// Hook function called during each idle task cycle
void vApplicationIdleHook(void) {
    volatile size_t xFreeStackSpace = xPortGetFreeHeapSize();  // Get free heap space
    if (xFreeStackSpace > 100) {
        // If there is a lot of free heap, configTOTAL_HEAP_SIZE can be reduced
    }
}

/*-----------------------------------------------------------*/

// Hardware setup function
static void prvSetupHardware(void) {
    NVIC_SetPriorityGrouping(0);  // Set priority grouping for NVIC
    // Additional hardware setup can be added here if needed
}