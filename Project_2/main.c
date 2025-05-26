/* Standard includes. */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "stm32f4_discovery.h"
/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"



enum task_type {PERIODIC, APERIODIC};

typedef struct dd_task dd_task;
struct dd_task {
	TaskHandle_t t_handle;
	enum task_type type;
	uint32_t task_id;
	uint32_t release_time;
	uint32_t absolute_deadline;
	uint32_t completion_time;
	uint32_t period;
};

typedef struct dd_task_list dd_task_list;
struct dd_task_list {
	dd_task task;
	dd_task_list *next_task;
};

typedef enum msg_type msg_type;
enum msg_type {ACTIVE, COMPLETED, OVERDUE, RELEASE, DELETE};

typedef struct dds_msg dds_msg;
struct dds_msg {
	msg_type msg;
	dd_task task;
};

/*-----------------------------------------------------------*/
#define mainQUEUE_LENGTH 100

TaskHandle_t handle1;
TaskHandle_t handle2;
TaskHandle_t handle3;
TaskHandle_t monitor_handle;
TaskHandle_t dds_handle;

// ------------- BENCH 1 -----------
//#define T1_PERIOD pdMS_TO_TICKS(500)
//#define T2_PERIOD pdMS_TO_TICKS(500)
//#define T3_PERIOD pdMS_TO_TICKS(750)
//
//#define T1_EXECUTION pdMS_TO_TICKS(95)
//#define T2_EXECUTION pdMS_TO_TICKS(150)
//#define T3_EXECUTION pdMS_TO_TICKS(250)

// ------------- BENCH 2 -----------
// #define T1_PERIOD pdMS_TO_TICKS(250)
// #define T2_PERIOD pdMS_TO_TICKS(500)
// #define T3_PERIOD pdMS_TO_TICKS(750)
//
// #define T1_EXECUTION pdMS_TO_TICKS(95)
// #define T2_EXECUTION pdMS_TO_TICKS(150)
// #define T3_EXECUTION pdMS_TO_TICKS(250)

// ------------- BENCH 3 -----------
 #define T1_PERIOD pdMS_TO_TICKS(500)
 #define T2_PERIOD pdMS_TO_TICKS(500)
 #define T3_PERIOD pdMS_TO_TICKS(500)

 #define T1_EXECUTION pdMS_TO_TICKS(100)
 #define T2_EXECUTION pdMS_TO_TICKS(200)
 #define T3_EXECUTION pdMS_TO_TICKS(200)


/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );

static void dd_task_generator(void *pvParameter);
static void deadline_driven_scheduler(void *pvParameter);
static void monitor(void *pvParameter);
static void user_task_1(void *pvParameter);
static void user_task_2(void *pvParameter);
static void user_task_3(void *pvParameter);

static void task1Callback(TimerHandle_t xTimer);
static void task2Callback(TimerHandle_t xTimer);
static void task3Callback(TimerHandle_t xTimer);

void create_dd_task(TaskHandle_t t_handle, enum task_type type, uint32_t task_id, uint32_t absolute_deadline);
void delete_dd_task(uint32_t task_id);
dd_task_list** get_active_dd_task_list(void);
dd_task_list** get_completed_dd_task_list(void);
dd_task_list** get_overdue_dd_task_list(void);

xQueueHandle xQueue_dds_msg = 0;
xQueueHandle xQueue_list = 0;


/*-----------------------------------------------------------*/

int main(void)
{
	prvSetupHardware();

	xQueue_dds_msg = xQueueCreate(mainQUEUE_LENGTH, sizeof( dds_msg ) );
	xQueue_list = xQueueCreate(1, sizeof( dd_task_list** ) );

	xTaskCreate(deadline_driven_scheduler, "DD_Scheduler", configMINIMAL_STACK_SIZE, NULL, 3, &dds_handle);

	xTaskCreate(dd_task_generator, "DD_Generator", configMINIMAL_STACK_SIZE, NULL, 4, NULL);

	xTaskCreate(monitor, "Monitor", configMINIMAL_STACK_SIZE, NULL, 3, &monitor_handle);

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	return 0;
}

static void user_task_1(void *pvParameter) {

	int tick_counter = 0;
	TickType_t current_tick = xTaskGetTickCount();
	TickType_t previous_tick;

    // Finish time should be time of exeuction from current time
	int finish_time = current_tick + T1_EXECUTION;

	while(current_tick < finish_time) {
		previous_tick = current_tick;
		current_tick = xTaskGetTickCount();

        // Check if the ticks are changing
        // Another task might start running causing this task to freeze
		if(previous_tick != current_tick)
			tick_counter++;

		finish_time = current_tick + T1_EXECUTION - tick_counter;
	}

	delete_dd_task(1);
}

static void user_task_2(void *pvParameter) {

	int tick_counter = 0;
	TickType_t current_tick = xTaskGetTickCount();
	TickType_t previous_tick;
	int finish_time = current_tick + T2_EXECUTION;

	while(current_tick < finish_time) {
		previous_tick = current_tick;
		current_tick = xTaskGetTickCount();

		if(previous_tick != current_tick)
			tick_counter++;

		finish_time = current_tick + T2_EXECUTION - tick_counter;
	}

	delete_dd_task(2);
}

static void user_task_3(void *pvParameter) {

	int tick_counter = 0;
	TickType_t current_tick = xTaskGetTickCount();
	TickType_t previous_tick;
	int finish_time = current_tick + T3_EXECUTION;

	while(current_tick < finish_time) {
		previous_tick = current_tick;
		current_tick = xTaskGetTickCount();

		if(previous_tick != current_tick)
			tick_counter++;

		finish_time = current_tick + T3_EXECUTION - tick_counter;
	}

	delete_dd_task(3);
}


static void task1Callback(xTimerHandle xTimer) {
    // Create a new task each time and suspend immediately so DDS can run it later
	xTaskCreate(user_task_1, "User_1", configMINIMAL_STACK_SIZE, NULL, 1, &handle1);
	vTaskSuspend(handle1);

    // Release the task from create_dd_task
	create_dd_task(handle1, PERIODIC, 1, xTaskGetTickCount() + T1_PERIOD);
}

static void task2Callback(xTimerHandle xTimer) {
	xTaskCreate(user_task_2, "User_2", configMINIMAL_STACK_SIZE, NULL, 1, &handle2);
	vTaskSuspend(handle2);

	create_dd_task(handle2, PERIODIC, 2, xTaskGetTickCount() + T2_PERIOD);
}

static void task3Callback(xTimerHandle xTimer) {
	xTaskCreate(user_task_3, "User_3", configMINIMAL_STACK_SIZE, NULL, 1, &handle3);
	vTaskSuspend(handle3);

	create_dd_task(handle3, PERIODIC, 3, xTaskGetTickCount() + T3_PERIOD);
}


static void dd_task_generator(void *pvParameter) {
	// Initial tasks to be released immediately
	task1Callback(NULL);
	task2Callback(NULL);
	task3Callback(NULL);

	// Use timers for all subsequent releases
    // Timers are run periodically using bench periods
	TimerHandle_t task_1_timer = xTimerCreate("UserTimer1", T1_PERIOD, pdTRUE, (void *) 1, task1Callback);
	TimerHandle_t task_2_timer = xTimerCreate("UserTimer2", T2_PERIOD, pdTRUE, (void *) 2, task2Callback);
	TimerHandle_t task_3_timer = xTimerCreate("UserTimer3", T3_PERIOD, pdTRUE, (void *) 3, task3Callback);

	while(1) {
		xTimerStart(task_1_timer, 0);
		xTimerStart(task_2_timer, 0);
		xTimerStart(task_3_timer, 0);

		vTaskSuspend(NULL);
	}
}

static void monitor(void *pvParameter) {

	while(1) {

        // Get all list and delay monitor task

		get_active_dd_task_list();
		get_completed_dd_task_list();
		get_overdue_dd_task_list();

		vTaskDelay(1500 / portTICK_PERIOD_MS);
	}
}

dd_task createEmptyTask() {
	dd_task emptyTask;

    // Empty task for inital setup
	emptyTask.t_handle = NULL;
	emptyTask.type = PERIODIC;
	emptyTask.task_id = 0;
	emptyTask.absolute_deadline = 0;
	emptyTask.completion_time = 0;
	emptyTask.release_time = 0;

	return emptyTask;
}

static void deadline_driven_scheduler(void *pvParameter) {
	dd_task new_task;
	dd_task delete_task;
	dd_task overdue_task;
	dds_msg new_msg;
	uint32_t event_id = 0;

	dd_task_list* active_list = malloc(sizeof(dd_task_list));
	dd_task_list* completed_list = malloc(sizeof(dd_task_list));
	dd_task_list* overdue_list = malloc(sizeof(dd_task_list));

	active_list->task = createEmptyTask();
	completed_list->task = createEmptyTask();
	overdue_list->task = createEmptyTask();

	active_list->next_task = NULL;
	completed_list->next_task = NULL;
	overdue_list->next_task = NULL;

	while(1) {
		if(xQueueReceive(xQueue_dds_msg, &new_msg, portMAX_DELAY)) {
			dd_task_list* current;
			dd_task_list* previous;

			if(new_msg.msg == RELEASE) {
				new_task = new_msg.task;

				printf("\n%d \t\t Task %d released \t\t %d", ++event_id, new_msg.task.task_id, xTaskGetTickCount());

				current = active_list;
				previous = active_list;

				dd_task_list* temp = malloc(sizeof(dd_task_list));
				temp->task = new_task;
				temp->next_task = NULL;

				// if empty
				if(current->task.absolute_deadline == 0) {
					active_list->task = new_task;
				}

				else {
					// if new task's deadline is after the current's deadline
					while (current != NULL && new_task.absolute_deadline >= current->task.absolute_deadline) {
						// traverse through the list
						previous = current;
						current = current->next_task;
					}

					// reached the end of the list
					if(current == NULL)
						previous->next_task = temp;

					// add between two tasks
					else {
						temp->next_task = current;

						// only one task in the list
						if(previous == current)
							active_list = temp;

						else
							previous->next_task = temp;

						// if the task had been released, delete the task and create a new one
						// this will cause the task to suspend execution
						if(previous->task.release_time > 0) {
							vTaskSuspend(previous->task.t_handle);
							previous->task.release_time = 0;
						}
					}
				}
			}

			else if (new_msg.msg == DELETE) {
				delete_task = new_msg.task;

				printf("\n%d \t\t Task %d completed \t\t %d", ++event_id, new_msg.task.task_id, xTaskGetTickCount());

				// Check if the task to be deleted is the one in the active list
				if(delete_task.task_id == active_list->task.task_id) {
                    // Delete user task if it's in the active list
					if (active_list->task.t_handle != NULL) {
						vTaskDelete(active_list->task.t_handle);
						active_list->task.t_handle = NULL;
					}

                    // get the task from active list and assign completion time
					delete_task = active_list->task;
					delete_task.completion_time = xTaskGetTickCount();

                    // Update active list to point to the next task
					if(active_list->next_task != NULL) {
						active_list->task = active_list->next_task->task;
						active_list->next_task = active_list->next_task->next_task;
					}
                    // If the active list becomes empty
					else {
						active_list->task = createEmptyTask();
						active_list->next_task = NULL;
					}

					current = completed_list;
					previous = completed_list;

					// If the completed list is empty, insert the completed task
					if(current->task.absolute_deadline == 0) {
						completed_list->task = delete_task;
						completed_list->next_task = NULL;
					}

					// Otherwise, insert the completed task in sorted order
					else {
						dd_task_list* temp = malloc(sizeof(dd_task_list));
						temp->task = delete_task;
						temp->next_task = NULL;

						while (current != NULL) {
							// Traverse through the list and schedule between tasks
							previous = current;
							current = current->next_task;
						}

						previous->next_task = temp;
					}
				}
			}

            // Send the respective list to the queue
			else if (new_msg.msg == ACTIVE) {
				xQueueSend(xQueue_list, &active_list, portMAX_DELAY);
			}

			else if (new_msg.msg == COMPLETED) {
				xQueueSend(xQueue_list, &completed_list, portMAX_DELAY);
			}

			else if (new_msg.msg == OVERDUE) {
				xQueueSend(xQueue_list, &overdue_list, portMAX_DELAY);
			}

			// Handle overdue tasks
			if(active_list->task.absolute_deadline > 0 && xTaskGetTickCount() > (active_list->task.absolute_deadline)) {
				overdue_task = active_list->task;

                // Delete overdue task
				if (overdue_task.t_handle != NULL) {
					vTaskDelete(overdue_task.t_handle);
					overdue_task.t_handle = NULL;  // Nullify the task handle to prevent accidental re-deletion
				}

				current = overdue_list;
				previous = overdue_list;

				// If the overdue list is empty, insert the overdue task
				if(current->task.absolute_deadline == 0) {
					overdue_list->task = overdue_task;
					overdue_list->next_task = NULL;
				}

				// Otherwise, insert the overdue task in sorted order
				else {
					dd_task_list* temp = malloc(sizeof(dd_task_list));
					temp->task = overdue_task;
					temp->next_task = NULL;

					while (current != NULL) {
						// Traverse through the list until the end
						previous = current;
						current = current->next_task;
					}

					previous->next_task = temp;
				}

                // Update active list to point to the next task
				if(active_list->next_task != NULL) {
					active_list->task = active_list->next_task->task;
					active_list->next_task = active_list->next_task->next_task;
				}
				else {
					active_list->task = createEmptyTask();
					active_list->next_task = NULL;
				}
			}

            // Wait until at least three tasks are available
            // If it hasn't been released and the task is not empty, then run it
			if(event_id >= 3 && active_list->task.absolute_deadline > 0 && active_list->task.release_time == 0) {

                // Set release time to now. Give priority and resume task execution
				active_list->task.release_time = xTaskGetTickCount();
				vTaskPrioritySet(active_list->task.t_handle, 3);
				vTaskResume(active_list->task.t_handle);
			}
		}
	}
}


// create new task and send RELEASE msg to DDS
void create_dd_task(TaskHandle_t t_handle, enum task_type type, uint32_t task_id, uint32_t absolute_deadline) {
	 dd_task new_task = {
		.t_handle = t_handle,
		.type = type,
		.task_id = task_id,
		.absolute_deadline = absolute_deadline
	};

	dds_msg  new_msg = {
			.msg = RELEASE,
			.task = new_task
	};

	xQueueSend(xQueue_dds_msg, &new_msg, portMAX_DELAY);
}

// get deletion task's id and send DELETE msg to DDS
void delete_dd_task(uint32_t task_id) {
	 dd_task delete_task = createEmptyTask();
	 delete_task.task_id = task_id;

	dds_msg  new_msg = {
			.msg = DELETE,
			.task = delete_task
	};

	xQueueSend(xQueue_dds_msg, &new_msg, portMAX_DELAY);
	vTaskSuspend(NULL);
}

dd_task_list** get_active_dd_task_list() {
	dds_msg msg = { .msg = ACTIVE };
	dd_task_list* active_list;

    // Send msg to receive list
	xQueueSend(xQueue_dds_msg, &msg, portMAX_DELAY);
	xQueueReceive(xQueue_list, &active_list, portMAX_DELAY);

	int total_active_tasks = 0;
	dd_task_list* active_task = active_list;

    // traverse through the list until the end
	while(active_task != NULL) {
		if(active_task->task.absolute_deadline > 0)
			total_active_tasks++;

		active_task = active_task->next_task;
	}

	printf("\nTotal Active Tasks: %d", total_active_tasks);
}

dd_task_list** get_completed_dd_task_list() {
	dds_msg msg = { .msg = COMPLETED };
	dd_task_list* completed_list;

	xQueueSend(xQueue_dds_msg, &msg, portMAX_DELAY);
	xQueueReceive(xQueue_list, &completed_list, portMAX_DELAY);

	int total_completed_tasks = 0;
	dd_task_list* completed_task = completed_list;

	while(completed_task != NULL) {
		if(completed_task->task.absolute_deadline > 0)
			total_completed_tasks++;

		completed_task = completed_task->next_task;
	}

	printf("\nTotal Completed Tasks: %d", total_completed_tasks);

}

dd_task_list** get_overdue_dd_task_list() {
	dds_msg msg = { .msg = OVERDUE };
	dd_task_list* overdue_list;

	xQueueSend(xQueue_dds_msg, &msg, portMAX_DELAY);
	xQueueReceive(xQueue_list, &overdue_list, portMAX_DELAY);

	int total_overdue_tasks = 0;
	dd_task_list* overdue_task = overdue_list;

	while(overdue_task != NULL) {
		if(overdue_task->task.absolute_deadline > 0)
			total_overdue_tasks++;

		overdue_task = overdue_task->next_task;
	}

	printf("\nTotal Overdue Tasks: %d", total_overdue_tasks);
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
}
