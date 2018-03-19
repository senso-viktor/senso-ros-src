#include <stdio.h>
#include <stdlib.h>
#include "torque_sensor.h"
#include "torque_sensor_private.h"
#include "rtwtypes.h"
#include "limits.h"
#include "linuxinitialize.h"
#define UNUSED(x)                      x = x

/* Function prototype declaration*/
void exitFcn(int sig);
void *terminateTask(void *arg);
void *baseRateTask(void *arg);
void *subrateTask(void *arg);
volatile boolean_T runModel = true;
sem_t stopSem;
sem_t baserateTaskSem;
pthread_t schedulerThread;
pthread_t baseRateThread;
unsigned long threadJoinStatus[8];
int terminatingmodel = 0;
void *baseRateTask(void *arg)
{
  runModel = (rtmGetErrorStatus(torque_sensor_M) == (NULL));
  while (runModel) {
    sem_wait(&baserateTaskSem);
    torque_sensor_step();

    /* Get model outputs here */
    runModel = (rtmGetErrorStatus(torque_sensor_M) == (NULL));
  }

  runModel = 0;
  terminateTask(arg);
  pthread_exit((void *)0);
  return NULL;
}

void exitFcn(int sig)
{
  UNUSED(sig);
  rtmSetErrorStatus(torque_sensor_M, "stopping the model");
}

void *terminateTask(void *arg)
{
  UNUSED(arg);
  terminatingmodel = 1;

  {
    runModel = 0;
  }

  /* Disable rt_OneStep() here */

  /* Terminate model */
  torque_sensor_terminate();
  sem_post(&stopSem);
  return NULL;
}

int main(int argc, char **argv)
{
  UNUSED(argc);
  UNUSED(argv);
  void slros_node_init(int argc, char** argv);
  slros_node_init(argc, argv);
  rtmSetErrorStatus(torque_sensor_M, 0);

  /* Initialize model */
  torque_sensor_initialize();

  /* Call RTOS Initialization function */
  myRTOSInit(0.005, 0);

  /* Wait for stop semaphore */
  sem_wait(&stopSem);
  return 0;
}
