#ifndef MINIPID_H
#define MINIPID_H

#include "stdbool.h"

int MiniPID(float p, float i, float d);
int MiniPIDF(float p, float i, float d, float f);
void pid_setP(float p);
void pid_setI(float i);
void pid_setD(float d);
void pid_setF(float f);
void pid_setPID(float p, float i, float d);
void pid_setPIDF(float p, float i, float d, float f);
void pid_setMaxIOutput(float maximum);
void pid_setOutputLimits(float minimum, float maximum);
void pid_setDirection(bool rev);
void pid_setSetpoint(float set);
void pid_reset();
void pid_setOutputRampRate(float rate);
void pid_setSetpointRange(float range);
void pid_setOutputFilter(float strength);
float pid_getOutput(float actual, float setpoint);
static float pid_clamp(float value, float min, float max);
static bool pid_bounded(float value, float min, float max);
static void pid_init();


void pid_checkSigns();

float P;
float I;
float D;
float F;

float maxIOutput;
float maxError;
float errorSum;

float maxOutput;
float minOutput;

float setpoint;

float lastActual;

bool firstRun;
bool reversed;

float outputRampRate;
float lastOutput;

float outputFilter;

float setpointRange;



#endif