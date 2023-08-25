#include "MiniPID.h"

int MiniPID(float p,float i, float d)
{
    pid_init();
    P=p; I=i; D=d;
    return 0;
}

int MiniPIDF(float p,float i, float d,float f)
{
    pid_init();
    P=p; I=i; D=d; F=f;
    return 0;
}

static void pid_init()
{
    P=0;
	I=0;
	D=0;
	F=0;

	maxIOutput=0;
	maxError=0;
	errorSum=0;
	maxOutput=0; 
	minOutput=0;
	setpoint=0;
	lastActual=0;
	firstRun=true;
	reversed=false;
	outputRampRate=0;
	lastOutput=0;
	outputFilter=0;
	setpointRange=0;

}

void pid_setP(float p)
{
    P=p;
    pid_checkSigns();
}

void pid_setI(float i)
{
    if(I!=0)
    {
        errorSum=errorSum*I/i;
    }
    if(maxIOutput!=0)
    {
        maxError=maxIOutput/i;
    }
    I=i;
    pid_checkSigns();
}

void pid_setD(float d)
{
    D=d;
    pid_checkSigns();
}

void pid_setF(float f)
{
    F=f;
    pid_checkSigns();
}

void pid_setPID(float p,float i,float d)
{
    P=p;
    I=i;
    D=d;
    pid_checkSigns();
}

void pid_setPIDF(float p,float i,float d,float f)
{
    P=p;
    I=i;
    D=d;
    F=f;
    pid_checkSigns();
}

void pid_setMaxIOutput(float maximum)
{
    maxIOutput=maximum;
    if(I!=0)
    {
        maxError=maxIOutput/I;
    }
}

void pid_setOutputLimits(float minimum,float maximum)
{
    if(minimum>maximum)return;
    minOutput=minimum;
    maxOutput=maximum;

    if(maxIOutput==0||maxIOutput>(maximum-minimum))
    {
        pid_setMaxIOutput(maximum-minimum);
    }
}

void pid_setDirection(bool rev)
{
    reversed=rev;
}

void pid_setSetpoint(float set)
{
    setpoint=set;
}

float pid_getOutput(float actual,float set)
{
    float output;
    float Poutput;
    float Ioutput;
    float Doutput;
    float Foutput;

    setpoint=set;

    if(setpointRange!=0)
    {
        setpoint=pid_clamp(setpoint,actual-setpointRange,actual+setpointRange);
    }

    float error=setpoint-actual;

    Foutput=F*setpoint;

    Poutput=P*error;

    if(firstRun)
    {
        lastActual=actual;
        lastOutput=Poutput+Foutput;
        firstRun=false;
    }

    Doutput=-D*(actual-lastActual);
    lastActual=actual;

    Ioutput=I*errorSum;
    if(maxIOutput!=0)
    {
        Ioutput=pid_clamp(Ioutput,-maxIOutput,maxIOutput);
    }
    
    output=Poutput+Ioutput+Doutput+Foutput;

    if(minOutput!=maxOutput&&!pid_bounded(output, minOutput,maxOutput))
    {
        
    }
    else if(outputRampRate!=0&&pid_bounded(output, lastOutput-outputRampRate,lastOutput+outputRampRate))
    {
        errorSum=error;
    }
    else if(maxIOutput!=0)
    {
        errorSum=pid_clamp(errorSum+error,-maxError,maxError);
    }
    else
    {
        errorSum+=error;
    }

    if(outputRampRate!=0)
    {
        output=pid_clamp(output, lastOutput-outputRampRate,lastOutput+outputRampRate);
    }
    if(minOutput!=maxOutput)
    {
        output=pid_clamp(output, minOutput,maxOutput);
    }
    if(outputFilter!=0)
    {
        output=lastOutput*outputFilter+output*(1-outputFilter);
    }

    lastOutput=output;
    return output;

}

void pid_reset()
{
    firstRun=true;
    errorSum=0;
}

void pid_setOutputRampRate(float rate)
{
    outputRampRate=rate;
}

void pid_setSetpointRange(float range)
{
    setpointRange=range;
}

void pid_setOutputFilter(float strength)
{
    if(strength==0||pid_bounded(strength, 0,1))
    {
        outputFilter=strength;
    }
}

static float pid_clamp(float value,float min,float max)
{
    if(value>max)return max;
    if(value<min)return min;
    return value;
}

static bool pid_bounded(float value,float min,float max)
{
    return (min<value&&value<max);
}

void pid_checkSigns()
{
    if(reversed){	//all values should be below zero
		if(P>0) P*=-1;
		if(I>0) I*=-1;
		if(D>0) D*=-1;
		if(F>0) F*=-1;
	}
	else{	//all values should be above zero
		if(P<0) P*=-1;
		if(I<0) I*=-1;
		if(D<0) D*=-1;
		if(F<0) F*=-1;
	}
}