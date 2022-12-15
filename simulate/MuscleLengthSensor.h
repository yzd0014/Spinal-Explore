#pragma once
#include <mujoco/mujoco.h>
#include <mujoco/mjxmacro.h>
extern std::fstream fs;
//class MuscleLengthSensor
//{
//public:	
//	MuscleLengthSensor() 
//	{
//		for (int i = 0; i < 5; i++)
//		{
//			pulseInputs[i] = 0;
//			potentialOutputs[i] = 0;
//		}
//	}
//	MuscleLengthSensor(mjtNum i_thresholdLength, mjModel* i_m, mjData* i_d, int i_actuatorId) :
//		thresholdLength(i_thresholdLength),
//		m(i_m),
//		d(i_d),
//		actuatorId(i_actuatorId)
//	{
//		mjtNum threshold_pulse_period = (a * thresholdLength + b) * m->opt.timestep;
//		threshold_pulse_period_ticks = round(threshold_pulse_period / m->opt.timestep);
//
//		for (int i = 0; i < 5; i++)
//		{
//			pulseInputs[i] = 0;
//			potentialOutputs[i] = 0;
//		}
//	}
//	void Initialize(mjtNum i_thresholdLength, mjModel* i_m, mjData* i_d, int i_actuatorId)
//	{
//		thresholdLength = i_thresholdLength;
//		m = i_m;
//		d = i_d;
//		actuatorId = i_actuatorId;
//
//		mjtNum threshold_pulse_period = (a * thresholdLength + b) * m->opt.timestep;
//		threshold_pulse_period_ticks = round(threshold_pulse_period / m->opt.timestep);
//	}
//
//	void FourthOderFilter(mjtNum* x, mjtNum* y)
//	{
//		mjtNum a[5];
//		a[0] = 1;
//		a[1] = -3.998358124568343;
//		a[2] = 5.995075721448251;
//		a[3] = -3.995077068543508;
//		a[4] = 0.998359471663756;
//
//		mjtNum b[5];
//		b[0] = 9.732916985815186e-15;
//		b[1] = 3.893166794326075e-14;
//		b[2] = 5.839750191489111e-14;
//		b[3] = 3.893166794326075e-14;
//		b[4] = 9.732916985815186e-15;
//
//		y[4] = (b[0] * x[4] + b[1] * x[3] + b[2] * x[2] + b[3] * x[1] + b[4] * x[0] - a[1] * y[3] - a[2] * y[2] - a[3] * y[1] - a[4] * y[0]) / a[0];
//
//		//update x and y
//		for (int i = 0; i < 4; i++)
//		{
//			x[i] = x[i + 1];
//			y[i] = y[i + 1];
//		}
//	}
//	void Update()
//	{
//		int time_ticks = round(d->time / m->opt.timestep);
//		
//		mjtNum threshold_pulse_period = (a * thresholdLength + b) * m->opt.timestep;
//		threshold_pulse_period_ticks = round(threshold_pulse_period / m->opt.timestep);
//		mjtNum threshold_input = (time_ticks % threshold_pulse_period_ticks) ? 0 : 1;
//
//		mjtNum currentLength_pulse_period = (a * d->actuator_length[actuatorId] + b) * m->opt.timestep;
//		//mjtNum currentLength_pulse_period = (a * 0.3 + b) * m->opt.timestep;
//		currentLength_pulse_period_ticks = round(currentLength_pulse_period / m->opt.timestep);
//		mjtNum currentLength_input = (time_ticks % currentLength_pulse_period_ticks) ? 0 : 1;
//		
//		pulseInputs[4] = currentLength_input - threshold_input;
//		//pulseInputs[4] = threshold_input;
//		FourthOderFilter(pulseInputs, potentialOutputs);
//		if (thresholdLength > d->actuator_length[actuatorId]) output = 0;
//		else output = potentialOutputs[4];
//	}
//
//	mjtNum thresholdLength = 0;
//	mjtNum pulseInputs[5];
//	mjtNum potentialOutputs[5];
//	mjtNum output = 0;
//
//	int threshold_pulse_period_ticks = 0;
//	int currentLength_pulse_period_ticks = 0;
//private:
//	mjModel* m = nullptr;
//	mjData* d = nullptr;
//	mjtNum a = -1285.71;
//	mjtNum b = 1257.14;
//	int actuatorId = 0;
//}; 

class MuscleLengthSensor
{
public:
	MuscleLengthSensor(){}
	MuscleLengthSensor(mjModel* i_m, mjData* i_d, mjtNum i_thresholdLength, int i_actuatorId):
		m(i_m),
		d(i_d),
		thresholdLength(i_thresholdLength),
		actuatorId(i_actuatorId)
	{
		a = 200 / 0.4 * freqMultiplier;
	}
	void Initialize(mjModel* i_m, mjData* i_d, mjtNum i_thresholdLength, int i_actuatorId)
	{
		m = i_m;
		d = i_d;
		thresholdLength = i_thresholdLength;
		actuatorId = i_actuatorId;
		a = 200 / 0.4 * freqMultiplier;
	}
	void Update(mjtNum dt, mjtNum &currSimtime, mjtNum &ctrl)
	{
		threshold_f = a * thresholdLength;
		mjtNum threshold_period = 1 / threshold_f;
		input1 = 0;
		if (threshold_f > 0.0000001 && currSimtime - input1TimeStamp > threshold_period)
		{
			input1TimeStamp = currSimtime;
			input1 = 1;
		}
		input1Accum += input1;

		curr_f = a * d->actuator_length[actuatorId];
		mjtNum curr_period = 1 / curr_f;
		input2 = 0;
		if (curr_f > 0.0000001 && currSimtime - input2TimeStamp > curr_period)
		{
			input2TimeStamp = currSimtime;
			input2 = 1;
		}
		input2Accum += input2;
		
		inputsIntegrateResult += input2 - input1;
		if (inputsIntegrateResult < 0)
		{
			inputsIntegrateResult = 0;
		}
		output_period += dt;
		//thresholdReached = false;
		if (inputsIntegrateResult > freqMultiplier)
		{
			ctrl = 1;
			potential = inputsIntegrateResult;
			inputsIntegrateResult = 0;
			output_f = 1 / output_period;
			output_period = 0;
			thresholdReached = true;

			input1Accum = 0;
			input2Accum = 0;
		}
		currSimtime += dt;
	}
	mjtNum curr_f = 0;
	mjtNum threshold_f = 0;
	
	mjtNum a = 0;
	mjtNum thresholdLength = 0;
	
	mjtNum output_period = 0;
	mjtNum output_f = 0;
	mjtNum ctrl = 0;
	
	mjtNum inputsIntegrateResult = 0;
	mjtNum potential = 0;
	bool thresholdReached = false;
	
	mjtNum input1Accum = 0;
	mjtNum input2Accum = 0;
private:
	mjtNum input1 = 0;
	mjtNum input2 = 0;

	mjtNum input1TimeStamp = 0;
	mjtNum input2TimeStamp = 0;
	mjtNum freqMultiplier = 50;
	mjModel* m = nullptr;
	mjData* d = nullptr;
	int actuatorId = 0;
};