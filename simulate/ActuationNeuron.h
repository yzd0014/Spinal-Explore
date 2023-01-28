#pragma once
#include <mujoco/mujoco.h>
#include <mujoco/mjxmacro.h>

class ActuationNeuron
{
public:
	ActuationNeuron(){}
	ActuationNeuron(const mjModel* i_m, mjData* i_d, mjtNum i_thresholdLength, int i_actuatorId, int i_input3Id):
		m(i_m),
		d(i_d),
		thresholdLength(i_thresholdLength),
		actuatorId(i_actuatorId),
		input3Id(i_input3Id)
	{
		a = 200 / 0.4 * freqMultiplier;
	}
	void Update(mjtNum dt, mjtNum currSimtime, mjtNum &output)
	{
		threshold_f = a * thresholdLength;
		input1 = 0;
		if (threshold_f > 0.0000001 && currSimtime - input1TimeStamp > 1 / threshold_f)
		{
			input1TimeStamp = currSimtime;
			input1 = 1;
		}
		input1Accum += input1;

		curr_f = a * d->actuator_length[actuatorId];
		input2 = 0;
		if (curr_f > 0.0000001 && currSimtime - input2TimeStamp > 1 / curr_f)
		{
			input2TimeStamp = currSimtime;
			input2 = 1;
		}
		input2Accum += input2;

		input3 = 0;
		mjtNum w = inhibitoryCoeff * a * d->actuator_length[input3Id];
		if (w > 0.0000001 && currSimtime - input3TimeStamp > 1 / w)
		{
			input3TimeStamp = currSimtime;
			input3 = 1;
		}
		
		inputsIntegrateResult += input2 - input1 - input3;
		if (inputsIntegrateResult < 0)
		{
			inputsIntegrateResult = 0;
		}
		output_period += dt;
		if (inputsIntegrateResult > freqMultiplier)
		{
			output = 1;
			potential = inputsIntegrateResult;
			inputsIntegrateResult = 0;
			output_f = 1 / output_period;
			output_period = 0;
			thresholdReached = true;

			input1Accum = 0;
			input2Accum = 0;
		}
	}
	mjtNum curr_f = 0;
	mjtNum threshold_f = 0;
	
	mjtNum a = 0;
	mjtNum thresholdLength = 0;
	
	mjtNum output_period = 0;
	mjtNum output_f = 0;
	
	mjtNum inputsIntegrateResult = 0;
	mjtNum potential = 0;
	bool thresholdReached = false;
	
	mjtNum input1Accum = 0;//for debug purpose
	mjtNum input2Accum = 0;//for debug purpose

	mjtNum input1 = 0;
	mjtNum input2 = 0;
	mjtNum input3 = 0;
private:
	mjtNum input1TimeStamp = 0;
	mjtNum input2TimeStamp = 0;
	mjtNum input3TimeStamp = 0;

	mjtNum freqMultiplier = 50;
	mjtNum inhibitoryCoeff = 0;
	const mjModel* m = nullptr;
	mjData* d = nullptr;
	int actuatorId = 0;
	int input3Id = 0;
};