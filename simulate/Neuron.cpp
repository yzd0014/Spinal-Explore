#include "Neuron.h"

namespace NeuronRateModel
{
	std::vector<mjtNum> rates;
	std::vector<mjtNum> sum_old;
	std::vector<mjtNum> positive_sum_old;
	
	std::vector<std::vector<int>> neighbors;
	std::vector<std::vector<mjtNum>> inputWeights;
	std::vector<std::vector<int>> inputSigns;

	void UpdateRates()
	{
		mjtNum Ka = 0.3;
		int neuron_num = rates.size();
		for (int i = 0; i < neuron_num; i++)
		{
			if (!neighbors.empty())
			{
				mjtNum X1 = 0;
				mjtNum X2 = 0;
				int neighbors_num = neighbors[i].size();
				for (int k = 0; k < neighbors_num; k++)
				{
					int j = neighbors[i][k];
					int w = inputWeights[i][k];
					int s = inputSigns[i][k];
					X1 += rates[j] * w * s;
					X2 += rates[j] * w;
				}
				mjtNum T1 = sum_old[i] * (1 - Ka) + X1 * Ka;
				mjtNum T2 = positive_sum_old[i] * (1 - Ka) + X2 * Ka;
				rates[i] = T1 / (0.5 + T2);
				if (rates[i] < 0)
				{
					rates[i] = 0;
				}

				//get ready for the next timestep
				sum_old[i] = T1;
				positive_sum_old[i] = T2;
			}
		}
	}
}