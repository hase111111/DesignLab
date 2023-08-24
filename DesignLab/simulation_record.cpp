#include "simulation_record.h"

#include "leg_state.h"
#include "hexapod_const.h"


std::ofstream& operator<<(std::ofstream& ofs, const SSimulationRecord& record)
{
	const size_t kLength = record.result_nodes.size();


	for (size_t i = 0; i < kLength; i++)
	{
		ofs << i << ",";

		//�m�[�h�̏�Ԃ̏o��
		ofs << record.result_nodes[i] << ",";

		//�v�Z���Ԃ̏o��
		if (record.computation_time.size() > i)
		{
			ofs << record.computation_time[i] << ",";
		}
		else
		{
			ofs << ",";
		}

		//�O���t�T���̌��ʂ̏o��
		if (record.graph_search_results.size() > i)
		{
			ofs << std::to_string(record.graph_search_results[i]) << ",";
		}
		else
		{
			ofs << ",";
		}

		ofs << std::endl;
	}

	//�ŏI�I�Ȍ��ʂ̏o��
	ofs << std::to_string(record.simulation_result) << ",";

	ofs << std::endl;

	return ofs;
}
