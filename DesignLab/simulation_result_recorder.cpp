#include "simulation_result_recorder.h"

#include <magic_enum.hpp>

#include "leg_state.h"
#include "hexapod_const.h"


std::ofstream& operator<<(std::ofstream& ofs, const SimulationResultRecorder& record)
{
	//�V�~�����[�V�����̍ŏI�I�Ȍ��ʂ̏o��
	ofs << "Simulation Result," << magic_enum::enum_name(record.simulation_result) << std::endl;
	ofs << std::endl;

	ofs << GraphSearchResultRecoder::GetCSVHeader() << std::endl;

	const size_t kLength = record.graph_search_result_recoder.size();

	for (size_t i = 0; i < kLength; i++)
	{
		ofs << i << ",";

		//�O���t�T���̌��ʂ̏o��
		ofs << record.graph_search_result_recoder[i].ToCSVString() << ",";

		ofs << std::endl;
	}


	ofs << std::endl;

	return ofs;
}