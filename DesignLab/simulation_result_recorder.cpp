#include "simulation_result_recorder.h"

#include <sstream>

#include "designlab_string_util.h"
#include "leg_state.h"
#include "hexapod_const.h"


namespace dlsu = ::designlab::string_util;


std::string SimulationResultRecorder::ToCsvString() const
{
	//�V�~�����[�V�����̍ŏI�I�Ȍ��ʂ̏o��
	std::stringstream ss;
	ss << "Simulation Result," << dlsu::MyEnumToString(simulation_result) << std::endl;
	ss << std::endl;

	ss << GraphSearchResultRecoder::GetCsvHeader() << std::endl;

	const size_t kLength = graph_search_result_recoder.size();

	for (size_t i = 0; i < kLength; i++)
	{
		ss << i << ",";

		//�O���t�T���̌��ʂ̏o��
		ss << graph_search_result_recoder[i].ToCsvString() << ",";

		ss << std::endl;
	}


	ss << std::endl;

	return ss.str();
}
