#include "simulation_result_recorder.h"

#include <sstream>

#include <magic_enum.hpp>

#include "leg_state.h"
#include "hexapod_const.h"


std::string SimulationResultRecorder::ToCsvString() const
{
	//シミュレーションの最終的な結果の出力
	std::stringstream ss;
	ss << "Simulation Result," << magic_enum::enum_name(simulation_result) << std::endl;
	ss << std::endl;

	ss << GraphSearchResultRecoder::GetCsvHeader() << std::endl;

	const size_t kLength = graph_search_result_recoder.size();

	for (size_t i = 0; i < kLength; i++)
	{
		ss << i << ",";

		//グラフ探索の結果の出力
		ss << graph_search_result_recoder[i].ToCsvString() << ",";

		ss << std::endl;
	}


	ss << std::endl;

	return ss.str();
}
