#include "simulation_record.h"

#include "leg_state.h"
#include "hexapod_const.h"


std::ofstream& operator<<(std::ofstream& ofs, const SSimulationRecord& record)
{


	const size_t length = record.m_node.size();

	for (size_t i = 0; i < length; i++)
	{
		ofs << i << ",";

		//遊脚・接地脚の出力
		for (int j = 0; j < HexapodConst::LEG_NUM; j++)
		{
			ofs << dl_leg::isGrounded(record.m_node[i].leg_state, j) << ",";
		}

		//階層の出力
		for (int j = 0; j < HexapodConst::LEG_NUM; j++)
		{
			ofs << dl_leg::getLegState(record.m_node[i].leg_state, j) << ",";
		}

		//重心位置の出力
		ofs << dl_leg::getComPatternState(record.m_node[i].leg_state) << ",";

		//姿勢の出力
		ofs << record.m_node[i].rot.pitch << "," << record.m_node[i].rot.roll << "," << record.m_node[i].rot.yaw << ",";
	}
}
