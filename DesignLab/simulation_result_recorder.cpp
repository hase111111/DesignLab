#include "simulation_result_recorder.h"

#include <magic_enum.hpp>

#include "leg_state.h"
#include "hexapod_const.h"


std::ofstream& operator<<(std::ofstream& ofs, const SimulationResultRecorder& record)
{
	const size_t kLength = record.result_nodes.size();
	const int kPrecision = 3;

	ofs << magic_enum::enum_name(record.simulation_result) << std::endl;	//�ŏI�I�Ȍ��ʂ̏o��


	ofs << std::endl;

	ofs << "number,is Grounded,,,,,,Discretized Leg Pos,,,,,,Com Type,Rotate[rad],,,Leg Pos 0[mm],,,Leg Pos 1[mm],,,Leg Pos 2[mm],,,Leg Pos 3[mm],,,Leg Pos 4[mm],,,Leg Pos 5[mm],,,";
	ofs << "Leg Base Pos 0[mm],,,Leg Base Pos 1[mm],,,Leg Base Pos 2[mm],,,Leg Base Pos 3[mm],,,Leg Base Pos 4[mm],,,Leg Base Pos 5[mm],,,Center of Mass[mm],,,Next Move,Time[msec],Graph Search Result" << std::endl;
	ofs << ",leg0,leg1,leg2,leg3,leg4,leg5,leg0,leg1,leg2,leg3,leg4,leg5,,,,,x,y,z,x,y,z,x,y,z,x,y,z,x,y,z,x,y,z,x,y,z,x,y,z,x,y,z,x,y,z,x,y,z,x,y,z,x,y,z," << std::endl;

	for (size_t i = 0; i < kLength; i++)
	{
		ofs << i << ",";

		//�m�[�h�̏�Ԃ̏o��
		ofs << record.result_nodes[i] << ",";

		//�v�Z���Ԃ̏o��
		if (record.computation_time.size() > i)
		{
			ofs << std::fixed << std::setprecision(kPrecision) << record.computation_time[i] << ",";
		}
		else
		{
			ofs << ",";
		}

		//�O���t�T���̌��ʂ̏o��
		if (record.graph_search_results.size() > i)
		{
			//magic enum���g���ė񋓌^�𕶎���ɕϊ�
			ofs << magic_enum::enum_name(record.graph_search_results[i]) << ",";
		}
		else
		{
			ofs << ",";
		}

		ofs << std::endl;
	}


	ofs << std::endl;

	return ofs;
}