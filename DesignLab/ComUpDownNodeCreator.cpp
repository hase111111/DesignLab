#include "ComUpDownNodeCreator.h"
#include "HexapodConst.h"
#include <cfloat>
#include "LegState.h"

void ComUpDownNodeCreator::create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph)
{
	//�d�S���ł����������邱�Ƃ̂ł���ʒu�ƁC�ł��Ⴍ�����邱�Ƃ̂ł���ʒu�����߂�D
	float _highest_pos = FLT_MIN;
	float _lowest_pos = FLT_MAX;

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		//�ڒn���Ă���r�ɂ��Ă̂ݍl����D
		if (LegState::isGrounded(_current_node.leg_state, i) == true)
		{

		}
	}
}
