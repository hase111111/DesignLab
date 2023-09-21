#include "abstract_hexapod_state_calculator.h"

#include "cassert_define.h"
#include "leg_state.h"


float AbstractHexapodStateCalculator::calcStabilityMargin(const dl_leg::LegStateBit& leg_state, const std::array<designlab::Vector3, HexapodConst::LEG_NUM>& leg_pos) const
{
	//�d�S�����_�Ƃ������W�n�ŁC�r�̈ʒu���v�Z����D
	// std::min ���J�b�R�ň͂�ł���̂́C�}�N���� min �Ɣ�邽�߁D(std::min) �Ə����Ɩ��O���Փ˂��Ȃ�

	std::vector<designlab::Vector2> ground_leg_pos;

	//�ڒn�r�̂ݒǉ�����
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (dl_leg::IsGrounded(leg_state, i))
		{
			ground_leg_pos.push_back(leg_pos[i].ProjectedXY() + getLocalLegBasePosition(i).ProjectedXY());
		}
	}


	float min_margin = 1000000;

	for (int i = 0; i < ground_leg_pos.size(); i++)
	{
		designlab::Vector2 i_to_i_plus_1 = ground_leg_pos.at((i + 1) % ground_leg_pos.size()) - ground_leg_pos.at(i);
		i_to_i_plus_1.Normalized();

		designlab::Vector2 i_to_com = designlab::Vector2{ 0,0 } - ground_leg_pos.at(i);

		min_margin = (std::min)(min_margin, i_to_com.Cross(i_to_i_plus_1));
	}

	return min_margin;
}
