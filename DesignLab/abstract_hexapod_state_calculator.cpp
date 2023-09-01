#include "abstract_hexapod_state_calculator.h"

#include "leg_state.h"


dl_vec::SVector AbstractHexapodStateCalculator::getLocalLegBasePosition(const int leg_index) const
{
	if constexpr (DO_CHECK_LEG_INDEX)
	{
		if (!checkLegIndex(leg_index))
		{
			return { 0,0,0 };
		}
	}

	return m_local_leg_base_pos[leg_index];
}


float AbstractHexapodStateCalculator::calcStabilityMargin(const int leg_state, const dl_vec::SVector leg_pos[HexapodConst::LEG_NUM]) const
{
	//�d�S�����_�Ƃ������W�n�ŁC�r�̈ʒu���v�Z����D
	// std::min ���J�b�R�ň͂�ł���̂́C�}�N���� min �Ɣ�邽�߁D(std::min) �Ə����Ɩ��O���Փ˂��Ȃ�

	std::vector<dl_vec::SVector2> ground_leg_pos;

	//�ڒn�r�̂ݒǉ�����
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (dl_leg::isGrounded(leg_state, i))
		{
			ground_leg_pos.push_back(leg_pos[i].projectedXY() + getLocalLegBasePosition(i).projectedXY());
		}
	}


	float min_margin = 1000000;

	for (int i = 0; i < ground_leg_pos.size(); i++)
	{
		dl_vec::SVector2 i_to_i_plus_1 = ground_leg_pos.at((i + 1) % ground_leg_pos.size()) - ground_leg_pos.at(i);
		i_to_i_plus_1.normalized();

		dl_vec::SVector2 i_to_com = dl_vec::SVector2{ 0,0 } - ground_leg_pos.at(i);

		min_margin = (std::min)(min_margin, i_to_com.cross(i_to_i_plus_1));
	}

	return min_margin;
}
