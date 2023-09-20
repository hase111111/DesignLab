#include "abstract_hexapod_state_calculator.h"

#include "leg_state.h"


designlab::Vector3 AbstractHexapodStateCalculator::getLocalLegBasePosition(const int leg_index) const
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


float AbstractHexapodStateCalculator::calcStabilityMargin(const std::bitset<dl_leg::LEG_STATE_BIT_NUM> leg_state, const designlab::Vector3 leg_pos[HexapodConst::LEG_NUM]) const
{
	//重心を原点とした座標系で，脚の位置を計算する．
	// std::min をカッコで囲んでいるのは，マクロの min と被るため．(std::min) と書くと名前が衝突しない

	std::vector<designlab::SVector2> ground_leg_pos;

	//接地脚のみ追加する
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
		designlab::SVector2 i_to_i_plus_1 = ground_leg_pos.at((i + 1) % ground_leg_pos.size()) - ground_leg_pos.at(i);
		i_to_i_plus_1.Normalized();

		designlab::SVector2 i_to_com = designlab::SVector2{ 0,0 } - ground_leg_pos.at(i);

		min_margin = (std::min)(min_margin, i_to_com.Cross(i_to_i_plus_1));
	}

	return min_margin;
}
