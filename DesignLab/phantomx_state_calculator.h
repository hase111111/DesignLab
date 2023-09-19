#pragma once

#include "abstract_hexapod_state_calculator.h"

#include <algorithm>

#include "hexapod_const.h"


class PhantomXStateCalclator final : public AbstractHexapodStateCalculator
{
public:

	PhantomXStateCalclator();


	bool calculateAllJointState(const SNode& node, SHexapodJointState joint_state[HexapodConst::LEG_NUM]) const override;


	designlab::Vector3 convertGlobalToLegPosition(const int leg_index, const designlab::Vector3& leg_pos, const designlab::Vector3& global_center_of_mass, const designlab::SRotator& robot_rot, const bool consider_rot) const override;


	designlab::Vector3 getLocalLegPosition(const int leg_index, const designlab::Vector3& leg_pos) const override;


	designlab::Vector3 getGlobalLegBasePosition(const int leg_index, const designlab::Vector3& global_center_of_mass, const designlab::SRotator& robot_rot, const bool consider_rot) const override;

	designlab::Vector3 getGlobalLegPosition(const int leg_index, const designlab::Vector3& leg_pos, const designlab::Vector3& global_center_of_mass, const designlab::SRotator& robot_rot, const bool consider_rot) const override;


	bool isLegInRange(const int leg_index, const designlab::Vector3& leg_pos) const override;

	bool isLegInterfering(const designlab::Vector3 leg_pos[HexapodConst::LEG_NUM]) const override;


private:

	static constexpr int LEG_POS_DIV_NUM = 50;		//�r�ʒu�̗��U����

	static constexpr float LEG_POS_MARGIN = 2;

	static constexpr float LEG_POS_MIN = -(HexapodConst::PHANTOMX_COXA_LENGTH + HexapodConst::PHANTOMX_FEMUR_LENGTH + HexapodConst::PHANTOMX_TIBIA_LENGTH + LEG_POS_MARGIN);	//�r�ʒu�̍ŏ��l

	static constexpr float LEG_POS_MAX = (HexapodConst::PHANTOMX_COXA_LENGTH + HexapodConst::PHANTOMX_FEMUR_LENGTH + HexapodConst::PHANTOMX_TIBIA_LENGTH + LEG_POS_MARGIN);		//�r�ʒu�̍ő�l

	static constexpr float MIN_LEG_R = 120;


	constexpr int getLegPosIndex(const float leg_pos) const
	{
		constexpr float converter = LEG_POS_DIV_NUM / (LEG_POS_MAX - LEG_POS_MIN);
		int res = static_cast<int>((leg_pos - LEG_POS_MIN) * converter);			// ���U�������r�ʒu���擾

		return res;										// ���U�������r�ʒu��Ԃ�
	}

	bool initIsAbleLegPos(const int leg_index, const int x, const int y, const int z) const;		// �r�ʒu�̗L������������������

	void calculateLocalJointState(const int leg_index, const designlab::Vector3& leg_pos, SHexapodJointState* joint_state) const;		// �r�ʒu����֐ߊp�x���v�Z����D1�r��



	// �r�ԍ��Cx���W�Cy���W�Cz���W�̏��ŃA�N�Z�X����ƁC���̍��W���L�����ǂ�����bool�Ŋi�[����Ă���
	bool m_is_able_leg_pos[HexapodConst::LEG_NUM][LEG_POS_DIV_NUM][LEG_POS_DIV_NUM][LEG_POS_DIV_NUM];
};


