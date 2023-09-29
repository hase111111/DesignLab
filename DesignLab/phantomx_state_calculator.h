#pragma once

#include "abstract_hexapod_state_calculator.h"

#include <algorithm>

#include "hexapod_const.h"


class PhantomXStateCalclator : public AbstractHexapodStateCalculator
{
public:

	PhantomXStateCalclator();


	bool CalculateAllJointState(const RobotStateNode& node, std::array<HexapodJointState, HexapodConst::LEG_NUM>* joint_state) const override;


	designlab::Vector3 ConvertGlobalToLegPosition(int leg_index, const designlab::Vector3& leg_pos, const designlab::Vector3& global_center_of_mass, const designlab::EulerXYZ& robot_rot, const bool consider_rot) const override;


	designlab::Vector3 GetFreeLegPosition(int leg_index) const override;


	designlab::Vector3 GetLocalLegBasePosition(int leg_index) const override;

	designlab::Vector3 GetLocalLegPosition(int leg_index, const designlab::Vector3& leg_pos) const override;


	designlab::Vector3 GetGlobalLegBasePosition(int leg_index, const designlab::Vector3& global_center_of_mass, const designlab::EulerXYZ& robot_rot, const bool consider_rot) const override;

	designlab::Vector3 GetGlobalLegPosition(int leg_index, const designlab::Vector3& leg_pos, const designlab::Vector3& global_center_of_mass, const designlab::EulerXYZ& robot_rot, const bool consider_rot) const override;


	virtual bool IsLegInRange(int leg_index, const designlab::Vector3& leg_pos) const override;

	bool IsLegInterfering(const std::array<designlab::Vector3, HexapodConst::LEG_NUM>& leg_pos) const override;


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

	void calculateLocalJointState(const int leg_index, const designlab::Vector3& leg_pos, HexapodJointState* joint_state) const;		// �r�ʒu����֐ߊp�x���v�Z����D1�r��



	// �r�ԍ��Cx���W�Cy���W�Cz���W�̏��ŃA�N�Z�X����ƁC���̍��W���L�����ǂ�����bool�Ŋi�[����Ă���
	bool m_is_able_leg_pos[HexapodConst::LEG_NUM][LEG_POS_DIV_NUM][LEG_POS_DIV_NUM][LEG_POS_DIV_NUM];

	//!< �r�̕t�����̍��W( leg base position)�D���{�b�g�̏d�S�����_�C�����Ă��������x���Ƃ������[�J��(���{�b�g)���W�n�ł���D
	designlab::Vector3 m_local_leg_base_pos[HexapodConst::LEG_NUM];	

	//!< �V�r����ʒu�D�r���W�n
	const std::array<designlab::Vector3, HexapodConst::LEG_NUM> free_leg_pos_;
};


