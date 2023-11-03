//! @file phantomx_state_calculator.h
//! @brief PhantomX�̏�Ԃ��v�Z����N���X�D


#ifndef DESIGNLAB_PHANTOMX_STATE_CALCULATOR_H_
#define DESIGNLAB_PHANTOMX_STATE_CALCULATOR_H_


#include "abstract_hexapod_state_calculator.h"

#include <algorithm>
#include <array>

#include "designlab_vector2.h"
#include "hexapod_const.h"
#include "phantomx_const.h"


//! @class PhantomXStateCalclator
//! @brief PhantomX�̏�Ԃ��v�Z����N���X�D
// todo ���낢��������݌v�Ȃ̂ŁC�S�ʓI�ɉ��C����D
class PhantomXStateCalclator : public AbstractHexapodStateCalculator
{
public:

	PhantomXStateCalclator();


	void CalculateAllJointState(const RobotStateNode& node, std::array<HexapodJointState, HexapodConst::kLegNum>* joint_state) const override;

	bool IsVaildJointState(const RobotStateNode& node, const std::array<HexapodJointState, HexapodConst::kLegNum>& joint_state) const override;

	bool IsVaildJointStateOneLeg(const int leg_index, const RobotStateNode& node, const HexapodJointState& joint_state) const override;


	designlab::Vector3 ConvertGlobalToLegPosition(int leg_index, const designlab::Vector3& leg_pos, const designlab::Vector3& global_center_of_mass, const designlab::EulerXYZ& robot_rot, const bool consider_rot) const override;


	designlab::Vector3 GetFreeLegPosition(int leg_index) const override;


	designlab::Vector3 GetLocalLegBasePosition(int leg_index) const override;

	designlab::Vector3 GetLocalLegPosition(int leg_index, const designlab::Vector3& leg_pos) const override;


	designlab::Vector3 GetGlobalLegBasePosition(int leg_index, const designlab::Vector3& global_center_of_mass, const designlab::EulerXYZ& robot_rot, const bool consider_rot) const override;

	designlab::Vector3 GetGlobalLegPosition(int leg_index, const designlab::Vector3& leg_pos, const designlab::Vector3& global_center_of_mass, const designlab::EulerXYZ& robot_rot, const bool consider_rot) const override;


	virtual bool IsLegInRange(int leg_index, const designlab::Vector3& leg_pos) const override;

	bool IsLegInterfering(const std::array<designlab::Vector3, HexapodConst::kLegNum>& leg_pos) const override;

protected:
	const std::array<::designlab::Vector2, HexapodConst::kLegNum> min_leg_pos_xy_;
	const std::array<::designlab::Vector2, HexapodConst::kLegNum> max_leg_pos_xy_;

private:

	static constexpr int kLegPosDivNum = 50;	//!< �r�ʒu�̗��U����

	static constexpr float kLegPosMargin = 2;

	static constexpr float kLegPosMin = -(PhantomXConst::kCoxaLength + PhantomXConst::kFemurLength + PhantomXConst::kTibiaLength + kLegPosMargin);	//�r�ʒu�̍ŏ��l

	static constexpr float kLegPosMax = (PhantomXConst::kCoxaLength + PhantomXConst::kFemurLength + PhantomXConst::kTibiaLength + kLegPosMargin);	//�r�ʒu�̍ő�l

	static constexpr float kMinLegR = 120;

	constexpr int GetLegPosIndex(const float leg_pos) const
	{
		constexpr float converter = kLegPosDivNum / (kLegPosMax - kLegPosMin);
		int res = static_cast<int>((leg_pos - kLegPosMin) * converter);			// ���U�������r�ʒu���擾

		return res;										// ���U�������r�ʒu��Ԃ�
	}

	bool InitIsAbleLegPos(int leg_index, int x, int y, int z) const;		// �r�ʒu�̗L������������������

	void CalculateLocalJointState(int leg_index, const designlab::Vector3& leg_pos, HexapodJointState* joint_state) const;		// �r�ʒu����֐ߊp�x���v�Z����D1�r��
	
	std::array<::designlab::Vector2, HexapodConst::kLegNum> InitMinLegPosXY() const;		// �r�̕t�����̍��W( leg base position)�̍ŏ��l���v�Z����
	std::array<::designlab::Vector2, HexapodConst::kLegNum> InitMaxLegPosXY() const;		// �r�̕t�����̍��W( leg base position)�̍ŏ��l���v�Z����


	// �r�ԍ��Cx���W�Cy���W�Cz���W�̏��ŃA�N�Z�X����ƁC���̍��W���L�����ǂ�����bool�Ŋi�[����Ă���
	bool is_able_leg_pos_[HexapodConst::kLegNum][kLegPosDivNum][kLegPosDivNum][kLegPosDivNum];

	//!< �r�̕t�����̍��W( leg base position)�D���{�b�g�̏d�S�����_�C�����Ă��������x���Ƃ������[�J��(���{�b�g)���W�n�ł���D
	designlab::Vector3 local_leg_base_pos_[HexapodConst::kLegNum];	

	//!< �V�r����ʒu�D�r���W�n
	const std::array<designlab::Vector3, HexapodConst::kLegNum> free_leg_pos_;
};


#endif