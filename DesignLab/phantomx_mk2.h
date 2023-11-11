//! @file phantomx_mk2.h
//! @brief PhantomX�̏�Ԃ��v�Z����N���X�D


#ifndef DESIGNLAB_PHANTOMX_STATE_CALCULATOR_H_
#define DESIGNLAB_PHANTOMX_STATE_CALCULATOR_H_


#include "interface_hexapod_coordinate_converter.h"
#include "interface_hexapod_joint_calculator.h"
#include "interface_hexapod_state_presenter.h"
#include "interface_hexapod_vaild_checker.h"


//! @class PhantomXMkII
//! @brief PhantomX mk-2 �̏�Ԃ��v�Z����N���X�D
class PhantomXMkII : public IHexapodCoordinateConverter, public IHexapodJointCalculator, public IHexapodStatePresenter, public IHexapodVaildChecker
{
public:

	PhantomXMkII();


	std::array<HexapodJointState, HexapodConst::kLegNum> CalculateAllJointState(const RobotStateNode& node) const noexcept override;

	HexapodJointState CalculateJointState(const int leg_index, const designlab::Vector3& leg_pos) const noexcept override;

	bool IsVaildAllJointState(const RobotStateNode& node, const std::array<HexapodJointState, HexapodConst::kLegNum>& joint_state) const noexcept override;

	bool IsVaildJointState(const int leg_index, const designlab::Vector3& leg_pos, const HexapodJointState& joint_state) const noexcept override;


	designlab::Vector3 ConvertGlobalToLegCoordinate(const designlab::Vector3& converted_position, int leg_index, 
		const designlab::Vector3& center_of_mass_global, const designlab::EulerXYZ& robot_rot, const bool consider_rot) const override;

	designlab::Vector3 ConvertLegToGlobalCoordinate(const designlab::Vector3& converted_position, int leg_index, 
		const designlab::Vector3& center_of_mass_global, const designlab::EulerXYZ& robot_rot, const bool consider_rot) const override;

	designlab::Vector3 ConvertRobotToGlobalCoordinate(const designlab::Vector3& converted_position, 
		const designlab::Vector3& center_of_mass_global, const designlab::EulerXYZ& robot_rot, const bool consider_rot) const override;


	designlab::Vector3 GetFreeLegPosLegCoodinate(int leg_index) const noexcept override;

	designlab::Vector3 GetLegBasePosRobotCoodinate(int leg_index) const noexcept override;

	float GetGroundHeightMarginMin() const noexcept override;

	float GetGroundHeightMarginMax() const noexcept override;


	bool IsLegInRange(int leg_index, const designlab::Vector3& leg_pos) const override;

	bool IsLegInterfering(const std::array<designlab::Vector3, HexapodConst::kLegNum>& leg_pos) const override;

	float CalculateStabilityMargin(const ::designlab::leg_func::LegStateBit& leg_state,
		const std::array<designlab::Vector3, HexapodConst::kLegNum>& leg_pos) const override;


private:

	const float kBodyLiftingHeightMin = 30;		//!< �n�ʂ��瓷�̂������グ�鍂��[mm]�D�ŏ������܂ŉ�������D
	const float kBodyLiftingHeightMax = 160;	//!< �n�ʂ��瓷�̂������グ�鍂��[mm]�D�ő傱���܂ŏグ����D

	const float kMovableCoxaAngleMin = designlab::math_util::ConvertDegToRad(-40.f);	//!< �r�̉��͈͂̍ŏ��l[rad]
	const float kMovableCoxaAngleMax = designlab::math_util::ConvertDegToRad(40.f);	//!< �r�̉��͈͂̍ő�l[rad]

	static constexpr float kMinLegR{120.f};		//!< �r�̕t��������r��܂ł̍ŏ�����[mm]
	static constexpr int kMaxLegRSize{200};		//!< kMaxLegR�̔z��̃T�C�Y�D
	std::array<float, kMaxLegRSize> kMaxLegR;	//!< �r�̕t��������r��܂ł̍ő勗��[mm]�D�r�̕t�����Əd�S��z�����̋����̍����C���f�b�N�X�ɂ���D

	std::array<designlab::Vector2, HexapodConst::kLegNum> kMinLegPosXY;	//!< coxa joint�̍ŏ��ʒu�܂ŉ񂵂����̋r����W�D�r���W�n��xy����݂����W�D
	std::array<designlab::Vector2, HexapodConst::kLegNum> kMaxLegPosXY;	//!< coxa joint�̍ő�ʒu�܂ŉ񂵂����̋r����W�D�r���W�n��xy����݂����W�D

	const float kFreeLegHeight = -20.f;			//!< �d�S���猩���V�r����[mm]�D


	//!< �r�̕t�����̍��W( leg base position )�D���{�b�g���W�n�D
	const std::array<designlab::Vector3, HexapodConst::kLegNum> leg_base_pos_robot_coordinate_;

	//!< �V�r����ʒu�D�r���W�n�D
	const std::array<designlab::Vector3, HexapodConst::kLegNum> free_leg_pos_leg_coordinate_;


	std::array<float, kMaxLegRSize> InitMaxLegR() const;
	std::array<designlab::Vector2, HexapodConst::kLegNum> InitMinLegPosXY() const;
	std::array<designlab::Vector2, HexapodConst::kLegNum> InitMaxLegPosXY() const;
};


#endif