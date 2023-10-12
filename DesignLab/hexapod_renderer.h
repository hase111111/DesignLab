//! @file hexapod_renderer.h
//! @brief ���{�b�g�̕`����s���N���X�D

#ifndef DESIGNLAB_HEXAPOD_RENDERER_H_
#define DESIGNLAB_HEXAPOD_RENDERER_H_


#include <array>
#include <memory>

#include <Dxlib.h>

#include "abstract_hexapod_state_calculator.h"
#include "hexapod_const.h"
#include "robot_state_node.h"


//! @class HexapodRenderer
//! @brief ���{�b�g�̕`����s���N���X�D
class HexapodRenderer
{
public:
	HexapodRenderer(const std::shared_ptr<const AbstractHexapodStateCalculator>& calculator_ptr);
	~HexapodRenderer() = default;

	//! @brief ���{�b�g�̏�Ԃ��X�V����D
	//! @param [in] node �`�悷�郍�{�b�g�̏��
	void SetDrawNode(const RobotStateNode& node);

	//! @brief ���{�b�g��3D��Ԃɕ`�悷��D
	void Draw() const;

private:

	bool IsAbleCoxaLeg(const designlab::Vector3& coxa_joint, const designlab::Vector3& femur_joint) const;
	bool IsAbleFemurLeg(const designlab::Vector3& femur_joint, const designlab::Vector3& tibia_joint) const;
	bool IsAbleTibiaLeg(const designlab::Vector3& tibia_joint, const designlab::Vector3& leg_joint) const;


	const unsigned int kColorBody;			//!< ���̂̐F
	const unsigned int kColorLeg;			//!< �r�̐F
	const unsigned int kColorLiftedLeg;		//!< �V�r���Ă���r�̐F
	const unsigned int kColorJoint;			//!< �W���C���g�̐F
	const unsigned int kColorLiftedJoint;	//!< �V�r���Ă���W���C���g�̐F
	const unsigned int kColorLegBase;		//!< �r�̊�̐F
	const unsigned int kColorKineLeg;
	const unsigned int kColorKineJoint;
	const unsigned int kColorErrorJoint;	//!< �����̐F
	const unsigned int kColorErrorText;		//!< �G���[�̕����F

	const int kCapsuleDivNum;	//!< ���{�b�g�̃��f���̉~�����ǂꂾ���ׂ����`�悷�邩�D4 �` 20���炢�����傤�ǂ悢�Ǝv���D
	const int kSphereDivNum;	//!< ���{�b�g�̃��f���̋����ǂꂾ���ׂ����`�悷�邩�D16 �` 32���炢�����傤�ǂ悢�Ǝv���D
	const float kLegRadius;		//!< �r�̔��a�D���̃N���X�ł͋r���~���ɋߎ����ĕ`�悵�Ă���D�`�掞�̃f�[�^�̂��߁C�����ω������Ă��V�~�����[�V�����ɉe���͂Ȃ��D
	const float kJointRadius;	//!< �W���C���g�̔��a�D�`�掞�̃f�[�^�̂��߁C�����ω������Ă��V�~�����[�V�����ɉe���͂Ȃ��D

	const bool kDoOutputDebugLog = false;	//!< �r��Ԃ𕶎���ŏo�͂��邩�ǂ���


	std::shared_ptr<const AbstractHexapodStateCalculator> calculator_ptr_;	//!< ���{�b�g�̏�Ԃ��v�Z����N���X

	RobotStateNode draw_node_;						//!< �`�悷�郍�{�b�g�̏��

	std::array<HexapodJointState, HexapodConst::kLegNum> draw_joint_state_;	//!< �`�悷�郍�{�b�g�̃W���C���g�̏��


	// ��ł܂Ƃ߂違�C��
	VECTOR kCoxaJointPos[HexapodConst::kLegNum];
	VECTOR kFemurJointPos[HexapodConst::kLegNum];
	VECTOR kTibiaJointPos[HexapodConst::kLegNum];
	VECTOR kLegEndPos[HexapodConst::kLegNum];
	VECTOR kLegBasePos[HexapodConst::kLegNum];

	float kCoxaCos[HexapodConst::kLegNum];
	float kCoxaSin[HexapodConst::kLegNum];
	float kFemurCos[HexapodConst::kLegNum];
	float kFemurSin[HexapodConst::kLegNum];
	float kTibiaCos[HexapodConst::kLegNum];
	float kTibiaSin[HexapodConst::kLegNum];

	designlab::Vector3 kKineCoxaJointVec[HexapodConst::kLegNum];
	designlab::Vector3 kKineFemurJointVec[HexapodConst::kLegNum];
	designlab::Vector3 kKineTibiaJointVec[HexapodConst::kLegNum];
	designlab::Vector3 kKineLegVec[HexapodConst::kLegNum];

	VECTOR kKineCoxaJointPos[HexapodConst::kLegNum];
	VECTOR kKineFemurJointPos[HexapodConst::kLegNum];
	VECTOR kKineTibiaJointPos[HexapodConst::kLegNum];
	VECTOR kKineLegPos[HexapodConst::kLegNum];

	float kCoxaLinkLength[HexapodConst::kLegNum];
	float kFemurLinkLength[HexapodConst::kLegNum];
	float kTibiaLinkLength[HexapodConst::kLegNum];

	bool kIsAbleCoxaAngle[HexapodConst::kLegNum];
	bool kIsAbleFemurAngle[HexapodConst::kLegNum];
	bool kIsAbleTibiaAngle[HexapodConst::kLegNum];
};


#endif // !DESIGNLAB_HEXAPOD_RENDERER_H_