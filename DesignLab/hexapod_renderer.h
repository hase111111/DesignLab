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

	std::array<HexapodJointState, HexapodConst::LEG_NUM> draw_joint_state_;	//!< �`�悷�郍�{�b�g�̃W���C���g�̏��


	// ��ł܂Ƃ߂違�C��
	VECTOR kCoxaJointPos[HexapodConst::LEG_NUM];
	VECTOR kFemurJointPos[HexapodConst::LEG_NUM];
	VECTOR kTibiaJointPos[HexapodConst::LEG_NUM];
	VECTOR kLegEndPos[HexapodConst::LEG_NUM];
	VECTOR kLegBasePos[HexapodConst::LEG_NUM];

	float kCoxaCos[HexapodConst::LEG_NUM];
	float kCoxaSin[HexapodConst::LEG_NUM];
	float kFemurCos[HexapodConst::LEG_NUM];
	float kFemurSin[HexapodConst::LEG_NUM];
	float kTibiaCos[HexapodConst::LEG_NUM];
	float kTibiaSin[HexapodConst::LEG_NUM];

	designlab::Vector3 kKineCoxaJointVec[HexapodConst::LEG_NUM];
	designlab::Vector3 kKineFemurJointVec[HexapodConst::LEG_NUM];
	designlab::Vector3 kKineTibiaJointVec[HexapodConst::LEG_NUM];
	designlab::Vector3 kKineLegVec[HexapodConst::LEG_NUM];

	VECTOR kKineCoxaJointPos[HexapodConst::LEG_NUM];
	VECTOR kKineFemurJointPos[HexapodConst::LEG_NUM];
	VECTOR kKineTibiaJointPos[HexapodConst::LEG_NUM];
	VECTOR kKineLegPos[HexapodConst::LEG_NUM];

	float kCoxaLinkLength[HexapodConst::LEG_NUM];
	float kFemurLinkLength[HexapodConst::LEG_NUM];
	float kTibiaLinkLength[HexapodConst::LEG_NUM];

	bool kIsAbleCoxaAngle[HexapodConst::LEG_NUM];
	bool kIsAbleFemurAngle[HexapodConst::LEG_NUM];
	bool kIsAbleTibiaAngle[HexapodConst::LEG_NUM];
};


#endif // !DESIGNLAB_HEXAPOD_RENDERER_H_