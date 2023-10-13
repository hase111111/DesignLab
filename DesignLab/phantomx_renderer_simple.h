//! @file phantomx_renderer_simple.h
//! @brief ���{�b�g�̕`����s���N���X�D

#ifndef DESIGNLAB_PHANTOMX_RENDERER_SIMPLE_H_
#define DESIGNLAB_PHANTOMX_RENDERER_SIMPLE_H_


#include <array>
#include <memory>

#include <Dxlib.h>

#include "abstract_hexapod_state_calculator.h"
#include "define.h"
#include "display_quality.h"
#include "hexapod_const.h"
#include "interface_hexapod_renderer.h"
#include "robot_state_node.h"


#ifndef DESIGNLAB_DONOT_USE_DXLIB


//! @class PhantomXRendererSimple
//! @brief PhantomX�̕`����s���N���X�D3D���f�����g�p�����C���p�`��g�ݍ��킹�ă��{�b�g��`�悷��D
class PhantomXRendererSimple final : public IHexapodRenderer
{
public:

	PhantomXRendererSimple(const std::shared_ptr<const AbstractHexapodStateCalculator>& calculator_ptr, DisplayQuality display_quality);

	void SetDrawNode(const RobotStateNode& node) override;

	void Draw() const override;

private:

	//! @struct PhantomXDrawData
	//! @brief �`��ɕK�v�ȃf�[�^�𖈉�v�Z����͖̂��ʂȂ̂ŁC���炩���ߌv�Z���āC���̒l��ێ����Ă������߂̍\���́D
	struct PhantomXDrawData
	{
		VECTOR coxa_joint_pos;		//!< �t�^���w�ŋ��߂���P�֐߂̈ʒu
		VECTOR femur_joint_pos;		//!< �t�^���w�ŋ��߂���Q�֐߂̈ʒu
		VECTOR tibia_joint_pos;		//!< �t�^���w�ŋ��߂���R�֐߂̈ʒu
		VECTOR leg_end_pos;			//!< �t�^���w�ŋ��߂��r��̈ʒu
		VECTOR leg_reference_pos;	//!< �r���ڒn����ۂɊ�Ƃ���ʒu

		float coxa_cos;		//!< ��P�֐߂̃R�T�C��
		float coxa_sin;		//!< ��P�֐߂̃T�C��
		float femur_cos;	//!< ��Q�֐߂̃R�T�C��
		float femur_sin;	//!< ��Q�֐߂̃T�C��
		float tibia_cos;	//!< ��R�֐߂̃R�T�C��
		float tibia_sin;	//!< ��R�֐߂̃T�C��

		designlab::Vector3 kine_coxa_joint_vec;		//!< �Ԑڊp�x���珇�^���w�ŋ��߂���P�֐߂̈ʒu(designlab::Vector3)�D�r���W�n
		designlab::Vector3 kine_femur_joint_vec;	//!< �Ԑڊp�x���珇�^���w�ŋ��߂���Q�֐߂̈ʒu(designlab::Vector3)�D�r���W�n
		designlab::Vector3 kine_tibia_joint_vec;	//!< �Ԑڊp�x���珇�^���w�ŋ��߂���R�֐߂̈ʒu(designlab::Vector3)�D�r���W�n
		designlab::Vector3 kine_leg_end_vec;		//!< �Ԑڊp�x���珇�^���w�ŋ��߂��r��̈ʒu(designlab::Vector3)�D�r���W�n

		VECTOR kine_coxa_joint_pos;		//!< �Ԑڊp�x���珇�^���w�ŋ��߂���P�֐߂̈ʒu(VECTOR)
		VECTOR kine_femur_joint_pos;	//!< �Ԑڊp�x���珇�^���w�ŋ��߂���Q�֐߂̈ʒu(VECTOR)
		VECTOR kine_tibia_joint_pos;	//!< �Ԑڊp�x���珇�^���w�ŋ��߂���R�֐߂̈ʒu(VECTOR)
		VECTOR kine_leg_end_pos;		//!< �Ԑڊp�x���珇�^���w�ŋ��߂��r��̈ʒu(VECTOR)

		float coxa_link_length;		//!< ��P�����N�̒���
		float femur_link_length;	//!< ��Q�����N�̒���
		float tibia_link_length;	//!< ��R�����N�̒���

		bool is_able_coxa_angle;	//!< ��P�֐߂̊p�x���L�����ǂ���
		bool is_able_femur_angle;	//!< ��Q�֐߂̊p�x���L�����ǂ���
		bool is_able_tibia_angle;	//!< ��R�֐߂̊p�x���L�����ǂ���
	};


	//! @brief �ʏ�ʂ�Ƀ��{�b�g�̕`�������
	void DrawHexapodNormal() const;

	//! @brief ��i���Ƀ��{�b�g�̕`�������
	void DrawHexapodLow() const;

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

	std::array<PhantomXDrawData, HexapodConst::kLegNum> draw_data_;			//!< �`��ɕK�v�ȃf�[�^

	DisplayQuality display_quality_;	//!< �`��i��
};

#endif

#endif