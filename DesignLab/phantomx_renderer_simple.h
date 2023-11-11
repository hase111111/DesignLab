//! @file phantomx_renderer_simple.h
//! @brief ���{�b�g�̕`����s���N���X�D

#ifndef DESIGNLAB_PHANTOMX_RENDERER_SIMPLE_H_
#define DESIGNLAB_PHANTOMX_RENDERER_SIMPLE_H_


#include <array>
#include <memory>

#include "define.h"
#include "display_quality.h"
#include "hexapod_const.h"
#include "interface_hexapod_coordinate_converter.h"
#include "interface_hexapod_joint_calculator.h"
#include "robot_state_node.h"

#ifndef DESIGNLAB_DONOT_USE_DXLIB

#include <Dxlib.h>

#include "interface_hexapod_renderer.h"


//! @class PhantomXRendererSimple
//! @brief PhantomX�̕`����s���N���X�D3D���f�����g�p�����C���p�`��g�ݍ��킹�ă��{�b�g��`�悷��D
class PhantomXRendererSimple final : public IHexapodRenderer
{
public:

	PhantomXRendererSimple(
		const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr,
		const std::shared_ptr<const IHexapodJointCalculator>& calculator_ptr,
		DisplayQuality display_quality
	);

	void SetDrawNode(const RobotStateNode& node) override;

	void Draw() const override;

private:

	//! @brief �ʏ�ʂ�Ƀ��{�b�g�̕`�������
	void DrawHexapodNormal() const;

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


	const std::shared_ptr<const IHexapodCoordinateConverter> converter_ptr_;
	const std::shared_ptr<const IHexapodJointCalculator> calculator_ptr_;	

	RobotStateNode draw_node_;						//!< �`�悷�郍�{�b�g�̏��

	std::array<HexapodJointState, HexapodConst::kLegNum> draw_joint_state_;	//!< �`�悷�郍�{�b�g�̃W���C���g�̏��

	DisplayQuality display_quality_;	//!< �`��i��
};

#endif	// #ifndef DESIGNLAB_DONOT_USE_DXLIB

#endif	// #ifndef DESIGNLAB_PHANTOMX_RENDERER_SIMPLE_H_