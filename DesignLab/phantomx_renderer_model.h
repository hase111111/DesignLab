//! @file phantomx_renderer_model.h
//! @brief ���{�b�g�̕`����s���N���X�D

#ifndef DESIGNLAB_PHANTOMX_RENDERER_MODEL_H_
#define DESIGNLAB_PHANTOMX_RENDERER_MODEL_H_


#include <array>
#include <memory>

#include <Dxlib.h>

#include "define.h"
#include "display_quality.h"
#include "hexapod_const.h"
#include "interface_hexapod_renderer.h"
#include "interface_hexapod_coordinate_converter.h"
#include "interface_hexapod_joint_calculator.h"
#include "robot_state_node.h"


#ifndef DESIGNLAB_DONOT_USE_DXLIB


//! @class PhantomXRendererModel
//! @brief PhantomX�̕`����s���N���X�D3D���f�����g�p����
class PhantomXRendererModel final : public IHexapodRenderer
{
public:

	PhantomXRendererModel(
		const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr,
		const std::shared_ptr<const IHexapodJointCalculator>& calculator_ptr
	);

	void SetDrawNode(const RobotStateNode& node) override;

	void Draw() const override;

private:

	void DrawBody() const;

	void DrawCoxaLink(int leg_index) const;

	void DrawFemurLink(int leg_index) const;

	void DrawTibiaLink(int leg_index) const;

	void DrawJointAxis(int leg_index) const;

	const std::shared_ptr<const IHexapodCoordinateConverter> converter_ptr_;	//!< ���{�b�g�̋r����W���v�Z����N���X
	const std::shared_ptr<const IHexapodJointCalculator> calculator_ptr_;	//!< ���{�b�g�̊Ԑڈʒu���v�Z����N���X

	RobotStateNode draw_node_;	//!< �`�悷�郍�{�b�g�̏��
	std::array<HexapodJointState, HexapodConst::kLegNum> draw_joint_state_;	//!< �`�悷�郍�{�b�g�̃W���C���g�̏��
	DisplayQuality display_quality_;	//!< �`��i��
};

#endif	// #ifndef DESIGNLAB_DONOT_USE_DXLIB

#endif	// #ifndef DESIGNLAB_PHANTOMX_RENDERER_MODEL_H_