//! @file hexapod_renderer_builder.h
//! @brief HexapodRenderer�N���X�̃C���X�^���X���쐬����N���X


#ifndef DESIGNLAB_HEXAPOD_RENDERER_BUILDER_H_
#define DESIGNLAB_HEXAPOD_RENDERER_BUILDER_H_


#include <memory>

#include "display_quality.h"
#include "interface_hexapod_renderer.h"
#include "interface_hexapod_coordinate_converter.h"
#include "interface_hexapod_joint_calculator.h"

//! @class HexapodRendererBuilder
//! @brief HexapodRenderer�N���X�̃C���X�^���X���쐬����N���X
class HexapodRendererBuilder final
{
public:

	//! @brief HexapodRenderer�N���X�̃C���X�^���X���쐬����Dstatic�֐��Ȃ̂ŁCHexapodRendererBuilder::Build()�ƌĂяo���D
	//! @param [in] calculator AbstractHexapodStateCalculator�N���X�̃C���X�^���X�D
	//! @n ���̃N���X�̌^�𔻕ʂ��āC�K�؂�HexapodRenderer�N���X�̃C���X�^���X���쐬����D
	//! @param [in] converter AbstractHexapodCoordinateConverter�N���X�̃C���X�^���X�Dshared_ptr�œn�����ƁD
	//! @param [in] calculator AbstractHexapodStateCalculator�N���X�̃C���X�^���X�Dshared_ptr�œn�����ƁD
	//! @param [in] display_quality �`��i���D
	//! @return HexapodRenderer�N���X�̃C���X�^���X�Dunique_ptr�ŕԂ��D
	static std::unique_ptr<IHexapodRenderer> Build(
		const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr,
		const std::shared_ptr<const IHexapodJointCalculator>& calculator_ptr,
		DisplayQuality display_quality
	);
};


#endif	// #ifndef DESIGNLAB_HEXAPOD_RENDERER_BUILDER_H_