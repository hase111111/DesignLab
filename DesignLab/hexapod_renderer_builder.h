//@file hexapod_renderer_builder.h
//@brief HexapodRenderer�N���X�̃C���X�^���X���쐬����N���X


#ifndef DESIGNLAB_HEXAPOD_RENDERER_BUILDER_H_
#define DESIGNLAB_HEXAPOD_RENDERER_BUILDER_H_


#include <memory>

#include "abstract_hexapod_state_calculator.h"
#include "display_quality.h"
#include "interface_hexapod_renderer.h"


//! @class HexapodRendererBuilder
//! @brief HexapodRenderer�N���X�̃C���X�^���X���쐬����N���X
class HexapodRendererBuilder
{
public:

	//! @brief HexapodRenderer�N���X�̃C���X�^���X���쐬����Dstatic�����o�֐�
	//! @param [in] calculator AbstractHexapodStateCalculator�N���X�̃C���X�^���X�D
	//! @n ���̃N���X�̌^�𔻕ʂ��āC�K�؂�HexapodRenderer�N���X�̃C���X�^���X���쐬����
	//! @param [in] display_quality �`��i��
	//! @return HexapodRenderer�N���X�̃C���X�^���X
	static std::unique_ptr<IHexapodRenderer> Build(const std::shared_ptr<const AbstractHexapodStateCalculator>& calculator, DisplayQuality display_quality);
};


#endif 