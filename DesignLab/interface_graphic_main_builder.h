#pragma once

#include <memory>

#include "interface_graphic_main.h"
#include "graphic_data_broker.h"
#include "abstract_hexapod_state_calculator.h"
#include "application_setting_recorder.h"


class IGraphicMainBuilder
{
public:
	IGraphicMainBuilder() = default;
	virtual ~IGraphicMainBuilder() = default;

	//! @brief GraphicMain�̃C���X�^���X�𐶐�����D
	//! @param[in] broker GraphicMain�ɓn���f�[�^�𒇉��N���X�̃|�C���^�D
	//! @param[in] calc ���{�b�g�̏�Ԃ��v�Z����N���X�̃V�F�A�[�h�|�C���^�D
	//! @param[in] setting �A�v���P�[�V�����̐ݒ��ێ�����N���X�̃|�C���^�D
	virtual std::unique_ptr<IGraphicMain> build(const GraphicDataBroker* const broker, std::shared_ptr<AbstractHexapodStateCalculator> calc,
		const SApplicationSettingRecorder* const setting) = 0;

private:

};
