#include "other_graphic_main_builder.h"

#include "graphic_main_test.h"

std::unique_ptr<AbstractGraphicMain> TestGraphicMainBuilder::build(const GraphicDataBroker* const broker, std::shared_ptr<AbstractHexapodStateCalculator> calc,
	const SApplicationSettingRecorder* const setting)
{
	//�쐬���āC���j�[�N�|�C���^��move���ĕԂ��D
	return std::make_unique<GraphicMainTest>(broker, calc, setting);
}
