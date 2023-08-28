#include "basic_graphic_main_builder.h"

#include "graphic_main_basic.h"


std::unique_ptr<AbstractGraphicMain> BasicGraphicMainBuilder::build(const GraphicDataBroker* const broker, const SApplicationSettingRecorder* const setting)
{
	//�쐬���āC���j�[�N�|�C���^��move���ĕԂ��D
	return std::make_unique<GraphicMainBasic>(broker, setting);
}
