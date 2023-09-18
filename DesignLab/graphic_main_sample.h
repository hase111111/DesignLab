//! @file graphic_main_sample.h
//! @brief IGraphicMain�̎g�������

#ifndef DESIGNLAB_GRAPHIC_MAIN_SAMPLE_H_
#define DESIGNLAB_GRAPHIC_MAIN_SAMPLE_H_

#include "interface_graphic_main.h"

#include <memory>

#include "application_setting_recorder.h"


//! @class GraphicMainSample
//! @brief �T���v���`��N���X�D�T���v�����b�Z�[�W��\������D
//! @details ���̃N���X�͎��ۂɎg�p����ړI�ō�������̂ł͂Ȃ��C�C���^�[�t�F�C�X��p�����N���X�̎����@�̉���̂��߂ɍ�������́D
//! @n �Œ���C���̂悤�ȗv�f������Όp���ł���D
//! @n
//! @n [�\���ɂ���] 
//! @n �����Ⴒ���Ⴂ�낢����Ă��邪�C
//! @n final �c ����ȏ�p���͂ł��Ȃ��Ƃ����Ӗ��D���̃N���X���X�Ɍp�����悤�Ƃ���ƃG���[�ɂȂ�D���Ȃ݂ɁC�p�����Ȃ��N���X�ɂ͊�{�I�ɕt���Ă����Ƃ悢�D
//! @n public IGraphicMain �c IGraphicMain�N���X���p�������C�Ƃ����Ӗ��D
//! @n override �c �p�����̊֐����I�[�o�[���C�h�����Ƃ����Ӗ��C�t���Ȃ��Ƃ����Ȃ����t���Ă����ƃR���p�C���ɖ����I�ɃI�[�o�[���C�h��m�点����̂Ńo�O��h���₷���D

class GraphicMainSample final : public IGraphicMain
{
public:
	GraphicMainSample(const std::shared_ptr<const SApplicationSettingRecorder>& setting_ptr);
	~GraphicMainSample() = default;

	bool Update() override;

	void Draw() const override;

private:

	int kBoxSizeX, kBoxSizeY;

	int counter_ = 0;
};


#endif // !DESIGNLAB_GRAPHIC_MAIN_SAMPLE_H_