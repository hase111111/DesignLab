#pragma once
#include "InterfaceGraphicMain.h"

//�����Ⴒ���Ⴂ�낢����Ă��܂����C
// final �c ����ȏ�p���͂ł��Ȃ��Ƃ����Ӗ��D���̃N���X���X�Ɍp�����悤�Ƃ���ƃG���[�ɂȂ�܂��D�p�����Ȃ��N���X�ɂ͊�{�I�ɕt���Ă����Ƃ悢�ł��D
// public AbstractGraphicMain �c AbstractGraphicMain�N���X���p�������C�Ƃ����Ӗ�
// override �c �p�����̊֐����I�[�o�[���C�h�����Ƃ����Ӗ��C�t���Ȃ��Ƃ����Ȃ����t���Ă����ƃR���p�C���ɖ����I�ɃI�[�o�[���C�h��m�点����̂Ńo�O��h���₷���D

//�T���v���`��N���X�D�T���v�����b�Z�[�W��\�����܂��D
class GraphicMainSample final : public IGraphicMain
{
public:
	GraphicMainSample(const GraphicDataBroker* _broker) : IGraphicMain(_broker) {};
	~GraphicMainSample() = default;

	bool update() override;

	void draw() const override;

private:

	int m_counter = 0;
};
