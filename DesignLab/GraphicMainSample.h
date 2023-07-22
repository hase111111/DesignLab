#pragma once
#include "InterfaceGraphicMain.h"


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


//! @file GraphicMainSample.h
//! @brief IGraphicMain�̎g�������
//! @author ���J��

//! @class GraphicMainSample
//! @brief �T���v���`��N���X�D�T���v�����b�Z�[�W��\������D
//! @details ���̃N���X�͎��ۂɎg�p����ړI�ō�������̂ł͂Ȃ��C�C���^�[�t�F�C�X��p�����N���X�̎����@�̉���̂��߂ɍ�������́D<br>
//! �Œ���C���̂悤�ȗv�f������Όp���ł���D<br>
//! [�\���ɂ���] <br>
//! �����Ⴒ���Ⴂ�낢����Ă��邪�C<br>
//! final �c ����ȏ�p���͂ł��Ȃ��Ƃ����Ӗ��D���̃N���X���X�Ɍp�����悤�Ƃ���ƃG���[�ɂȂ�D���Ȃ݂ɁC�p�����Ȃ��N���X�ɂ͊�{�I�ɕt���Ă����Ƃ悢�D<br>
//! public AbstractGraphicMain �c AbstractGraphicMain�N���X���p�������C�Ƃ����Ӗ��D<br>
//! override �c �p�����̊֐����I�[�o�[���C�h�����Ƃ����Ӗ��C�t���Ȃ��Ƃ����Ȃ����t���Ă����ƃR���p�C���ɖ����I�ɃI�[�o�[���C�h��m�点����̂Ńo�O��h���₷���D<br>
//! GraphicMainSample(const GraphicDataBroker* _broker) : IGraphicMain(_broker) {}; �c �R���X�g���N�^�ɂ����āC�e�N���X�̃R���X�g���N�^���Ăяo���C�Ƃ����Ӗ��D����������Y���ƃG���[�ɂȂ�̂Œ��ӁD
//! @author ���J��
