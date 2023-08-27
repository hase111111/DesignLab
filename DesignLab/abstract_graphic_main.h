#pragma once

#include "graphic_data_broker.h"


//! @class AbstractGraphicMain
//! @date 2023/08/09
//! @author ���J��
//! @brief GraphicMain�̒��ۃN���X
//! @details �`��̏������s���N���X�͕K�����̃N���X���p�����邱�ƁD�t�Ɍ����Όp������Ύ��R�ɏ����������\�ɂȂ�D@n 
//! @n �Q�l: https://qiita.com/okazuki/items/a0f2fb0a63ca88340ff6 
//! @n https://qiita.com/okazuki/items/0c17a161a921847cd080 
//! @n �Ƃ������C�����\�ȃO���t�B�b�N�̃p�[�c����邽�߂̌^�����̐e�N���X���Ƃ������ƁD
//! @n �ЂƂ܂��C�p�����GraphicMainSample���m�F����΂Ȃ�ƂȂ��킩��Ǝv���D
class AbstractGraphicMain
{
public:

	//! @brief ���̃N���X�̌p����ł́CGraphicDataBroker�N���X�̃|�C���^�������Ɏ��R���X�g���N�^����������K�v������D
	AbstractGraphicMain(const GraphicDataBroker* broker);
	virtual ~AbstractGraphicMain() = default;


	//! @brief �`���ʂ̍X�V���s���D�������z�֐��̂��߁C�p����ł͕K��override����K�v������D
	//! @return bool ���[�v�𔲂��C�O���t�B�b�N�̕\�����I������Ȃ��false���������D
	virtual bool update() = 0;


	//! @brief �`����s���D�����ł͕`��n�̏����݂̂��s�������̃f�[�^���X�V���Ȃ�����const��t���Ă���D�������z�֐��D
	virtual void draw() const = 0;

protected:

	const GraphicDataBroker* const mp_broker;	//!< �摜�\�����s�����̃N���X�ƁC�f�[�^�������s���O���̃N���X���q������l�N���X�̃|�C���^���󂯎��D

};


//! @file abstract_graphic_main.h
//! @date 2023/08/09
//! @author ���J��
//! @brief AbstractGraphicMain�N���X�DAbstractGraphicMain�͒��ۃN���X�ł���̂Ŏ��̂����Ȃ��D
//! @n �s�� : @lineinfo
