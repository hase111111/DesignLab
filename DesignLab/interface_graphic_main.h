#pragma once

#include "graphic_data_broker.h"


//! @class IGraphicMain
//! @date 2023/08/09
//! @author ���J��
//! @brief GraphicMain�̃C���^�[�t�F�[�X
//! @details �`��̏������s���N���X�͕K�����̃N���X���p�����邱�ƁD�t�Ɍ����Όp������Ύ��R�ɏ����������\�ɂȂ�D@n 
//! @n [�C���^�[�t�F�C�X�ɂ���] 
//! @n ���̃N���X�̓C���^�[�t�F�[�X(�������z�֐��݂̂������̂𐶐��ł��Ȃ��N���X)�Ƃ�����D
//! @n �����u�ˑ����̒����v�Ƃ�����DGraphicLoop��GraphicMain�̏����Ɉˑ����Ă��܂����߁C������C���^�[�t�F�C�X�ɂ��邱�Ƃ�GraphicLoop��GraphicMain��
//! @n �C���^�[�t�F�C�X(update�֐��̖߂�l)�ɂ̂݊֐S�����悤�ɂȂ�C�ˑ����������ł���悤�ɂȂ�D
//! @n �܂��u�ˑ����̒����v�ɂ��Ă͐����������悭�킩���Ă��Ȃ��̂ŁC�������Ȃ��Ă����v�D
//! @n �Q�l: https://qiita.com/okazuki/items/a0f2fb0a63ca88340ff6 
//! @n https://qiita.com/okazuki/items/0c17a161a921847cd080 
//! @n �Ƃ������C�����\�ȃO���t�B�b�N�̃p�[�c����邽�߂̌^�����̐e�N���X���Ƃ������ƁD@n
//! @n �p����̃N���X�ł�update��draw��override����K�v������D
//! @n �悭�킩��Ȃ��Ȃ�C�u�N���X�̌p���v�u�֐��̃I�[�o�[���C�h�i�I�[�o�[���[�h�Ƃ͕ʕ��j�v�u���S���z�֐��v������𒲂ׂĂ݂�Ƃ悢�Ǝv���D�Ԃ����Ⴏ�C�悭�킩��Ȃ��Ă����v�D
//! @n �ЂƂ܂��C�p�����GraphicMainSample���m�F����΂Ȃ�ƂȂ��킩��Ǝv���D
class IGraphicMain
{
public:

	//! @brief ���̃N���X�̌p����ł́CGraphicDataBroker�N���X�̃|�C���^�������Ɏ��R���X�g���N�^����������K�v������D
	IGraphicMain(const GraphicDataBroker* broker);
	virtual ~IGraphicMain() = default;

	//! @brief �`���ʂ̍X�V���s���D�������z�֐��̂��߁C�p����ł͕K��override����K�v������D
	//! @return bool ���[�v�𔲂��C�O���t�B�b�N�̕\�����I������Ȃ��false���������D
	virtual bool update() = 0;

	//! @brief �`����s���D�����ł͕`��n�̏����݂̂��s�������̃f�[�^���X�V���Ȃ�����const��t���Ă���D�������z�֐��D
	virtual void draw() const = 0;

protected:

	const GraphicDataBroker* const mp_broker;	//!< �摜�\�����s�����̃N���X�ƁC�f�[�^�������s���O���̃N���X���q������l�N���X�̃|�C���^���󂯎��D

};


//! @file InterfaceGraphicMain.h
//! @date 2023/08/09
//! @author ���J��
//! @brief IGraphicMain�N���X�DIGraphicMain�̓C���^�[�t�F�C�X�ł���̂Ŏ��Ԃ����Ȃ��D
