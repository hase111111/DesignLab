#pragma once
#include "GraphicDataBroker.h"

//�`��̏������s���N���X�͕K�����̃N���X���p�����邱�ƁD���̃N���X�̓C���^�[�t�F�[�X(���S���z�֐��݂̂������̂𐶐��ł��Ȃ��N���X)�Ƃ����܂��D
//�����u�ˑ����̒����v�Ƃ�����ł��D�u�ˑ����̒����v�ɂ��Ă͐����������悭�킩���Ă��Ȃ��̂ŁC�������Ȃ��Ă����v���Ǝv���܂��D
//�p����̃N���X�ł�update��draw��override����K�v������܂��D
//�悭�킩��Ȃ��Ȃ�C�u�N���X�̌p���v�u�֐��̃I�[�o�[���C�h�i�I�[�o�[���[�h�Ƃ͕ʕ��ł��j�v�u���S���z�֐��v������𒲂ׂĂ݂Ă��������D�Ԃ����Ⴏ�C�悭�킩��Ȃ��Ă����v�ł��D
//�ЂƂ܂��CGraphicMainSample���m�F���Ă���������΂Ȃ�ƂȂ��킩��Ǝv���܂��D

class AbstractGraphicMain
{
public:
	//���̃N���X�̌p����ł́Cconst GraphicDataBroker* _broker�������Ɏ��R���X�g���N�^����������K�v������܂��D
	AbstractGraphicMain(const GraphicDataBroker* _broker);
	virtual ~AbstractGraphicMain() = default;

	//�`���ʂ̍X�V���s���D���S���z�֐��̂��߁C�p����ł͕K��override����K�v������D
	virtual bool update() = 0;

	//�`����s���D�����ł͕`��n�̏����݂̂��s�������̃f�[�^���X�V���Ȃ�����const��t���Ă���D���S���z�֐��D
	virtual void draw() const = 0;

protected:

	//�摜�\�����s�����̃N���X�ƁC�f�[�^�������s���O���̃N���X���q���N���X�̃|�C���^���󂯎��D
	const GraphicDataBroker* mp_Broker;
};