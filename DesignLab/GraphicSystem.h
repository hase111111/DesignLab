#pragma once
#include "GraphicDataBroker.h"

// GraphicMain�� Dxlib�̏������s���Ă����N���X�ł��D
// Dxlib(�f���b�N�X ���C�u����)�̓E�B���h�E��\�����āC�����R�}���h���C���ɕ�����\�����邾���̎₵���v���O�����ɍʂ��^���Ă�����ł��D
// ��ɃQ�[���v���O���~���O������ۂɁC�E�B���h�E��\�����邽�߂̃��C�u�����Ƃ��Ďg�p����܂��D
// Dxlib�ȊO�ɂ� OpenCV�Ȃǂɂ��E�B���h�E��\������@�\������܂����C����̃v���O�����ł�Dxlib��p���Č��ʂ�\�����܂��D
// Dxlib�� WinAPI �Ƃ���Windows�̃A�v���P�[�V��������邽�߂̋@�\���C�g���₷�����Ă���郉�C�u�����ł��D
// �ȉ��Q�l�y�[�W
// https://dixq.net/rp2/ ��C++�p�̎����D���X��������D
// https://dixq.net/g/   ��C����p�̎����D���܂�Q�l�ɂȂ�Ȃ�����
// https://dxlib.xsrv.jp/dxfunc.html �������̊֐��̃��t�@�����X(�ڎ��I�Ȃ���)�D

class GraphicSystem final
{
public:
	GraphicSystem() = default;
	~GraphicSystem() = default;

	//Dxlib�̏��������s���܂��D���s�����false��Ԃ��܂��D
	bool init(const GraphicDataBroker* _p_broker);

	// �E�B���h�E�̕\�����s���Ă����֐��ł��Dboost::thread�ɂ��̊֐���n���ĕ��񏈗����s���܂��Dinit�Ɏ��s���Ă���C�܂���init���ĂԑO�Ɏ��s�������͑����ɏI�����܂��D
	void main();

private:

	void finalize() const;	//Dxlib�̏I���������s���܂��D

	bool m_is_init_success = false;

	//�摜�\�����s�����̃N���X�ƁC�f�[�^�������s���O���̃N���X���q���u���[�J�[�N���X�̃|�C���^���󂯎��D
	const GraphicDataBroker* mp_Broker;
};
