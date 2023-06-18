//! @file GraphicSystem.h
//! @brief Dxlib�̏������s���Ă����GraphicSystem�N���X���������Ă���D
//! @details Dxlib(�f���b�N�X ���C�u����)�̓E�B���h�E��\�����āC�����R�}���h���C���ɕ�����\�����邾���̎₵���v���O�����ɍʂ��^���Ă�����ł��D<br>
//! ��ɃQ�[���v���O���~���O������ۂɁC�E�B���h�E��\�����邽�߂̃��C�u�����Ƃ��Ďg�p����܂��D<br>
//! Dxlib�ȊO�ɂ� OpenCV�Ȃǂɂ��E�B���h�E��\������@�\������܂����C����̃v���O�����ł�Dxlib��p���Č��ʂ�\�����܂��D<br>
//! Dxlib�� WinAPI �Ƃ���Windows�̃A�v���P�[�V��������邽�߂̋@�\���C�g���₷�����Ă���郉�C�u�����ł��D<br> �ȉ��Q�l�y�[�W <br>
//! https://dixq.net/rp2/ ��C++�p�̎����D���X��������D<br> https://dixq.net/g/   ��C����p�̎����D���܂�Q�l�ɂȂ�Ȃ�����<br>
//! https://dxlib.xsrv.jp/dxfunc.html �������̊֐��̃��t�@�����X(�֐��̖ڎ��I�Ȃ���)�D<br>
//! @author ���J��

#pragma once
#include "GraphicDataBroker.h"

/**
 * @class GraphicSystem
 * @brief Dxlib�̏������s���N���X�D
 * @details Dxlib��񓯊������œ��������ƂŕʃX���b�h�ōs���Ă���O���t�T���̏��������ƂɃ��{�b�g�̏�Ԃ�\������D<br>
 * �������CDxlib�͔񓯊��������l�����Đ݌v����Ă��Ȃ��̂ŁC���������ɂ���Ă͂��܂����삵�܂���D<br>
 * ���̃v���W�F�N�g�ł͂��̊֐��̒��ł̂�Dxlib�̏����𓮂������ƂŁC�G���[��h���ł��܂����C�\�����ʃG���[����������\���͂���܂��D<br> <br> [Dxlib�̒���] <br>
 * ���ӂƂ��āCDxlib�n�̊֐��� �^�U��啶���� TRUE��FALSE���g���ĕ\���̂ŁC�]����true false���g�p���Ȃ��悤�ɂ��܂��傤�D<br>
 * �܂��C�Ԃ����Ⴏ�������̕��ł��������ǁC�o�[�W�����̍X�V�ɂ���ē����Ȃ��Ȃ�\��������̂�Dxlib�ɑg�ݍ��܂�Ă�����̂��g���̂�����ł��D<br>
 * �܂��CDxlib�̓G���[���o���Ƃ��� -1 ��Ԃ��֐������ɑ����ł��D���̂��ߗႦ�� if(DxLib_Init() == false) �Ə����Ă��G���[���󂯎��Ȃ����Ƃ�����܂��D<br> 
 * �������� if(DxLib_Init() < 0) �ƂȂ�܂��D����� bool�^ ���f�t�H���g�ő��݂��Ȃ�C����ł��g�p���邱�Ƃ��ł���悤�ɂ��邽�߂̔z���ł���CC++�ŏ�����Ă���{�R�[�h�ɂ����Ă�<br>
 * �����̌��ł�(��)�DDxlib�̃G���[��bool�ł͂Ȃ��Cint�^�̕��̒l�Ƃ������Ƃ��o���Ă����Ă��������D<br>
 * @author ���J��
 */
class GraphicSystem final
{
public:
	GraphicSystem() = default;
	~GraphicSystem() = default;

	/**
	 * GraphicSystem�N���X�̏�����������D�u���[�J�[(����l)�N���X�̃|�C���^���󂯎��.
	 * @param[in] _p_broker
	 */
	void init(const GraphicDataBroker* _p_broker);

	/**
	 * �E�B���h�E�̕\�����s���Ă����֐��ł��Dboost::thread�ɂ��̊֐���n���ĕ��񏈗����s���܂��D<br>init�Ɏ��s���Ă���C�܂���init���ĂԑO�Ɏ��s�������͑����ɏI�����܂��D<br>
	 * �܂������o�֐���dxlibInit�֐��Ɏ��s�����ꍇ���I�����܂��D<br>dxlib��2���ł��Ȃ��̂Ŏ��s����ꍇ�̓^�X�N�}�l�[�W���[����dxlib�𗎂Ƃ��Ă��������D
	 */
	void main();

private:

	bool dxlibInit();			//Dxlib�̏������������s���܂��D���s�����ꍇfalse��Ԃ��܂��D
	void dxlibFinalize() const;	//Dxlib�̏I���������s���܂��D

	const GraphicDataBroker* mp_Broker;	// �摜�\�����s�����̃N���X�ƁC�f�[�^�������s���O���̃N���X���q���u���[�J�[(����l)�N���X�̃|�C���^���󂯎�邽�߂̕ϐ��D
};
