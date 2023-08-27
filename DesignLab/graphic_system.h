#pragma once

#include <memory>

#include "abstract_graphic_main.h"
#include "application_setting_recorder.h"


class GraphicSystem final
{
public:
	GraphicSystem() = default;
	~GraphicSystem() = default;


	//! @brief GraphicSystem�N���X�̏�����������D�u���[�J�[(����l)�N���X�̃|�C���^�������GraphicMain�N���X���󂯎��.
	//! @param [in] graphic_main GraphicSystem�N���X�̃����o�֐����Ăяo�����߂̃u���[�J�[�N���X�̃|�C���^�D
	void init(std::unique_ptr<AbstractGraphicMain>&& graphic_main, const SApplicationSettingRecorder* const setting);


	//! @brief �E�B���h�E�̕\�����s���Ă����֐��ł��Dboost::thread�ɂ��̊֐���n���ĕ��񏈗����s���܂��D<br>init�Ɏ��s���Ă���C�܂���init���ĂԑO�Ɏ��s�������͑����ɏI�����܂��D<br>
	//! �܂������o�֐���dxlibInit�֐��Ɏ��s�����ꍇ���I�����܂��D<br>dxlib��2���ł��Ȃ��̂Ŏ��s����ꍇ�̓^�X�N�}�l�[�W���[����dxlib�𗎂Ƃ��Ă��������D
	void main();

private:

	bool dxlibInit(const SApplicationSettingRecorder* const setting);			//Dxlib�̏������������s���܂��D���s�����ꍇfalse��Ԃ��܂��D
	void dxlibFinalize() const;	//Dxlib�̏I���������s���܂��D

	std::unique_ptr<AbstractGraphicMain> mp_graphic_main;	// �O���t�B�b�N�̕\�����s���N���X�̃|�C���^�D
	const SApplicationSettingRecorder* mp_setting = nullptr;		// �ݒ��ۑ�����N���X�̃|�C���^�D
};


//! @class GraphicSystem
//! @date 2023/08/08
//! @author ���J��
//! @brief Dxlib�̏������s���N���X�D
//! @details Dxlib��񓯊������œ��������ƂŕʃX���b�h�ōs���Ă���O���t�T���̏��������ƂɃ��{�b�g�̏�Ԃ�\������D
//! @n �������CDxlib�͔񓯊��������l�����Đ݌v����Ă��Ȃ��̂ŁC���������ɂ���Ă͂��܂����삵�Ȃ��D
//! @n ���̃v���W�F�N�g�ł͂��̊֐��̒��ł̂�Dxlib�̏����𓮂������ƂŁC�G���[��h���ł��邪�C�\�����ʃG���[����������\��������D@n @n [Dxlib�̒���] 
//! @n ���ӂƂ��āCDxlib�n�̊֐��� �^�U��啶���� TRUE��FALSE���g���ĕ\���̂ŁC�]����true false���g�p���Ȃ��悤�ɂ��邱�ƁD
//! @n (���͏������̕��ł��������ǁC�o�[�W�����̍X�V�ɂ���ē����Ȃ��Ȃ�\��������̂�Dxlib�ɑg�ݍ��܂�Ă�����̂��g���̂�����D)
//! @n �܂��CDxlib�̓G���[���o���Ƃ��� - 1 ��Ԃ��֐������ɑ����D���̂��ߗႦ�� if (DxLib_Init() == false) �Ə����Ă��G���[���󂯎��Ȃ����Ƃ�����D
//! @n �������� if (DxLib_Init() < 0) �ƂȂ�D����� bool�^ ���f�t�H���g�ő��݂��Ȃ�C����ł��g�p���邱�Ƃ��ł���悤�ɂ��邽�߂̔z���ł���CC++�ŏ�����Ă���{�R�[�h�ɂ����Ă�
//! @n �����̌��ł����Ȃ�(��)�DDxlib�̃G���[��bool�ł͂Ȃ��Cint�^�̕��̒l�Ƃ������Ƃ��o���Ă������ƁD


//! @file graphic_system.h
//! @date 2023/08/08
//! @author ���J��
//! @brief Dxlib�̏������s��GraphicSystem�N���X�D
//! @n �s�� : @lineinfo
//! @details Dxlib(�f���b�N�X ���C�u����)�̓E�B���h�E��\�����āC�����R�}���h���C���ɕ�����\�����邾���̎₵���v���O�����ɍʂ��^���Ă�����D
//! @n ��ɃQ�[���v���O���~���O������ۂɁC�E�B���h�E��\�����邽�߂̃��C�u�����Ƃ��Ďg�p�����D
//! @n Dxlib�ȊO�ɂ� OpenCV�Ȃǂɂ��E�B���h�E��\������@�\�����邪�C����̃v���O�����ł�Dxlib��p���Č��ʂ�\������D
//! @n Dxlib�� WinAPI �Ƃ���Windows�̃A�v���P�[�V��������邽�߂̋@�\���C�g���₷�����Ă���郉�C�u�����ł���D@n  @n �ȉ��Q�l�y�[�W @n
//! @n �Ehttps://dixq.net/rp2/ ��C++�p�̎����D���X��� @n �Ehttps://dixq.net/g/   ��C����p�̎����D���܂�Q�l�ɂȂ�Ȃ����� 
//! @n �Ehttps://dxlib.xsrv.jp/dxfunc.html �������̊֐��̃��t�@�����X(�֐��̖ڎ�)�D
