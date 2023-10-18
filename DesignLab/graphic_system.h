//! @file graphic_system.h
//! @brief Dxlib�̏������s���N���X�D
//! @details Dxlib(�f���b�N�X ���C�u����)�̓E�B���h�E��\�����āC
//! @n �����R�}���h���C���ɕ�����\�����邾���̎₵���v���O�����ɍʂ��^���Ă�����D
//! @n ��ɃQ�[���v���O���~���O������ۂɁC�E�B���h�E��\�����邽�߂̃��C�u�����Ƃ��Ďg�p�����D
//! @n Dxlib�ȊO�ɂ� OpenCV�Ȃǂɂ��E�B���h�E��\������@�\�����邪�C����̃v���O�����ł�Dxlib��p���Č��ʂ�\������D
//! @n Dxlib�� Windows API �Ƃ�����Windows�̃A�v���P�[�V��������邽�߂̋@�\���C�g���₷�����Ă���郉�C�u�����ł���D���Ԃ�
//! @n  
//! @n �ȉ��Q�l�y�[�W 
//! @n
//! @n �Ehttps://dixq.net/rp2/ ��C++�p�̎����D���X��� 
//! @n �Ehttps://dixq.net/g/   ��C����p�̎����D���܂�Q�l�ɂȂ�Ȃ����� 
//! @n �Ehttps://dxlib.xsrv.jp/dxfunc.html �������̊֐��̃��t�@�����X(�֐��̖ڎ�)�D


#ifndef DESIGNLAB_GRAPHIC_SYSTEM_H_
#define DESIGNLAB_GRAPHIC_SYSTEM_H_

#include <memory>

#include <boost/thread.hpp>

#include "define.h"

#ifndef DESIGNLAB_DONOT_USE_DXLIB

#include "abstract_hexapod_state_calculator.h"
#include "application_setting_recorder.h"
#include "fps_controller.h"
#include "graphic_data_broker.h"
#include "interface_graphic_main.h"


//! @class GraphicSystem
//! @brief Dxlib�̏������s���N���X�D
//! @details Dxlib��񓯊������œ��������ƂŕʃX���b�h�ōs���Ă���O���t�T���̏��������ƂɃ��{�b�g�̏�Ԃ�\������D
//! @n �������CDxlib�͔񓯊��������l�����Đ݌v����Ă��Ȃ��̂ŁC���������ɂ���Ă͂��܂����삵�Ȃ��D
//! @n ���̃v���W�F�N�g�ł͂��̊֐��̒��ł̂�Dxlib�̏����𓮂������ƂŁC�G���[��h���ł��邪�C�\�����ʃG���[����������\��������D
//! @n 
//! @n [Dxlib�̒���] 
//! @n ���ӂƂ��āCDxlib�n�̊֐��� �^�U��啶���� TRUE��FALSE���g���ĕ\���̂ŁC�]����true false���g�p���Ȃ��悤�ɂ��邱�ƁD
//! @n (���͏������̕��ł��������ǁC�o�[�W�����̍X�V�ɂ���ē����Ȃ��Ȃ�\��������̂�Dxlib�ɑg�ݍ��܂�Ă�����̂��g���̂�����D)
//! @n �܂��CDxlib�̓G���[���o���Ƃ��� - 1 ��Ԃ��֐������ɑ����D���̂��ߗႦ�� if (DxLib_Init() == false) �Ə����Ă�
//! @n �G���[���󂯎��Ȃ����Ƃ�����D
//! @n �������� if (DxLib_Init() < 0) �ƂȂ�D
//! @n ����� bool�^ ���f�t�H���g�ő��݂��Ȃ�C����ł��g�p���邱�Ƃ��ł���悤�ɂ��邽�߂̔z���ł���C
//! @n C++�ŏ�����Ă���{�R�[�h�ɂ����Ă͍����̌��ł����Ȃ�(��)�D
//! @n Dxlib�̃G���[��bool�ł͂Ȃ��Cint�^�̕��̒l�Ƃ������Ƃ��o���Ă������ƁD
//! @n 
//! @n �܂��CDxlib��2���ł��Ȃ��̂ŁC���s�Ɏ��s����ꍇ�̓^�X�N�}�l�[�W���[����dxlib�𗎂Ƃ��Ă��������D
class GraphicSystem final
{
public:

	//! @param [in] graphic_main_ptr GraphicMain�N���X�̃|�C���^�D
	//! @param [in] setting_ptr �A�v���P�[�V�����̐ݒ���L�^����N���X�̃|�C���^�D
	GraphicSystem(const std::shared_ptr<const ApplicationSettingRecorder> setting_ptr);


	//! @brief �E�B���h�E�̕\�����s���Ă����֐��ł��Dboost::thread�ɂ��̊֐���n���ĕ��񏈗����s���D
	//! @n �����o�֐���MyDxlibInit�֐��Ɏ��s�����ꍇ�C�I������D
	void Main();

	//! @brief �O���t�B�b�N�̕\�����s���N���X��ύX����D
	//! @param [in] graphic_main_ptr GraphicMain�N���X�̃��j�[�N�|�C���^�D
	void ChangeGraphicMain(std::unique_ptr<IGraphicMain>&& graphic_main_ptr);

private:

	//! @brief Dxlib�̏������������s���D
	//! @n ���������b�p���Ď��삷��ꍇ��My�𓪂ɂ���Ɨǂ��炵���D
	//! @return bool �������ɐ����������ǂ����D
	bool MyDxlibInit();

	//! @brief GraphicSystem�N���X��while���[�v�̒��Ŗ��t���[���Ă΂�鏈��
	//! @return bool ���[�v�𑱂��邩�ǂ����Dfalse�Ȃ�΃��[�v�𔲂���D�ُ킪�N�����ꍇ��fase��Ԃ��D
	bool Loop();

	//! @brief Dxlib�̏I���������s���D
	void MyDxlibFinalize() const;


	std::unique_ptr<IGraphicMain> graphic_main_ptr_;	//!< �O���t�B�b�N�̕\�����s���N���X�̃|�C���^�D

	const std::shared_ptr<const ApplicationSettingRecorder> setting_ptr_;	//!< �ݒ��ۑ�����\���̂̃|�C���^�D

	FpsController fps_controller_;		//!< FPS�����ɐ��䂷��N���X�D�ڂ�����fps_controller.h��

	boost::mutex mutex_;	//!< �����̊֐�����񓯊��I�ɁC�����ɃA�N�Z�X����Ɗ댯�Ȃ̂ŁC�����h�����߂�mutex��p���Ĕr��������s���D
};

#endif	// DESIGNLAB_DONOT_USE_DXLIB

#endif	// DESIGNLAB_GRAPHIC_SYSTEM_H_