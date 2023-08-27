#include "graphic_system.h"

#include "DxLib.h"

#include "graphic_const.h"
#include "graphic_loop.h"
#include "graphic_main_sample.h"
#include "graphic_main_basic.h"
#include "graphic_main_test.h"
#include "graphic_main_graph_viewer.h"
#include "designlab_dxlib.h"
#include "Define.h"


void GraphicSystem::init(std::unique_ptr<AbstractGraphicMain>&& graphic_main, const SApplicationSettingRecorder* const setting)
{
	mp_graphic_main = std::move(graphic_main);
	mp_setting = setting;
}


void GraphicSystem::main()
{
	//�ݒ�t�@�C����ǂݍ��߂Ă��Ȃ���ΏI��
	if (mp_setting == nullptr) { return; }

	//���������`�揈�����g��Ȃ��Ȃ�Α��I��
	if (!(*mp_setting).gui_display) { return; }

	//�u���[�J�[��null(���݂��Ȃ�)�Ȃ�I��
	if (!mp_graphic_main) { return; }

	// Dxlib�̊֐��͕����X���b�h�ŌĂԂ��Ƃ��l������Ă��Ȃ��̂ŁC�����̃X���b�h����ĂԂƕK����肪�N���܂��D���̂��߁C�����������C�`��C�I�������̑S�Ă����̊֐��̒��ŌĂԕK�v������܂��D
	if (!dxlibInit(mp_setting)) { return; }


	//�`��̏������s���N���X���Z�b�g����D���s����`��̓��e��ύX�������Ȃ�΁C���̂悤��IGraphicMain���p���������̃N���X��<>�ɓ���Ă��������D
	GraphicLoop looper(std::move(mp_graphic_main));


	// ProcessMessage�֐��̓E�B���h�E�́~�{�^�����������Ǝ��s�̒l��Ԃ��D�܂��C�E�B���h�E���ێ����邽�߂ɂ͒���I�ɌĂяo��������K�v������̂Ń��[�v�ŌĂё����Ă���D
	// ProcessMessage�͐�����0(C++�ɂ�����false)�C���s��-1(C++�ɂ�����true��0�ȊO�̒l)��Ԃ��C���̂��� !ProcessMessage �͂��̊֐��������̎��̂݃��[�v����...���̒ɂ������ł���D
	while (!ProcessMessage())
	{
		//false���A�����ꍇ�C���[�v�𔲂���D
		if (!looper.loop())
		{
			break;
		}
	}

	//�I���������s���D
	dxlibFinalize();
}


bool GraphicSystem::dxlibInit(const SApplicationSettingRecorder* const setting)
{
	// 1���̏������p�֐���Dxlib_Init���ĂԑO�Ɏ��s����K�v������̂ł����Ŏ��s���܂��D

	dl_dxlib::initDxlib3D();							//3D�֘A�̏��������s���D		

	SetOutApplicationLogValidFlag(FALSE);				//���O�o�͖����ɕύX�D��������Ȃ���Log.txt�Ƃ����ז��ȃt�@�C�����o�͂���܂��D
	SetMainWindowText(GraphicConst::WIN_NAME.c_str());	//�^�C�g����ύX�D�E�B���h�E�̍���ɕ\���������̂ł��D
	SetWindowSizeChangeEnableFlag(FALSE);               //�E�B���h�E�T�C�Y�����R�ɕύX�ł��Ȃ��悤�ɂ���D
	SetAlwaysRunFlag(TRUE);								//�E�C���h�E���A�N�e�B�u�ł͂Ȃ���Ԃł������𑱍s����悤�ɕύX�D
	ChangeWindowMode(TRUE);								//�E�C���h�E���[�h�ɕύX�D��������Ȃ��ƃt���X�N���[���ŕ\������܂��D

	//�E�B���h�E�̉����C�c���C�J���[��ݒ肵�܂��D
	SetGraphMode((*setting).window_size_x, (*setting).window_size_y, GraphicConst::COLOR_BIT);

	//�c�w���C�u��������������
	if (DxLib_Init() < 0)
	{
		return false;
	}

	//�`���𗠉�ʂɂ���D����������̂ł����C��ʂ̂�����������Ă������ʂ�����CDxlib���g���ȏ�K�{�̍��ڂł��D
	SetDrawScreen(DX_SCREEN_BACK);

	// �w�i�F�̐ݒ�
	SetBackgroundColor(GraphicConst::BACK_COLOR_R, GraphicConst::BACK_COLOR_G, GraphicConst::BACK_COLOR_B);

	return true;
}


void GraphicSystem::dxlibFinalize() const
{
	// DX���C�u�����̏I���������Ă�.
	DxLib_End();
}
