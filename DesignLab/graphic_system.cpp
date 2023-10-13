#include "graphic_system.h"

#include <Dxlib.h>

#include "cassert_define.h"
#include "dxlib_util.h"
#include "graphic_const.h"
#include "keyboard.h"
#include "mouse.h"


GraphicSystem::GraphicSystem(std::unique_ptr<IGraphicMain>&& graphic_main_ptr, const std::shared_ptr<const ApplicationSettingRecorder> setting_ptr) :
	graphic_main_ptr_(std::move(graphic_main_ptr)),
	setting_ptr_(setting_ptr),
	fps_controller_(setting_ptr ? setting_ptr->window_fps : 60)		// setting_ptr �� null ���ǂ����𒲂ׁC�����łȂ����window_fps�̒l�����o���D
{
}


void GraphicSystem::Main()
{
	//�ݒ�t�@�C����ǂݍ��߂Ă��Ȃ���ΏI��
	if (!setting_ptr_) 
	{
		// assert�͈�����false�̎��ɃG���[���o���C�ǂ��ŃG���[���o�����o�͂���Dcassert_define.h�ŗL��������Ă���Ύ��s�����D
		// ���̊֐��̓f�o�b�O���̂ݗL���ŁC�����[�X���ɂ͖����ɂȂ�Ƃ������ƁD
		// �f�o�b�O���̓G���[��f���C�����[�X���͂ЂƂ܂�����͂��Ă����Ƃ����`�ɂ��������߁C���̂悤�ɂ��Ă���D
		assert(false);
		return; 
	}

	//���������`�揈�����g��Ȃ��Ȃ�Α��I��
	if (!setting_ptr_->gui_display) 
	{
		return; 
	}

	//GraphicMain���쐬����Ă��Ȃ���ΏI��
	if (!graphic_main_ptr_) 
	{
		assert(false);
		return; 
	}

	// Dxlib�̊֐��͕����X���b�h�ŌĂԂ��Ƃ��l������Ă��Ȃ��̂ŁC�����̃X���b�h����ĂԂƕK����肪�N����D���̂��߁C�����������C�`��C�I�������̑S�Ă����̊֐��̒��ŌĂԕK�v������
	if (!MyDxlibInit()) 
	{
		assert(false);
		return; 
	}

	// ProcessMessage�֐��̓E�B���h�E�́~�{�^�����������Ǝ��s�̒l��Ԃ��D
	// �܂��C�E�B���h�E���ێ����邽�߂ɂ͒���I�ɌĂяo��������K�v������̂Ń��[�v�ŌĂё����Ă���D
	// ProcessMessage�͐�����0(C++�ɂ�����false)�C���s��-1(C++�ɂ�����true��0�ȊO�̒l)��Ԃ��C���̂��߁C���s����܂Ń��[�v����ꍇ�͈ȉ��̂悤�ɋL�q����
	while (ProcessMessage() >= 0)
	{
		// ���C�����[�v�Cfalse���A�����ꍇ�C���[�v�𔲂���D
		if (!Loop())
		{
			break;
		}
	}

	//�I���������s���D
	MyDxlibFinalize();
}


bool GraphicSystem::MyDxlibInit()
{
	// 1���̏������p�֐���Dxlib_Init���ĂԑO�Ɏ��s����K�v������̂ł����Ŏ��s����D	

	SetOutApplicationLogValidFlag(FALSE);					// ���O�o�͖����ɕύX�D��������Ȃ���Log.txt�Ƃ����ז��ȃt�@�C�����o�͂����D
	SetMainWindowText(GraphicConst::kWindowName.c_str());	// �^�C�g����ύX�D�E�B���h�E�̍���ɕ\���������́D
	SetWindowSizeChangeEnableFlag(FALSE);					// �E�B���h�E�T�C�Y�����R�ɕύX�ł��Ȃ��悤�ɂ���D
	SetAlwaysRunFlag(TRUE);									// �E�C���h�E���A�N�e�B�u�ł͂Ȃ���Ԃł������𑱍s����悤�ɕύX����D
	SetWaitVSyncFlag(FALSE);								// ���������M����҂��Ȃ��悤�ɕύX�D��������Ȃ���FPS��60�Œ�ɂȂ�D	
	ChangeWindowMode(TRUE);									// �E�C���h�E���[�h�ɕύX�D��������Ȃ��ƃt���X�N���[���ŕ\�������D
	SetUseDirectInputFlag(TRUE);							// DirectInput���g�p����悤�ɕύX�D��������Ȃ��ƃ}�E�X���͂ŃT�C�h�{�^�����󂯕t�����Ȃ��D
	SetDxLibEndPostQuitMessageFlag(FALSE);					// DxLib_End�֐����Ăяo�����ۂ� PostQuitMessage ���Ă΂Ȃ��悤�ɂ���(���x��GUI�𗧂��グ����悤�ɂ��邽��)�D

	//�E�B���h�E�̉����C�c���C�J���[��ݒ肷��D
	SetGraphMode(setting_ptr_->window_size_x, setting_ptr_->window_size_y, GraphicConst::kColorBit);

	//�c�w���C�u��������������
	if (DxLib_Init() < 0)
	{
		return false;
	}

	//�`���𗠉�ʂɂ���D������������C��ʂ̂�����������Ă������ʂ�����CDxlib���g���ȏ�K�{�̍��ځD
	SetDrawScreen(DX_SCREEN_BACK);

	// �w�i�F�̐ݒ�
	SetBackgroundColor(GraphicConst::kBackColorRed, GraphicConst::kBackColorGreen, GraphicConst::kBackColorBlue);

	// 3D�֘A�̏��������s���D	
	designlab::dxlib_util::InitDxlib3DSetting();	

	return true;
}


bool GraphicSystem::Loop()
{
	// [�`��̏����ɂ���]
	// ScreenFlip�֐���ClearDrawScreen�֐��̏ڍׁF�E�B���h�E�̉摜�\���̓p���p������̗l�ɉ�ʂ�f�����؂�ւ��邱�ƂŃA�j���[�V�������Č����Ă���D
	// �������C�P�ɉ�ʂ�؂�ւ����ꍇ�C�{���̃p���p������̗l�ɃE�B���h�E�ɂ�������łĂ��܂��D
	// ������GraphicSystem�N���X��dxlibInit�֐��̒��ŌĂ΂�Ă��� SetDrawScreen(DX_SCREEN_BACK) �ɂ���Ă������񗠉�ʂɊG��`�悵�Ă���C
	// ScreenFlip�֐��ŃE�B���h�E�ɊG��߂����Ƃŉ�ʂ̂�������Ȃ����Ă���D
	// ClearDrawScreen �� ScreenFlip �� ProcessMessage�ƕԂ��l�������Ȃ̂ŁCloop�֐��̗l�ȏ������ƂȂ�D


	// �O���t�B�b�N���C���N���X����Ȃ�false��Ԃ��D
	if (!graphic_main_ptr_) { return false; }


	// GUI��ʂւ̕W���o�͂����Z�b�g����
	clsDx();

	// �L�[���͂��X�V����D
	Keyboard::GetIns()->Update();
	Mouse::GetIns()->Update();

	// �������s��
	if (!graphic_main_ptr_->Update()) { return false; }


	// �`�悷��
	if (!fps_controller_.SkipDrawScene())
	{
		// ����ʂɕ`�悵���G������
		if (ClearDrawScreen() < 0) { return false; }

		graphic_main_ptr_->Draw();

		// �X�N���[���ɗ���ʂɕ`�悵�����e���ڂ�
		if (ScreenFlip() < 0) { return false; }

	}

	// FPS�����ɕۂ��߂ɑ҂D
	fps_controller_.Wait();

	return true;
}


void GraphicSystem::MyDxlibFinalize() const
{
	// DX���C�u�����̏I���������Ă�.
	DxLib_End();

	//�ق��ɂ�����������΂����ɒǋL����
}
