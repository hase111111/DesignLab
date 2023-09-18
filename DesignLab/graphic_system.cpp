#include "graphic_system.h"

#include "DxLib.h"

#include "dxlib_util.h"
#include "graphic_const.h"
#include "keyboard.h"
#include "mouse.h"


void GraphicSystem::Init(std::unique_ptr<IGraphicMainBuilder>&& graphic_main_builder, std::shared_ptr<AbstractHexapodStateCalculator> calc,
	const GraphicDataBroker* const broker, const SApplicationSettingRecorder* const setting)
{
	if (graphic_main_builder && calc && broker != nullptr && setting != nullptr)
	{
		//GraphicMain���쐬����D
		graphic_main_ptr_ = graphic_main_builder->build(broker, calc, setting);
	}


	//�ݒ�t�@�C����ǂݍ��ށD
	setting_ptr_ = setting;


	//Fps��ݒ肷��D
	if (setting_ptr_ != nullptr)
	{
		fps_ptr_ = std::make_unique<FpsController>((*setting_ptr_).window_fps);
	}
}


void GraphicSystem::Main()
{
	//���������`�揈�����g��Ȃ��Ȃ�Α��I��
	if (!(*setting_ptr_).gui_display) { return; }

	//�ݒ�t�@�C����ǂݍ��߂Ă��Ȃ���ΏI��
	if (setting_ptr_ == nullptr) { return; }

	//GraphicMain���쐬����Ă��Ȃ���ΏI��
	if (!graphic_main_ptr_) { return; }

	// FpsController ���쐬����Ă��Ȃ���ΏI��
	if (!fps_ptr_) { return; }

	// Dxlib�̊֐��͕����X���b�h�ŌĂԂ��Ƃ��l������Ă��Ȃ��̂ŁC�����̃X���b�h����ĂԂƕK����肪�N���܂��D���̂��߁C�����������C�`��C�I�������̑S�Ă����̊֐��̒��ŌĂԕK�v������܂��D
	if (!DxlibInit(setting_ptr_)) { return; }


	// ProcessMessage�֐��̓E�B���h�E�́~�{�^�����������Ǝ��s�̒l��Ԃ��D�܂��C�E�B���h�E���ێ����邽�߂ɂ͒���I�ɌĂяo��������K�v������̂Ń��[�v�ŌĂё����Ă���D
	// ProcessMessage�͐�����0(C++�ɂ�����false)�C���s��-1(C++�ɂ�����true��0�ȊO�̒l)��Ԃ��C���̂��� !ProcessMessage �͂��̊֐��������̎��̂݃��[�v����...���̒ɂ������ł���D
	while (!ProcessMessage())
	{
		// ���C�����[�v�Cfalse���A�����ꍇ�C���[�v�𔲂���D
		if (!Loop())
		{
			break;
		}
	}

	//�I���������s���D
	DxlibFinalize();
}


bool GraphicSystem::DxlibInit(const SApplicationSettingRecorder* const setting)
{
	// 1���̏������p�֐���Dxlib_Init���ĂԑO�Ɏ��s����K�v������̂ł����Ŏ��s����D

	designlab::dxlib_util::InitDxlib3DSetting();		// 3D�֘A�̏��������s���D		

	SetOutApplicationLogValidFlag(FALSE);				// ���O�o�͖����ɕύX�D��������Ȃ���Log.txt�Ƃ����ז��ȃt�@�C�����o�͂����D
	SetMainWindowText(GraphicConst::WIN_NAME.c_str());	// �^�C�g����ύX�D�E�B���h�E�̍���ɕ\���������́D
	SetWindowSizeChangeEnableFlag(FALSE);               // �E�B���h�E�T�C�Y�����R�ɕύX�ł��Ȃ��悤�ɂ���D
	SetAlwaysRunFlag(TRUE);								// �E�C���h�E���A�N�e�B�u�ł͂Ȃ���Ԃł������𑱍s����悤�ɕύX����D
	ChangeWindowMode(TRUE);								// �E�C���h�E���[�h�ɕύX�D��������Ȃ��ƃt���X�N���[���ŕ\�������D

	//�E�B���h�E�̉����C�c���C�J���[��ݒ肷��D
	SetGraphMode((*setting).window_size_x, (*setting).window_size_y, GraphicConst::COLOR_BIT);

	//�c�w���C�u��������������
	if (DxLib_Init() < 0)
	{
		return false;
	}

	//�`���𗠉�ʂɂ���D������������C��ʂ̂�����������Ă������ʂ�����CDxlib���g���ȏ�K�{�̍��ځD
	SetDrawScreen(DX_SCREEN_BACK);

	// �w�i�F�̐ݒ�
	SetBackgroundColor(GraphicConst::BACK_COLOR_R, GraphicConst::BACK_COLOR_G, GraphicConst::BACK_COLOR_B);

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


	//�O���t�B�b�N���C���N���X����Ȃ�false��Ԃ��D
	if (!graphic_main_ptr_) { return false; }


	//�W���o�͂�����
	clsDx();

	//�L�[���͂��X�V����D
	Keyboard::GetIns()->Update();
	Mouse::GetIns()->Update();

	//�������s��
	if (!graphic_main_ptr_->Update()) { return false; }


	//�`�悷��
	if (!fps_ptr_->SkipDrawScene())
	{
		//����ʂɕ`�悵���G������
		if (ClearDrawScreen() < 0) { return false; }

		graphic_main_ptr_->Draw();

		//�X�N���[���ɗ���ʂɕ`�悵�����e���ڂ�
		if (ScreenFlip() < 0) { return false; }

	}

	//FPS�����ɕۂ��߂ɑ҂D
	fps_ptr_->Wait();

	return true;
}


void GraphicSystem::DxlibFinalize() const
{
	// DX���C�u�����̏I���������Ă�.
	DxLib_End();
}
