#include "graphic_system.h"

#include "DxLib.h"

#include "graphic_const.h"
#include "mouse.h"
#include "keyboard.h"
#include "designlab_dxlib.h"
#include "Define.h"


void GraphicSystem::init(std::unique_ptr<IGraphicMainBuilder>&& graphic_main_builder, const GraphicDataBroker* const broker, const SApplicationSettingRecorder* const setting)
{
	if (graphic_main_builder && broker != nullptr && setting != nullptr)
	{
		//GraphicMain���쐬����D
		mp_graphic_main = graphic_main_builder->build(broker, setting);
	}


	//�ݒ�t�@�C����ǂݍ��ށD
	mp_setting = setting;


	//Fps��ݒ肷��D
	mp_fps = std::make_unique<Fps>((*mp_setting).window_fps);
}


void GraphicSystem::main()
{
	//���������`�揈�����g��Ȃ��Ȃ�Α��I��
	if (!(*mp_setting).gui_display) { return; }

	//�ݒ�t�@�C����ǂݍ��߂Ă��Ȃ���ΏI��
	if (mp_setting == nullptr) { return; }

	//GraphicMain���쐬����Ă��Ȃ���ΏI��
	if (!mp_graphic_main) { return; }

	// Fps���쐬����Ă��Ȃ���ΏI��
	if (!mp_fps) { return; }

	// Dxlib�̊֐��͕����X���b�h�ŌĂԂ��Ƃ��l������Ă��Ȃ��̂ŁC�����̃X���b�h����ĂԂƕK����肪�N���܂��D���̂��߁C�����������C�`��C�I�������̑S�Ă����̊֐��̒��ŌĂԕK�v������܂��D
	if (!dxlibInit(mp_setting)) { return; }


	// ProcessMessage�֐��̓E�B���h�E�́~�{�^�����������Ǝ��s�̒l��Ԃ��D�܂��C�E�B���h�E���ێ����邽�߂ɂ͒���I�ɌĂяo��������K�v������̂Ń��[�v�ŌĂё����Ă���D
	// ProcessMessage�͐�����0(C++�ɂ�����false)�C���s��-1(C++�ɂ�����true��0�ȊO�̒l)��Ԃ��C���̂��� !ProcessMessage �͂��̊֐��������̎��̂݃��[�v����...���̒ɂ������ł���D
	while (!ProcessMessage())
	{
		//false���A�����ꍇ�C���[�v�𔲂���D
		if (!loop())
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


bool GraphicSystem::loop()
{
	// [�`��̏����ɂ���]
	// ScreenFlip�֐���ClearDrawScreen�֐��̏ڍׁF�E�B���h�E�̉摜�\���̓p���p������̗l�ɉ�ʂ�f�����؂�ւ��邱�ƂŃA�j���[�V�������Č����Ă���D
	// �������C�P�ɉ�ʂ�؂�ւ����ꍇ�C�{���̃p���p������̗l�ɃE�B���h�E�ɂ�������łĂ��܂��D
	// ������GraphicSystem�N���X��dxlibInit�֐��̒��ŌĂ΂�Ă��� SetDrawScreen(DX_SCREEN_BACK) �ɂ���Ă������񗠉�ʂɊG��`�悵�Ă���C
	// ScreenFlip�֐��ŃE�B���h�E�ɊG��߂����Ƃŉ�ʂ̂�������Ȃ����Ă���D
	// ClearDrawScreen �� ScreenFlip �� ProcessMessage�ƕԂ��l�������Ȃ̂ŁCloop�֐��̗l�ȏ������ƂȂ�D


	//�O���t�B�b�N���C���N���X����Ȃ�false��Ԃ��D
	if (!mp_graphic_main) { return false; }


	//�W���o�͂�����
	clsDx();

	//�L�[���͂��X�V����D
	Keyboard::getIns()->update();
	Mouse::getIns()->update();

	//�������s��
	if (!mp_graphic_main->update()) { return false; }


	//�`�悷��
	if (!mp_fps->skipDrawScene())
	{
		//����ʂɕ`�悵���G������
		if (ClearDrawScreen() < 0) { return false; }

		mp_graphic_main->draw();

		//�X�N���[���ɗ���ʂɕ`�悵�����e���ڂ�
		if (ScreenFlip() < 0) { return false; }

	}

	//FPS�����ɕۂ��߂ɑ҂D
	mp_fps->wait();

	return true;
}


void GraphicSystem::dxlibFinalize() const
{
	// DX���C�u�����̏I���������Ă�.
	DxLib_End();
}
