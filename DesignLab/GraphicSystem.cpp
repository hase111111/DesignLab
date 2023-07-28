#include "GraphicSystem.h"
#include "DxLib.h"
#include "GraphicConst.h"
#include "GraphicLoop.h"
#include "GraphicMainSample.h"
#include "GraphicMainBasic.h"
#include "GraphicMainTest.h"
#include "GraphicMainGraphViewer.h"
#include "Define.h"


void GraphicSystem::init(std::unique_ptr<IGraphicMain>&& _graphic_main)
{
	mp_InterfaceGraphicMain = std::move(_graphic_main);
}

void GraphicSystem::main()
{
	//���������`�揈�����g��Ȃ��Ȃ�Α��I��
	if (Define::FLAG_GRAPHIC_AVAILABLE == false)
	{
		std::cout << "�摜�\�����s��Ȃ��ݒ�ɂȂ��Ă��邽�߁C�摜�\�����I�����܂��DDefine�N���X�̕ϐ��FFLAG_GRAPHIC_AVAILABLE�ŕύX�ł��܂��D" << std::endl;	// cout�֐��̓X���b�h�Z�[�t�ł͂Ȃ��̂Ŗ{���͌ĂԂׂ��ł͂Ȃ��D
		return;
	}

	//�u���[�J�[��null(���݂��Ȃ�)�Ȃ�I��
	if (!mp_InterfaceGraphicMain)
	{
		std::cout << "IGraphicMain�N���X�̎󂯎��Ɏ��s�������߁C�摜�\�����I�����܂��D���̃N���X�̌Ăяo�����������Ă��������D" << std::endl;	// cout�֐��̓X���b�h�Z�[�t�ł͂Ȃ��̂Ŗ{���͌ĂԂׂ��ł͂Ȃ��D
		return;
	}

	// Dxlib�̊֐��͕����X���b�h�ŌĂԂ��Ƃ��l������Ă��Ȃ��̂ŁC�����̃X���b�h����ĂԂƕK����肪�N���܂��D���̂��߁C�����������C�`��C�I�������̑S�Ă����̊֐��̒��ŌĂԕK�v������܂��D
	if (dxlibInit() == false)
	{
		std::cout << "DxLib_Init�֐��Ɏ��s�������߁C�摜�\�����I�����܂��D2�d�N�����Ă���\��������܂�" << std::endl;	// cout�֐��̓X���b�h�Z�[�t�ł͂Ȃ��̂Ŗ{���͌ĂԂׂ��ł͂Ȃ��D
		return;
	}

	//�`��̏������s���N���X���Z�b�g����D���s����`��̓��e��ύX�������Ȃ�΁C���̂悤��IGraphicMain���p���������̃N���X��<>�ɓ���Ă��������D
	GraphicLoop _Looper(std::move(mp_InterfaceGraphicMain));

	// ProcessMessage�֐��̓E�B���h�E�́~�{�^�����������Ǝ��s�̒l��Ԃ��D�܂��C�E�B���h�E���ێ����邽�߂ɂ͒���I�ɌĂяo��������K�v������̂Ń��[�v�ŌĂё����Ă���D
	// ProcessMessage�͐�����0(C++�ɂ�����false)�C���s��-1(C++�ɂ�����true��0�ȊO�̒l)��Ԃ��C���̂��� !ProcessMessage �͂��̊֐��������̎��̂݃��[�v����...���̒ɂ������ł���D
	while (!ProcessMessage())
	{
		//false���A�����ꍇ�C���[�v�𔲂���D
		if (_Looper.loop() == false)
		{
			break;
		}
	}

	//�I���������s���D
	dxlibFinalize();
}

bool GraphicSystem::dxlibInit()
{
	// 1���̏������p�֐���Dxlib_Init���ĂԑO�Ɏ��s����K�v������̂ł����Ŏ��s���܂��D

	SetOutApplicationLogValidFlag(FALSE);				//���O�o�͖����ɕύX�D��������Ȃ���Log.txt�Ƃ����ז��ȃt�@�C�����o�͂���܂��D
	SetMainWindowText(GraphicConst::WIN_NAME.c_str());	//�^�C�g����ύX�D�E�B���h�E�̍���ɕ\���������̂ł��D
	SetWindowSizeChangeEnableFlag(FALSE);               //�E�B���h�E�T�C�Y�����R�ɕύX�ł��Ȃ��悤�ɂ���D
	SetAlwaysRunFlag(TRUE);								//�E�C���h�E���A�N�e�B�u�ł͂Ȃ���Ԃł������𑱍s����悤�ɕύX�D
	ChangeWindowMode(TRUE);								//�E�C���h�E���[�h�ɕύX�D��������Ȃ��ƃt���X�N���[���ŕ\������܂��D

	//�E�B���h�E�̉����C�c���C�J���[��ݒ肵�܂��D
	SetGraphMode(GraphicConst::WIN_X, GraphicConst::WIN_Y, GraphicConst::COLOR_BIT);

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
