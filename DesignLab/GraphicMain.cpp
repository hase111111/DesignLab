#include "GraphicMain.h"
#include "DxLib.h"
#include "GraphicConst.h"
#include "GraphicLoop.h"

//���ӂƂ��āCDxlib�n�̊֐��� �^�U��啶���� TRUE��FALSE���g���ĕ\���̂ŁC�]����true false���g�p���Ȃ��悤�ɂ��܂��傤�D
//�܂��C�Ԃ����Ⴏ�������̕��ł��������ǁC�o�[�W�����̍X�V�ɂ���ē����Ȃ��Ȃ�\��������̂�Dxlib�ɑg�ݍ��܂�Ă�����̂��g���̂�����ł��D
//�܂��CDxlib�̓G���[���o���Ƃ��� -1 ��Ԃ��֐������ɑ����ł��D���̂��ߗႦ�� if(DxLib_Init() == false) �Ə����Ă��G���[���󂯎��Ȃ����Ƃ�����܂��D
//�������� if(DxLib_Init() < 0) �ƂȂ�܂��D����� bool�^ ���f�t�H���g�ő��݂��Ȃ�C����ł��g�p���邱�Ƃ��ł���悤�ɂ��邽�߂̔z���ł���CC++�ŏ�����Ă���{�R�[�h�ɂ����Ă�
//�����̌��ł�(��)�DDxlib�̃G���[��bool�ł͂Ȃ��Cint�^�̕��̒l�Ƃ������Ƃ��o���Ă����Ă��������D


bool GraphicMain::init()
{	
	// 1���̏������p�֐���Dxlib_Init���ĂԑO�Ɏ��s����K�v������̂ł����Ŏ��s���܂��D

	SetMainWindowText(GraphicConst::WIN_NAME.c_str());	//�^�C�g����ύX�D�E�B���h�E�̍���ɕ\���������̂ł��D
	ChangeWindowMode(TRUE);								//�E�C���h�E���[�h�ɕύX�D��������Ȃ��ƃt���X�N���[���ŕ\������܂��D
	SetWindowSizeChangeEnableFlag(FALSE);               //�E�B���h�E�T�C�Y�����R�ɕύX�ł��Ȃ��悤�ɂ���D
	SetOutApplicationLogValidFlag(FALSE);				//���O�o�͖����ɕύX�D��������Ȃ���Log.txt�Ƃ����ז��ȃt�@�C�����o�͂���܂��D
	SetAlwaysRunFlag(TRUE);								//�E�C���h�E���A�N�e�B�u�ł͂Ȃ���Ԃł������𑱍s����悤�ɕύX�D

	//�E�B���h�E�̉����C�c���C�J���[��ݒ肵�܂��D
	SetGraphMode(GraphicConst::WIN_X, GraphicConst::WIN_Y, GraphicConst::COLOR_BIT);

	//�c�w���C�u��������������
	if (DxLib_Init() < 0)
	{
		//�G���[���o��ƕ��̒l��Ԃ��̂ŁC���̏ꍇ��false�D
		m_is_init_success = false;
		return false;
	}

	//�`���𗠉�ʂɂ���D����������̂ł����C��ʂ̂�����������Ă������ʂ�����CDxlib���g���ȏ�K�{�̍��ڂł��D
	SetDrawScreen(DX_SCREEN_BACK);						

	//�������ɐ��������̂ŁC�t���O�𗧂ĂďI��.
	m_is_init_success = true;
	return true;
}

void GraphicMain::main()
{
	//�����������Ă��Ȃ� or ���s�����ꍇ�I��
	if (m_is_init_success == false) { return; }

	GraphicLoop _Looper;

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

	//�I������
	finalize();
}

void GraphicMain::finalize() const
{
	// DX���C�u�����̏I���������Ă�.
	DxLib_End();
}
