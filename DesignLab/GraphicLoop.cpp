#include "GraphicLoop.h"
#include "DxLib.h"

//�`��̏����ɂ��āC
// ScreenFlip�֐���ClearDrawScreen�֐��ɂ��āD�E�B���h�E�̉摜�\���̓p���p������̗l�ɉ�ʂ�f�����؂�ւ��邱�ƂŃA�j���[�V�������Č����Ă���D
// �������C�P�ɉ�ʂ�؂�ւ����ꍇ�C�{���̃p���p������̗l�ɃE�B���h�E�ɂ�������łĂ��܂��D
// ������init�֐��̒��ŌĂ΂�Ă��� SetDrawScreen(DX_SCREEN_BACK) �ɂ���Ă������񗠉�ʂɊG��`�悵�Ă���CScreenFlip�֐��ŃE�B���h�E�ɊG��߂����Ƃŉ�ʂ̂�������Ȃ����Ă���D
// ClearDrawScreen �� ScreenFlip �� ProcessMessage�ƕԂ��l�������Ȃ̂ŁC�ȉ��̗l�ȏ������ƂȂ�D


bool GraphicLoop::loop()
{
	//�������s��

	if (m_fps.skipDrawScene() == false) 
	{
		if (ClearDrawScreen() < 0) { return false; }	//����ʂɕ`�悵���G������

		//�`�悷��
		
		if (ScreenFlip() < 0) { return false; }		//�X�N���[���ɗ���ʂɕ`�悵�����e���ڂ�
	}

	m_fps.wait();	//FPS�����ɕۂ��߂ɑ҂D

	return true;
}