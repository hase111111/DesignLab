#include "GraphicLoop.h"
#include "DxLib.h"
#include "GraphicMainSample.h"
#include "Keyboard.h"

//�`��̏����ɂ��āC
// ScreenFlip�֐���ClearDrawScreen�֐��ɂ��āD�E�B���h�E�̉摜�\���̓p���p������̗l�ɉ�ʂ�f�����؂�ւ��邱�ƂŃA�j���[�V�������Č����Ă���D
// �������C�P�ɉ�ʂ�؂�ւ����ꍇ�C�{���̃p���p������̗l�ɃE�B���h�E�ɂ�������łĂ��܂��D
// ������init�֐��̒��ŌĂ΂�Ă��� SetDrawScreen(DX_SCREEN_BACK) �ɂ���Ă������񗠉�ʂɊG��`�悵�Ă���CScreenFlip�֐��ŃE�B���h�E�ɊG��߂����Ƃŉ�ʂ̂�������Ȃ����Ă���D
// ClearDrawScreen �� ScreenFlip �� ProcessMessage�ƕԂ��l�������Ȃ̂ŁC�ȉ��̗l�ȏ������ƂȂ�D


GraphicLoop::GraphicLoop(std::unique_ptr<AbstractGraphicMain>&& mp_graphic_main)
{
	//�O���t�B�b�N���C���N���X���Z�b�g����D���j�[�N�|�C���^��������ꍇ��move���s���D�Q�l https://nojima.hatenablog.com/entry/2014/12/10/014131
	mp_GraphicMain = std::move(mp_graphic_main);
}

bool GraphicLoop::loop()
{
	//�O���t�B�b�N���C���N���X����Ȃ�false��Ԃ��D
	if (!mp_GraphicMain) { return false; }

	//�L�[���͂��X�V����D
	Keyboard::getIns()->update();

	//�������s��
	mp_GraphicMain->update();

	if (m_Fps.skipDrawScene() == false) 
	{
		if (ClearDrawScreen() < 0) { return false; }	//����ʂɕ`�悵���G������

		//�`�悷��
		mp_GraphicMain->draw();
		
		if (ScreenFlip() < 0) { return false; }		//�X�N���[���ɗ���ʂɕ`�悵�����e���ڂ�
	}

	m_Fps.wait();	//FPS�����ɕۂ��߂ɑ҂D

	return true;
}