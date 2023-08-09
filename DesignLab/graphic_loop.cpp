#include "graphic_loop.h"

#include <iostream>

#include "DxLib.h"

#include "graphic_main_sample.h"
#include "keyboard.h"
#include "mouse.h"


GraphicLoop::GraphicLoop(std::unique_ptr<IGraphicMain>&& graphic_main)
{
	//�O���t�B�b�N���C���N���X��������D���j�[�N�|�C���^��������ꍇ��move���s���D�Q�l https://nojima.hatenablog.com/entry/2014/12/10/014131
	mp_graphic_main = std::move(graphic_main);
}


bool GraphicLoop::loop()
{
	// [�`��̏����ɂ���]
	// ScreenFlip�֐���ClearDrawScreen�֐��̏ڍׁF�E�B���h�E�̉摜�\���̓p���p������̗l�ɉ�ʂ�f�����؂�ւ��邱�ƂŃA�j���[�V�������Č����Ă���D
	// �������C�P�ɉ�ʂ�؂�ւ����ꍇ�C�{���̃p���p������̗l�ɃE�B���h�E�ɂ�������łĂ��܂��D
	// ������GraphicSystem�N���X��dxlibInit�֐��̒��ŌĂ΂�Ă��� SetDrawScreen(DX_SCREEN_BACK) �ɂ���Ă������񗠉�ʂɊG��`�悵�Ă���C
	// ScreenFlip�֐��ŃE�B���h�E�ɊG��߂����Ƃŉ�ʂ̂�������Ȃ����Ă���D
	// ClearDrawScreen �� ScreenFlip �� ProcessMessage�ƕԂ��l�������Ȃ̂ŁCloop�֐��̗l�ȏ������ƂȂ�D


	//�O���t�B�b�N���C���N���X����Ȃ�false��Ԃ��D
	if (!mp_graphic_main)
	{
		std::cout << "GraphicLoop::loop() : mp_graphic_main is empty.\n";
		return false;
	}

	//�W���o�͂�����
	clsDx();

	//�L�[���͂��X�V����D
	Keyboard::getIns()->update();
	Mouse::getIns()->update();

	//�������s��
	if (!mp_graphic_main->update())
	{
		std::cout << "GraphicLoop::loop() : mp_graphic_main->update() is false.\n";
		return false;
	}

	//�`�悷��
	if (!m_fps.skipDrawScene())
	{
		//����ʂɕ`�悵���G������
		if (ClearDrawScreen() < 0)
		{
			std::cout << "GraphicLoop::loop() : ClearDrawScreen() < 0.\n";
			return false;
		}

		mp_graphic_main->draw();

		//�X�N���[���ɗ���ʂɕ`�悵�����e���ڂ�
		if (ScreenFlip() < 0)
		{
			std::cout << "GraphicLoop::loop() : ScreenFlip() < 0.\n";
			return false;
		}
	}

	//FPS�����ɕۂ��߂ɑ҂D
	m_fps.wait();

	return true;
}