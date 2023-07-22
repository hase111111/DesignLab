#include "GraphicLoop.h"
#include "DxLib.h"
#include "GraphicMainSample.h"
#include "Keyboard.h"
#include "Mouse.h"


GraphicLoop::GraphicLoop(std::unique_ptr<IGraphicMain>&& mp_graphic_main)
{
	//�O���t�B�b�N���C���N���X��������D���j�[�N�|�C���^��������ꍇ��move���s���D�Q�l https://nojima.hatenablog.com/entry/2014/12/10/014131
	mp_GraphicMain = std::move(mp_graphic_main);
}

bool GraphicLoop::loop()
{
	//�O���t�B�b�N���C���N���X����Ȃ�false��Ԃ��D
	if (!mp_GraphicMain) { return false; }

	//�W���o�͂�����
	clsDx();

	//�L�[���͂��X�V����D
	Keyboard::getIns()->update();
	Mouse::getIns()->update();

	//�������s��
	if (mp_GraphicMain->update() == false) { return false; }

	//�`�悷��
	if (m_Fps.skipDrawScene() == false)
	{
		if (ClearDrawScreen() < 0) { return false; }	//����ʂɕ`�悵���G������

		mp_GraphicMain->draw();

		if (ScreenFlip() < 0) { return false; }		//�X�N���[���ɗ���ʂɕ`�悵�����e���ڂ�
	}

	//FPS�����ɕۂ��߂ɑ҂D
	m_Fps.wait();

	return true;
}