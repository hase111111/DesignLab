#pragma once
#include "Fps.h"

class GraphicLoop final
{
public:
	GraphicLoop() = default;
	~GraphicLoop() = default;

	//�摜�\���̏����̒��ŌĂ΂ꑱ���鏈�����܂Ƃ߂����́D���[�v�𔲂���������false��Ԃ��D
	bool loop();

private:

	//FPS�����ɐ��䂷��N���X�D�ڂ�����Fps.h��
	Fps m_fps;
};
