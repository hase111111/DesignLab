#pragma once
#include "Fps.h"
#include "AbstractGraphicMain.h"
#include <memory>

class GraphicLoop final
{
public:
	GraphicLoop() = delete;	//�ʏ�̃R���X�g���N�^�͌ĂׂȂ��D�K��std::unique_ptr<AbstractGraphicMain>�������ɂƂ�K�v������D

	GraphicLoop(std::unique_ptr<AbstractGraphicMain>&& mp_graphic_main);
	~GraphicLoop() = default;

	//�摜�\���̏����̒��ŌĂ΂ꑱ���鏈�����܂Ƃ߂����́D���[�v�𔲂���������false��Ԃ��D
	bool loop();

private:

	//FPS�����ɐ��䂷��N���X�D�ڂ�����Fps.h��
	Fps m_Fps;

	//�`�揈�����s�����C���̃N���X�Dunique_ptr(���j�[�N�|�C���^)�ɂ��ẮC�Q�Ɓ� https://qiita.com/seriru13/items/06d044cbe5bcc44cca10
	std::unique_ptr<AbstractGraphicMain> mp_GraphicMain;
};
