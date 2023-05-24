#pragma once
#include "InterfaceGraphicMain.h"
#include "MapState.h"
#include <vector>
#include "listFunc.h"
#include "GraphicConst.h"
#include "CameraController.h"
#include "Hexapod.h"
#include "GUIController.h"

//���̃v���W�F�N�g�ɂ�����W���I�ȃ��{�b�g�̕`��@�\�����N���X�D
class GraphicMainBasic final : public IGraphicMain
{
public:
	GraphicMainBasic(const GraphicDataBroker* _broker);
	~GraphicMainBasic() = default;

	bool update() override;

	void draw() const override;

private:

	CameraController m_Camera;	//�J��������N���X�D

	GUIController m_GUI;		// GUI (���{�b�g�̏�ԂƂ��\�����鑋) �𐧌䂷��N���X�D

	Hexapod m_hexapod;			//���{�b�g�̏�Ԃ��Ǘ�����N���X�D
	
	int m_counter = 0;			// ���̃N���X�����s����Ă��牽��update�֐����Ă΂ꂽ���J�E���g����

	std::vector<SNode> m_node;	//���{�b�g�̓����̑J�ڂ��L�^����vector

	int m_display_node = 0;		//�`�悵�Ă���m�[�h

	MapState m_Map;				//�\������}�b�v�D


	const int CHANGE_NEXT_NODE = (int)(0.2 * GraphicConst::GRAPHIC_FPS);	//���̃m�[�h���Đ�����܂ł̎��ԁD

	const int GET_NODE_COUNT = 2 * GraphicConst::GRAPHIC_FPS;	//2�b���Ƃɓǂݏo���D
};
