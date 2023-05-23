#pragma once
#include "InterfaceGraphicMain.h"
#include "MapState.h"
#include <vector>
#include "listFunc.h"
#include "GraphicConst.h"
#include "CameraController.h"
#include "Hexapod.h"

class GraphicMainBasic final : public IGraphicMain
{
public:
	GraphicMainBasic(const GraphicDataBroker* _broker);
	~GraphicMainBasic() = default;

	bool update() override;

	void draw() const override;

private:

	CameraController m_Camera;	//�J��������N���X�D

	MapState m_Map;				//�\������}�b�v�D

	std::vector<SNode> m_node;	//���{�b�g�̓����̑J�ڂ��L�^����vector

	Hexapod m_hexapod;			//���{�b�g�̏�Ԃ��Ǘ�����N���X�D
	
	int m_counter = 0;			// ���̃N���X�����s����Ă��牽��update�֐����Ă΂ꂽ���J�E���g����

	int m_display_node = 0;		//�`�悵�Ă���m�[�h

	const int CHANGE_NEXT_NODE = (int)(0.2 * GraphicConst::GRAPHIC_FPS);	//���̃m�[�h���Đ�����܂ł̎��ԁD

	const int GET_NODE_COUNT = 2 * GraphicConst::GRAPHIC_FPS;	//2�b���Ƃɓǂݏo���D
};
