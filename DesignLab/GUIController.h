#pragma once

#include <memory>
#include <vector>

#include "button_controller.h"
#include "graphic_const.h"
#include "Node.h"
#include "CameraController.h"


enum class ENodeDisplayNode : int
{
	AutoUpdate,
	AlwaysNew,
	Selectable
};


// UI��\������֐��D�\��������̂�ύX�������ꍇ�͕ҏW���Ă��������D
class GUIController
{
public:
	GUIController();

	void update(CameraController& _camera, const int _max_node, int& _display_node, const int _counter);

	void draw(const SNode _node) const;

private:

	const int BOX_X = 300;
	const int BOX_Y = GraphicConst::WIN_Y - 25 * 2;
	const int CENTER_X = 25 + BOX_X / 2;
	const int CENTER_Y = GraphicConst::WIN_Y / 2;

	const int CHANGE_NEXT_NODE = GraphicConst::GRAPHIC_FPS / 5;	//���̃m�[�h���Đ�����܂ł̎��ԁD

	bool m_is_displayed = true;		//UI��\�����邩�ǂ����D

	ENodeDisplayNode m_mode = ENodeDisplayNode::Selectable;		//�ǂ̂悤�Ƀm�[�h��\�����邩

	//�����Ƀm�[�h�̏�Ԃ�`�悷��֐��D��ω����֐��ł��D�\����Ȃ�
	void drawNodeByStr(const SNode _node) const;

	//�E���Ƀ{�^���̎g������`�悷��֐��D����
	void drawExplanationByStr() const;

	// GUI�ɕ\������{�^�����Ǘ�����N���X
	std::vector<std::unique_ptr<ButtomController>> m_buttom;
};
