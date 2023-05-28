#pragma once
#include "GraphicConst.h"
#include "Node.h"
#include "ButtonController.h"
#include "CameraController.h"
#include <memory>
#include <vector>

// UI��\������֐��D�\��������̂�ύX�������ꍇ�͕ҏW���Ă��������D
class GUIController
{
public:
	GUIController();

	void update(CameraController& _camera);

	void draw(const SNode _node) const;

private:

	const int BOX_X = 300;
	const int BOX_Y = GraphicConst::WIN_Y - 25 * 2;
	const int CENTER_X = 25 + BOX_X / 2;
	const int CENTER_Y = GraphicConst::WIN_Y / 2;

	//UI��\�����邩�ǂ����D
	bool m_is_displayed = true;

	//�����Ƀm�[�h�̏�Ԃ�`�悷��֐��D��ω����֐��ł��D�\����Ȃ�
	void drawNodeByStr(const SNode _node) const;

	//�E���Ƀ{�^���̎g������`�悷��֐��D����
	void drawExplanationByStr() const;

	// GUI�ɕ\������{�^�����Ǘ�����N���X
	std::vector<std::unique_ptr<ButtomController>> m_buttom;
};
