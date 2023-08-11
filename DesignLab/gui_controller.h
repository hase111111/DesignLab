#pragma once

#include <memory>
#include <vector>

#include "button_controller.h"
#include "graphic_const.h"
#include "Node.h"
#include "CameraController.h"


//! @enum ENodeDisplayNode
//! @date 2023/08/09
//! @auther ���J��
//! @brief �m�[�h�̕\�����@��\���񋓌^�D
enum class ENodeDisplayNode : int
{
	AUTO_UPDATE,
	ALWAYS_NEW,
	SELECTABLE
};


//! @class GUIController
//! @date 2023/08/09
//! @author ���J�� 
//! @brief UI��\������֐��D�\��������̂�ύX�������ꍇ�͕ҏW���Ă��������D
class GUIController
{
public:
	GUIController();

	void update(CameraController& camera, const int max_node, int& display_node, const int counter);

	void draw(const SNode& node) const;

private:

	//�����Ƀm�[�h�̏�Ԃ�`�悷��֐��D��ω����֐��ł��D�\����Ȃ�
	void drawNodeByStr(const SNode node) const;

	//�E���Ƀ{�^���̎g������`�悷��֐��D����
	void drawExplanationByStr() const;

	const int BOX_X = 300;
	const int BOX_Y = GraphicConst::WIN_Y - 25 * 2;
	const int CENTER_X = 25 + BOX_X / 2;
	const int CENTER_Y = GraphicConst::WIN_Y / 2;

	const int CHANGE_NEXT_NODE = GraphicConst::GRAPHIC_FPS / 5;	//���̃m�[�h���Đ�����܂ł̎��ԁD

	bool m_is_displayed = true;		//UI��\�����邩�ǂ����D

	ENodeDisplayNode m_mode = ENodeDisplayNode::SELECTABLE;		//�ǂ̂悤�Ƀm�[�h��\�����邩

	// GUI�ɕ\������{�^�����Ǘ�����N���X
	std::vector<std::unique_ptr<ButtomController>> m_buttom;
};


//! @file gui_controller.h
//! @date 2023/08/09
//! @auther ���J��
//! @brief GUI��\������N���X�D
//! @n �s�� : @lineinfo
