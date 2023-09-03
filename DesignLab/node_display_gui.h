#pragma once

#include <memory>
#include <map>

#include "node.h"
#include "button_controller.h"



//! @class NodeDisplayGUI
//! @date 2023/08/23
//! @author ���J��
//! @brief �m�[�h�̏���\������GUI
class NodeDisplayGUI final
{
public:
	NodeDisplayGUI() = default;
	NodeDisplayGUI(const int x_pos, const int y_pos);


	//! @brief �\������m�[�h��ݒ肷��
	//! @param [in] node �\������m�[�h
	void setDisplayNode(const SNode& node);


	//! @brief GUI�̃{�^���̍X�V���s��
	void update();

	//! @brief GUI�̕\�����s��
	void draw() const;


	const static int BOX_SIZE_X;
	const static int BOX_SIZE_Y;
	const static int BOX_SIZE_Y_CLOSED;

private:

	void drawBackground() const;

	void drawNodeInfo() const;


	enum class EButtonType
	{
		OPEN_CLOSE,
		SWITCHING
	};

	enum class EDisplayType
	{
		DEFUALT,	//�f�t�H���g
		LEG_STATE		//�p�x
	};


	const int kGUILeftPosX = 0;
	const int kGUITopPosY = 0;


	std::map<EButtonType, std::unique_ptr<ButtomController>> m_buttons;	//!< �{�^��

	SNode m_node;	//!< �\������m�[�h

	bool m_is_closed = false;	//!< GUI�����Ă��邩�ǂ���

	EDisplayType m_display_type = EDisplayType::DEFUALT;	//!< �\��������̎��
};


//! @file node_display_gui.h
//! @date 2023/08/23
//! @author ���J��
//! @brief �m�[�h�̏���\������GUI