#pragma once

#include "node.h"


//! @class NodeDisplayGUI
//! @date 2023/08/23
//! @author ���J��
//! @brief �m�[�h�̏���\������GUI
class NodeDisplayGUI
{
public:
	NodeDisplayGUI();


	//! @brief �\������m�[�h��ݒ肷��
	//! @param [in] node �\������m�[�h
	void setDisplayNode(const SNode& node);

	//! @brief GUI�̕\�����s��
	void draw() const;

private:

	const int kGUILeftPosX = 0;
	const int kGUITopPosY = 0;

	SNode m_node;	//!< �\������m�[�h
};


//! @file node_display_gui.h
//! @date 2023/08/23
//! @author ���J��
//! @brief �m�[�h�̏���\������GUI