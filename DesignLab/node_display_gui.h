//! @file node_display_gui.h
//! @brief �m�[�h�̏���\������GUI

#ifndef DESIGNLAB_NODE_DISPLAY_GUI_H_
#define DESIGNLAB_NODE_DISPLAY_GUI_H_

#include <memory>
#include <map>

#include "node.h"
#include "button_controller.h"
#include "abstract_hexapod_state_calculator.h"


//! @class NodeDisplayGui
//! @date 2023/08/23
//! @author ���J��
//! @brief �m�[�h�̏���\������GUI
class NodeDisplayGui final
{
public:

	//! @param [in] x_pos GUI�̍����x���W
	//! @param [in] y_pos GUI�̍����y���W
	//! @param [in] calc �Z�r���s���{�b�g�̏�Ԃ��v�Z����N���X
	NodeDisplayGui(const int x_pos, const int y_pos, std::shared_ptr<AbstractHexapodStateCalculator> calc);


	//! @brief �\������m�[�h��ݒ肷��C���̌�֐߂̊p�x���v�Z����
	//! @param [in] node �\������m�[�h
	void SetDisplayNode(const SNode& node);


	//! @brief GUI�̃{�^���̍X�V���s��
	void Update();

	//! @brief GUI�̕\�����s��
	void Draw() const;


	const static int kWidth;			//!< GUI�̕�
	const static int kHeight;			//!< GUI�̍���
	const static int kClosedHeight;		//!< GUI�����Ă���Ƃ��̍���

private:

	enum class ButtonType
	{
		kOpenClose,
		kModeSwitching
	};

	enum class DisplayMode
	{
		kDefualt,		//�f�t�H���g
		kJointState		//�p�x
	};


	void drawBackground() const;

	void drawNodeInfo() const;

	void drawJointInfo() const;


	const int kGUILeftPosX;

	const int kGUITopPosY;


	std::map<ButtonType, std::unique_ptr<ButtomController>> buttons_;	//!< �{�^��

	std::shared_ptr<AbstractHexapodStateCalculator> calculator_ptr_;		//!< �Z�r���s���{�b�g�̏�Ԃ��v�Z����N���X


	SNode display_node_;										//!< �\������m�[�h

	SHexapodJointState joint_state_[HexapodConst::LEG_NUM];	//!< �֐߂̊p�x

	bool is_closed_;											//!< GUI�����Ă��邩(�ŏ������Ă��邩)�ǂ���

	DisplayMode display_type_;									//!< �\��������̎��
};


#endif // !DESIGNLAB_NODE_DISPLAY_GUI_H_