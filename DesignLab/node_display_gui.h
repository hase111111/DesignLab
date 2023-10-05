//! @file node_display_gui.h
//! @brief �m�[�h�̏���\������GUI

#ifndef DESIGNLAB_NODE_DISPLAY_GUI_H_
#define DESIGNLAB_NODE_DISPLAY_GUI_H_

#include <array>
#include <memory>
#include <map>

#include "robot_state_node.h"
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
	//! @param [in] calculator_ptr �Z�r���s���{�b�g�̏�Ԃ��v�Z����N���X
	NodeDisplayGui(const int x_pos, const int y_pos, const std::shared_ptr<const AbstractHexapodStateCalculator>& calculator_ptr);


	//! @brief �\������m�[�h��ݒ肷��C���̌�֐߂̊p�x���v�Z���C�Z�b�g����
	//! @param [in] node �\������m�[�h
	void SetDisplayNode(const RobotStateNode& node);


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


	void DrawBackground() const;

	void DrawNodeInfo() const;

	void DrawJointInfo() const;


	const int kGuiLeftPosX;

	const int kGuiTopPosY;


	std::map<ButtonType, std::unique_ptr<ButtomController>> buttons_;				//!< �{�^��

	const std::shared_ptr<const AbstractHexapodStateCalculator> calculator_ptr_;	//!< �Z�r���s���{�b�g�̏�Ԃ��v�Z����N���X


	RobotStateNode display_node_;										//!< �\������m�[�h

	std::array<HexapodJointState, HexapodConst::LEG_NUM> joint_state_;		//!< �֐߂̊p�x

	bool is_closed_;											//!< GUI�����Ă��邩(�ŏ������Ă��邩)�ǂ���

	DisplayMode display_type_;									//!< �\��������̎��
};


#endif // !DESIGNLAB_NODE_DISPLAY_GUI_H_