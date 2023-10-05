//! @file camera_gui.h
//! @brief �J�����̑���E�Ǘ����s��Gui�̏����C�\�����s���N���X


#ifndef DESIGNLAB_CAMERA_GUI_H_
#define DESIGNLAB_CAMERA_GUI_H_


#include <map>
#include <memory>

#include "camera_input_controller.h"
#include "camera_state_manager.h"
#include "button_controller.h"
#include "designlab_vector3.h"


//! @class CameraGui
//! @brief �J�����̑���E�Ǘ����s��GUI���s���N���X
//! @details �g�p�������ꍇ�́C�����o�ɂ������������āCUpdate��Draw���Ăяo�������ł悢�D
class CameraGui final
{
public:

	//! @brief GUI��\������ʒu���w�肵�Ȃ��ꍇ�̃R���X�g���N�^�C���� (0, 0) ���N�_�ɕ\������
	CameraGui();

	//! @param [in] x_pos GUI��\������x���W
	//! @param [in] y_pos GUI��\������y���W
	//! @param [in] option GUI�̂ǂ̒n�_���N�_�ɕ\���ʒu���w�肷�邩
	CameraGui(int x_pos, int y_pos, unsigned int option);

	//! @brief �J�������������郍�{�b�g�̍��W��ݒ肷��
	//! @param[in] pos ���{�b�g�̍��W
	void SetHexapodPos(const designlab::Vector3& pos);

	//! @brief GUI��J�����̍X�V���s��
	void Update();

	//! @brief GUI�̕`����s��
	void Draw() const;

	
	constexpr static unsigned int kOptionLeftTop = 0b00;		//!< ������N�_�ɍ��W��ݒ肷��
	constexpr static unsigned int kOptionLeftBottom = 0b10;		//!< �������N�_�ɍ��W��ݒ肷��
	constexpr static unsigned int kOptionRightTop = 0b01;		//!< �E����N�_�ɍ��W��ݒ肷��
	constexpr static unsigned int kOptionRightBottom = 0b11;	//!< �E�����N�_�ɍ��W��ݒ肷��

private:

	enum class ButtonType
	{
		kLenghReset,
		kFront,
		kBack,
		kLeft,
		kRight,
		kTop,
		kTargetReset,
		kClosed
	};


	//! @brief GUI�̔w�i��`�悷��
	void DrawBackground() const;

	//! @brief �ŏ�������GUI�̔w�i��`�悷��
	void DrawClosedBackground() const;

	//! @brief GUI�̕�����`�悷��
	void DrawString() const;


	const int kGuiSizeX;		//!< GUI�̉���
	const int kGuiSizeY;		//!< GUI�̏c��
	const int kGuiSizeYClosed;	//!< GUI�̏c��(�ŏ�����)

	const int kGuiLeftPosX;		//!< GUI�̍��[�̈ʒu
	const int kGuiTopPosY;		//!< GUI�̏�[�̈ʒu

	const int kButtonDistance;	//!< �{�^�����m�̊Ԋu
	const int kButtonSize;		//!< �{�^���̃T�C�Y


	CameraInputController camera_controller_;	//!< �J�����̑�����s���N���X

	CameraStateManager camera_manager_;			//!< �J�����̊Ǘ����s���N���X

	std::map<ButtonType, std::unique_ptr<ButtomController>> button_list_;	//!< �{�^���̃��X�g


	bool is_closed_;
};


#endif	// DESIGNLAB_CAMERA_GUI_H_