//! @file button_controller.h
//! @brief �{�^���̏����C�`����Ǘ�����N���X�D


#ifndef DESIGNLAB_BUTTON_CONTROLLER_H_
#define DESIGNLAB_BUTTON_CONTROLLER_H_


#include <string>


//! @class ButtomController
//! @brief �{�^���̏����C�`����Ǘ�����N���X�D
class ButtomController final
{
public:

	ButtomController();
	ButtomController(int x_pos, int y_pos, int x_size, int y_size, const std::string& text);
	~ButtomController() = default;

	//! @brief �{�^���̏�Ԃ��X�V����D���t���[�����s���邱�ƁD
	void Update();

	//! @brief �{�^����`�悷��D�f�U�C����ύX�������Ȃ�΂�����ύX���Ă��������D
	void Draw() const;


	//! @biref �{�^���������ꂽ�u�Ԃ�true��Ԃ��D
	//! @return bool �{�^���������ꂽ�u�Ԃ�true��Ԃ��D�����ꑱ���Ă��Ă���x����true��Ԃ��Ȃ��D
	bool IsPushedNow() const;

	//! @brief �{�^����������Ă���Ȃ��true��Ԃ��D
	//! @return bool �{�^����������Ă���Ȃ��true��Ԃ��D
	bool IsPushed() const;

	//! @brief �{�^�������t���[���ɓn���ĉ����ꑱ���Ă���̂���Ԃ��D
	//! @return int �{�^�������t���[���ɓn���ĉ����ꑱ���Ă���̂���Ԃ��D�����l[�t���[��]�D60�t���[����1�b�D
	int GetPushingFlame() const;

private:

	const int kXPos, kYPos;			//!< �{�^���̍��W
	const int kXSize, kYSize;		//!< �{�^���̉����Əc��

	bool is_mouse_in_button_;	//!< �{�^���̒��Ƀ}�E�X�����邩�D
	bool is_pushed_;			//!< �{�^���͉�����Ă��邩�D
	int pushing_frame_;			//!< �{�^���������ꑱ���Ă��鎞�ԁD�P�ʂ̓t���[���D
	std::string text_;			//!< �{�^���ɏ�����Ă���e�L�X�g
};


#endif // DESIGNLAB_BUTTON_CONTROLLER_H_