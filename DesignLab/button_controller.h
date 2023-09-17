#pragma once

#include <string>


//! @class ButtomController
//! @date 2023/08/08
//! @author ���J��
//! @brief �{�^���̏����C�`����Ǘ�����N���X�D
class ButtomController final
{
public:

	ButtomController();
	ButtomController(const int xpos, const int ypos, const int xsize, const int ysize, const std::string text);
	~ButtomController() = default;

	//! @brief �{�^���̏�Ԃ��X�V����D���t���[�����s���邱�ƁD
	void Update();

	//! @brief �{�^����`�悷��D�f�U�C����ύX�������Ȃ�΂�����ύX���Ă��������D
	void Draw() const;


	//! @biref �{�^���������ꂽ�u�Ԃ�true��Ԃ��D
	//! @return bool �{�^���������ꂽ�u�Ԃ�true��Ԃ��D�����ꑱ���Ă��Ă���x����true��Ԃ��Ȃ��D
	bool isPushedNow() const;

	//! @brief �{�^����������Ă���Ȃ��true��Ԃ��D
	//! @return bool �{�^����������Ă���Ȃ��true��Ԃ��D
	bool isPushed() const;

	//! @brief �{�^�������t���[���ɓn���ĉ����ꑱ���Ă���̂���Ԃ��D
	//! @return int �{�^�������t���[���ɓn���ĉ����ꑱ���Ă���̂���Ԃ��D�����l[�t���[��]�D60�t���[����1�b�D
	int getPushingFlame() const;

private:

	bool m_is_mouse_in_buttom = false;	//!< �{�^���̒��Ƀ}�E�X�����邩�D
	bool m_is_pushed = false;			//!< �{�^���͉�����Ă��邩�D
	int m_pushing_frame = 0;			//!< �{�^���������ꑱ���Ă��鎞�ԁD�P�ʂ̓t���[���D

	const int kXPos, kYPos;			//!< �{�^���̍��W
	const int kXSize, kYSize;		//!< �{�^���̉����Əc��
	std::string m_text;				//!< �{�^���ɏ�����Ă���e�L�X�g
};

//! @file button_controller.h
//! @date 2023/08/08
//! @author ���J��
//! @brief �{�^���̏����C�`����Ǘ�����N���X�D
//! @n �s�� : @lineinfo
