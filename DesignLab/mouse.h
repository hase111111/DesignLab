//! @file mouse.h
//! @brief Dxlib�̃}�E�X���͂��擾����N���X�D


#ifndef DESIGNLAB_MOUSE_H_
#define DESIGNLAB_MOUSE_H_


#include "singleton.h"


//! @class Mouse
//! @brief Dxlib�̃}�E�X���͂��擾����N���X�D
//! @details ���ӓ_��Keyboard�N���X�ƑS�������D��������Q�Ƃ��Ċm�F���Ăق����D

class Mouse final : public Singleton<Mouse>
{
public:

	//! @brief �}�E�X���͂��X�V����D����𖈃t���[�����s���Ȃ��ƁC�}�E�X���͂��擾�ł��Ȃ��D
	void Update();

	//! @brief �}�E�X�J�[�\���̈ʒu���擾����DX���W�͉�ʂ̍��[��0�Ƃ��āC�E���������D�����Dxlib�̎d�l�Ȃ̂ŕύX�s�\�D
	//! @return int �}�E�X�J�[�\����X���W�D
	int cursor_pos_x() const { return cursor_pos_x_; };

	//! @brief �}�E�X�J�[�\���̈ړ��ʂ��擾����DX���W�͉�ʂ̍��[��0�Ƃ��āC�E���������D�����Dxlib�̎d�l�Ȃ̂ŕύX�s�\�D
	//! @return int �}�E�X�J�[�\����X�����̈ړ��ʁD
	int GetDiffPosX() const;

	//! @brief �}�E�X�J�[�\���̈ʒu���擾����DY���W�͉�ʂ̏�[��0�Ƃ��āC�����������D�����Dxlib�̎d�l�Ȃ̂ŕύX�s�\�D
	//! @return int �}�E�X�J�[�\����Y���W�D
	int cursor_pos_y() const { return cursor_pos_y_; };

	//! @brief �}�E�X�J�[�\���̈ړ��ʂ��擾����DY���W�͉�ʂ̏�[��0�Ƃ��āC�����������D�����Dxlib�̎d�l�Ȃ̂ŕύX�s�\�D
	//! @return int �}�E�X�J�[�\����Y�����̈ړ��ʁD
	int GetDiffPosY() const;

	//! @brief �}�E�X�J�[�\���̈ړ��ʂ��擾����D
	//! @return double �}�E�X�J�[�\���̈ړ��ʁD
	double getDiffPos() const;

	//! @brief �}�E�X�̍��N���b�N�̉����ꑱ���Ă���t���[�������擾����D
	//! @return int ������Ă���t���[�����D
	int left_pushing_counter() const { return left_pushing_counter_; };

	//! @brief �}�E�X�̉E�N���b�N�̉����ꑱ���Ă���t���[�������擾����D
	//! @return int ������Ă���t���[�����D
	int right_pushing_counter() const { return right_pushing_counter_; };

	//! @brief �}�E�X�̃z�C�[���{�^���̉����ꑱ���Ă���t���[�������擾����D
	//! @return int ������Ă���t���[�����D
	int middle_pushing_counter() const { return middle_pushing_counter_; };

	//! @brief �}�E�X�̍��N���b�N�̗����ꑱ���Ă���t���[�������擾����D
	//! @return int ������Ă���t���[�����D
	int left_releasing_counter() const { return left_releasing_counter_; };

	//! @brief �}�E�X�̉E�N���b�N�̗����ꑱ���Ă���t���[�������擾����D
	//! @return int ������Ă���t���[�����D
	int right_releasing_counter() const { return right_releasing_counter_; };

	//! @brief �}�E�X�̃z�C�[���{�^���̗����ꑱ���Ă���t���[�������擾����D
	//! @return int ������Ă���t���[�����D
	int middle_releasing_counter() const { return middle_releasing_counter_; };

	//! @brief �}�E�X�̃z�C�[���̉�]�ʂ��擾����D
	//! @n 1�t���[���ŉ�]�����ʂ��擾����D
	//! @n ��O�ɉ񂵂����̓}�C�i�X�̒l�Ƃ��āA���ɉ񂵂����̓v���X�̒l�Ƃ��ĕԂ�
	//! @return int �}�E�X�z�C�[���̉�]�ʁD
	int wheel_rot() const { return wheel_rot_; };

private:

	Mouse();
	friend Singleton<Mouse>;


	int cursor_pos_x_, cursor_pos_y_;				//!< �}�E�X�J�[�\���̈ʒu
	int cursor_past_pos_x_, cursor_past_pos_y_;		//!< 1�t���[���O�̃}�E�X�J�[�\���̈ʒu

	int right_pushing_counter_, left_pushing_counter_, middle_pushing_counter_;			//!< �}�E�X�̃{�^���������ꑱ���Ă���t���[����
	int right_releasing_counter_, left_releasing_counter_, middle_releasing_counter_;	//!< �}�E�X�̃{�^���������ꑱ���Ă���t���[����

	int wheel_rot_;		//!< �}�E�X�z�C�[���̉�]��
};


#endif // !DESIGNLAB_MOUSE_H_