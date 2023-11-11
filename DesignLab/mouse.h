//! @file mouse.h
//! @brief Dxlib�̃}�E�X���͂��擾����N���X�D


#ifndef DESIGNLAB_MOUSE_H_
#define DESIGNLAB_MOUSE_H_


#include <array>
#include <map>

#include "singleton.h"


//! @class Mouse
//! @brief Dxlib�̃}�E�X���͂��擾����N���X�D
//! @details ���ӓ_��Keyboard�N���X�ƑS�������D��������Q�Ƃ��Ċm�F���Ăق����D
class Mouse final : public Singleton<Mouse>
{
public:

	//! @brief �}�E�X���͂��X�V����D����𖈃t���[�����s���Ȃ��ƁC�}�E�X���͂��擾�ł��Ȃ��D
	void Update();

	//! @brief mouseCode�̃{�^����������Ă���t���[�������擾����D
	//! @param [in] mouseCode �ǂ̃{�^���𒲂ׂ������D
	//! @n �Ⴆ�ΉE�N���b�N��������CMOUSE_INPUT_RIGHT�ƂȂ�D
	//! @return int ������Ă���t���[�����D�ُ��mouse_code���n���ꂽ�ꍇ��-1��Ԃ��D
	int GetPressingCount(int mouse_code) const;

	//! @brief mouseCode�̃{�^����������Ă���t���[�������擾����D
	//! @param [in] mouseCode �ǂ̃{�^���𒲂ׂ������D
	//! @n �Ⴆ�ΉE�N���b�N��������CMOUSE_INPUT_RIGHT�ƂȂ�D
	//! @return int ������Ă���t���[�����D�ُ��mouse_code���n���ꂽ�ꍇ��-1��Ԃ��D
	int GetReleasingCount(int mouse_code) const;

	//! @brief �}�E�X�J�[�\���̈ʒu���擾����DX���W�͉�ʂ̍��[��0�Ƃ��āC�E���������D�����Dxlib�̎d�l�Ȃ̂ŕύX�s�\�D
	//! @return int �}�E�X�J�[�\����X���W�D
	inline int GetCursorPosX() const { return cursor_pos_x_; };

	//! @brief �}�E�X�J�[�\���̈ړ��ʂ��擾����DX���W�͉�ʂ̍��[��0�Ƃ��āC�E���������D�����Dxlib�̎d�l�Ȃ̂ŕύX�s�\�D
	//! @return int �}�E�X�J�[�\����X�����̈ړ��ʁD
	int GetDiffPosX() const;

	//! @brief �}�E�X�J�[�\���̈ʒu���擾����DY���W�͉�ʂ̏�[��0�Ƃ��āC�����������D�����Dxlib�̎d�l�Ȃ̂ŕύX�s�\�D
	//! @return int �}�E�X�J�[�\����Y���W�D
	inline int GetCursorPosY() const { return cursor_pos_y_; };

	//! @brief �}�E�X�J�[�\���̈ړ��ʂ��擾����DY���W�͉�ʂ̏�[��0�Ƃ��āC�����������D�����Dxlib�̎d�l�Ȃ̂ŕύX�s�\�D
	//! @return int �}�E�X�J�[�\����Y�����̈ړ��ʁD
	int GetDiffPosY() const;

	//! @brief �}�E�X�J�[�\���̈ړ��ʂ��擾����D
	//! @return double �}�E�X�J�[�\���̈ړ��ʁD
	double GetDiffPos() const;

	//! @brief �}�E�X�̃z�C�[���̉�]�ʂ��擾����D
	//! @n 1�t���[���ŉ�]�����ʂ��擾����D
	//! @n ��O�ɉ񂵂����̓}�C�i�X�̒l�Ƃ��āA���ɉ񂵂����̓v���X�̒l�Ƃ��ĕԂ�
	//! @return int �}�E�X�z�C�[���̉�]�ʁD
	inline int GetWheelRot() const { return wheel_rot_; };

private:

	Mouse();
	friend Singleton<Mouse>;

	constexpr static int kMouseKeyNum = 8;		//!< �}�E�X�̃{�^���̐��D�E�E���E���� + 5�̃{�^���D

	const std::array<int, kMouseKeyNum> kMouseKeyCodes;	//!< �}�E�X�̃{�^���̃L�[�R�[�h

	int cursor_pos_x_, cursor_pos_y_;			//!< �}�E�X�J�[�\���̈ʒu
	int cursor_past_pos_x_, cursor_past_pos_y_;	//!< 1�t���[���O�̃}�E�X�J�[�\���̈ʒu

	std::map<int, int> pushing_counter_;	//!< �}�E�X�̃{�^���������ꑱ���Ă���t���[�����D
	std::map<int, int> releasing_counter_;	//!< �}�E�X�̃{�^���������ꑱ���Ă���t���[�����D

	int wheel_rot_;		//!< �}�E�X�z�C�[���̉�]�ʁC��O�ɉ񂵂����̓}�C�i�X�̒l�Ƃ��āA���ɉ񂵂����̓v���X�̒l�Ƃ��ĕԂ�
};


#endif // DESIGNLAB_MOUSE_H_