#pragma once

#include "singleton.h"


//! @class Mouse
//! @date 2023/08/07
//! @author ���J��
//! @brief Dxlib�̃}�E�X���͂��擾����N���X�D
//! @details ���ӓ_��Keyboard�N���X�ƑS�������D��������Q�Ƃ��Ċm�F���Ăق����D
class Mouse final : public Singleton<Mouse>
{
public:

	//! @brief �}�E�X���͂��X�V����D����𖈃t���[�����s���Ȃ��ƁC�}�E�X���͂��擾�ł��Ȃ��D
	void update();

	//! @brief �}�E�X�J�[�\���̈ʒu���擾����DX���W�͉�ʂ̍��[��0�Ƃ��āC�E���������D�����Dxlib�̎d�l�Ȃ̂ŕύX�s�\�D
	//! @return int �}�E�X�J�[�\����X���W�D
	int getPosX() const;

	//! @brief �}�E�X�J�[�\���̈ړ��ʂ��擾����DX���W�͉�ʂ̍��[��0�Ƃ��āC�E���������D�����Dxlib�̎d�l�Ȃ̂ŕύX�s�\�D
	//! @return int �}�E�X�J�[�\����X�����̈ړ��ʁD
	int getDiffPosX() const;

	//! @brief �}�E�X�J�[�\���̈ʒu���擾����DY���W�͉�ʂ̏�[��0�Ƃ��āC�����������D�����Dxlib�̎d�l�Ȃ̂ŕύX�s�\�D
	//! @return int �}�E�X�J�[�\����Y���W�D
	int getPosY() const;

	//! @brief �}�E�X�J�[�\���̈ړ��ʂ��擾����DY���W�͉�ʂ̏�[��0�Ƃ��āC�����������D�����Dxlib�̎d�l�Ȃ̂ŕύX�s�\�D
	//! @return int �}�E�X�J�[�\����Y�����̈ړ��ʁD
	int getDiffPosY() const;

	//! @brief �}�E�X�J�[�\���̈ړ��ʂ��擾����D
	//! @return double �}�E�X�J�[�\���̈ړ��ʁD
	double getDiffPos() const;

	//! @brief �}�E�X�̍��N���b�N�̉����ꑱ���Ă���t���[�������擾����D
	//! @return int ������Ă���t���[�����D
	int getPushingCountLeft() const;

	//! @brief �}�E�X�̉E�N���b�N�̉����ꑱ���Ă���t���[�������擾����D
	//! @return int ������Ă���t���[�����D
	int getPushingCountRight() const;

	//! @brief �}�E�X�̃z�C�[���{�^���̉����ꑱ���Ă���t���[�������擾����D
	//! @return int ������Ă���t���[�����D
	int getPushingCountMiddle() const;

	//! @brief �}�E�X�̍��N���b�N�̗����ꑱ���Ă���t���[�������擾����D
	//! @return int ������Ă���t���[�����D
	int getReleasingCountLeft() const;

	//! @brief �}�E�X�̉E�N���b�N�̗����ꑱ���Ă���t���[�������擾����D
	//! @return int ������Ă���t���[�����D
	int getReleasingCountRight() const;

	//! @brief �}�E�X�̃z�C�[���{�^���̗����ꑱ���Ă���t���[�������擾����D
	//! @return int ������Ă���t���[�����D
	int getReleasingCountMiddle() const;

	//! @brief �}�E�X�̃z�C�[���̉�]�ʂ��擾����D
	//! @n 1�t���[���ŉ�]�����ʂ��擾����D
	//! @n ��O�ɉ񂵂����̓}�C�i�X�̒l�Ƃ��āA���ɉ񂵂����̓v���X�̒l�Ƃ��ĕԂ�
	//! @return int �}�E�X�z�C�[���̉�]�ʁD
	int getWheelRot() const;

private:

	Mouse();
	friend Singleton<Mouse>;

	//�N���b�N�J�E���^
	int m_posx, m_poy;
	int m_past_posx, m_past_posy;
	int m_pushing_count_right, m_pushing_count_left, m_pushing_count_middle;
	int m_releasing_count_right, m_releasing_count_left, m_releasing_count_middle;
	int m_wheel_rot;
};


//! @file mouse.h
//! @date 2023/08/07
//! @author ���J��
//! @brief Dxlib�̃}�E�X���͂��擾����N���X�D
//! @n �s�� : @lineinfo
