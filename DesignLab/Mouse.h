#pragma once
#include "Singleton.h"

class Mouse final : public Singleton<Mouse> 
{
private:
	Mouse();
	friend Singleton<Mouse>;

public:

	//! @brief �}�E�X���͂��X�V����D����𖈃t���[�����s���Ȃ��ƁC�}�E�X���͂��擾�ł��Ȃ��D
	void update();	

	//! @brief �}�E�X�J�[�\���̈ʒu���擾����DX���W�͉�ʂ̍��[��0�Ƃ��āC�E���������D�����Dxlib�̎d�l�Ȃ̂ŕύX�s�\�D
	//! @return int �}�E�X�J�[�\����X���W�D
	int getPosX() const;

	//! @brief �}�E�X�J�[�\���̈ʒu���擾����DY���W�͉�ʂ̏�[��0�Ƃ��āC�����������D�����Dxlib�̎d�l�Ȃ̂ŕύX�s�\�D
	//! @return int �}�E�X�J�[�\����Y���W�D
	int getPosY() const;

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

private:

	//�N���b�N�J�E���^
	int m_posx, m_poy;
	int m_pushing_count_right, m_pushing_count_left, m_pushing_count_middle;
	int m_releasing_count_right, m_releasing_count_left, m_releasing_count_middle;
};


//! @file Mouse.h
//! @brief Dxlib�̃}�E�X���͂��擾����N���X�̎����D
//! @author ���J��

//! @class Mouse
//! @brief Dxlib�̃}�E�X���͂��擾����N���X�D
//! @details ���ӓ_��Keyboard�N���X�ƑS�������D��������Q�Ƃ��Ċm�F���Ăق����D
//! @author ���J��
