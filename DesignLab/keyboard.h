#pragma once

#include <array>

#include "singleton.h"


//! @class Keyboard
//! @date 2023/08/07
//! @author ���J��
//! @brief Dxlib�̃L�[���͂��擾����N���X�D
//! @details �ڂ��������̓��e�͂����� https://dixq.net/rp2/08.html @n
//! @n Keyboard�̓V���O���g���N���X�Ȃ̂Ŏ��̂𐶐����邱�Ƃ��ł��Ȃ��D
//! @n getIns()��p���Ċ֐����Ăяo���K�v������D@n
//! @n ��) @n @t Keyboard::getIns()->update() @n
//! @n �܂����ӓ_�Ƃ��ẮC�����܂�Dxlib�̏������s�����߁C�R�}���h���C���ł͌Ăяo���Ȃ��D
//! @n �Ƃ������Ăяo���邯��ǂ��܂����삷�邩�͕ۏ؂���Ȃ��D�v���I�ȃo�O��G���[��U������\��������̂ŌĂԂׂ��łȂ��D
//! @n �f���� std::cin ���g���ׂ��D( C����̊֐��� scanf �͎g�p���ׂ��łȂ�)
class Keyboard final : public Singleton<Keyboard>
{
public:

	//! @brief �L�[���͂��X�V����D����𖈃t���[�����s���Ȃ��ƁC�L�[���͂��擾�ł��Ȃ��D
	void update();

	//! @brief keyCode�̃L�[��������Ă���t���[�������擾����D
	//! @param [in] keyCode �ǂ̃L�[�𒲂ׂ������C�Ⴆ��Z�L�[��������CKEY_INPUT_Z�ƂȂ�D
	//! @return int ������Ă���t���[�����D
	int getPressingCount(const int key_code) const;

	//! @brief keyCode�̃L�[��������Ă���t���[�������擾����D
	//! @param [in] keyCode �ǂ̃L�[�𒲂ׂ������C�Ⴆ��Z�L�[��������CKEY_INPUT_Z�ƂȂ�D
	//! @return int ������Ă���t���[�����D
	int getReleasingCount(const int key_code) const;

private:

	Keyboard();
	friend Singleton< Keyboard >;

	static const int KEY_NUM = 256;				//�L�[����
	std::array<int, KEY_NUM> m_pressing_count;	//������J�E���^
	std::array<int, KEY_NUM> m_releasing_count;	//������J�E���^

	bool isAvailableCode(const int key_code) const;			//keyCode���L���ȃL�[�ԍ����₤
};


//! @file keyboard.h
//! @date 2023/08/07
//! @author ���J��
//! @brief Dxlib�̃L�[���͂��擾����N���X�̎����D
//! @n �s�� : @lineinfo
