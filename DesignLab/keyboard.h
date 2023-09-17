//! @file keyboard.h
//! @brief Dxlib�̃L�[���͂��擾����N���X

#ifndef DESIGNLAB_KEYBOARD_H_
#define DESIGNLAB_KEYBOARD_H_


#include <array>

#include "singleton.h"


//! @class Keyboard
//! @brief Dxlib�̃L�[���͂��擾����N���X�D
//! @details �ڂ��������̓��e�͂����� https://dixq.net/rp2/08.html 
//! @n
//! @n Keyboard�̓V���O���g���N���X�Ȃ̂Ŏ��̂𐶐����邱�Ƃ��ł��Ȃ��D
//! @n getIns()��p���Ċ֐����Ăяo���K�v������D
//! @n
//! @n ��) 
//! @n @t Keyboard::getIns()->Update() 
//! @n
//! @n �܂����ӓ_�Ƃ��ẮC�����܂�Dxlib�̏������s�����߁C�R�}���h���C���ł͌Ăяo���Ȃ��D
//! @n �Ƃ������Ăяo���邯��ǂ��܂����삷�邩�͕ۏ؂���Ȃ��D�v���I�ȃo�O��G���[��U������\��������̂ŌĂԂׂ��łȂ��D
//! @n �f���� std::cin ���g���ׂ��D( C����̊֐��� scanf �͎g�p���ׂ��łȂ�)

class Keyboard final : public Singleton<Keyboard>
{
public:

	//! @brief �L�[���͂��X�V����D����𖈃t���[�����s���Ȃ��ƁC�L�[���͂��擾�ł��Ȃ��D
	void Update();

	//! @brief keyCode�̃L�[��������Ă���t���[�������擾����D
	//! @param [in] keyCode �ǂ̃L�[�𒲂ׂ������C�Ⴆ��Z�L�[��������CKEY_INPUT_Z�ƂȂ�D
	//! @return int ������Ă���t���[�����D�ُ��key_code���n���ꂽ�ꍇ��-1��Ԃ��D
	int GetPressingCount(const int key_code) const;

	//! @brief keyCode�̃L�[��������Ă���t���[�������擾����D
	//! @param [in] keyCode �ǂ̃L�[�𒲂ׂ������C�Ⴆ��Z�L�[��������CKEY_INPUT_Z�ƂȂ�D
	//! @return int ������Ă���t���[�����D�ُ��key_code���n���ꂽ�ꍇ��-1��Ԃ��D
	int GetReleasingCount(const int key_code) const;

private:

	Keyboard();
	friend Singleton< Keyboard >;

	static const int kKeyNum = 256;						//!< Dxlib�ɂ�����L�[����

	//! @brief keyCode���L���ȃL�[�ԍ����Ԃ�
	//! @param [in] keyCode �ǂ̃L�[�𒲂ׂ������C�Ⴆ��Z�L�[��������CKEY_INPUT_Z�ƂȂ�D
	//! @return bool �L���ȃL�[�ԍ��Ȃ�true�C�����łȂ����false�D
	bool IsAvailableCode(const int key_code) const;

	std::array<int, kKeyNum> key_pressing_counter_;	//������J�E���^
	std::array<int, kKeyNum> key_releasing_counter_;	//������J�E���^
};


#endif // !DESIGNLAB_KEYBOARD_H_