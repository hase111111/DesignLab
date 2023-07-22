#pragma once
#include "Singleton.h"
#include <array>


class Keyboard final : public Singleton<Keyboard> 
{
	Keyboard();
	friend Singleton< Keyboard >;

public:

	//! @brief �L�[���͂��X�V����D����𖈃t���[�����s���Ȃ��ƁC�L�[���͂��擾�ł��Ȃ��D
	void update();	
	
	//! @brief keyCode�̃L�[��������Ă���t���[�������擾����D
	//! @param [in] keyCode �ǂ̃L�[�𒲂ׂ������C�Ⴆ��Z�L�[��������CKEY_INPUT_Z�ƂȂ�D
	//! @return int ������Ă���t���[�����D
	int getPressingCount(int keyCode);

	//! @brief keyCode�̃L�[��������Ă���t���[�������擾����D
	//! @param [in] keyCode �ǂ̃L�[�𒲂ׂ������C�Ⴆ��Z�L�[��������CKEY_INPUT_Z�ƂȂ�D
	//! @return int ������Ă���t���[�����D
	int getReleasingCount(int keyCode);

private:
	static const int KEY_NUM = 256;				//�L�[����
    std::array<int, KEY_NUM> _pressingCount;	//������J�E���^
    std::array<int, KEY_NUM> _releasingCount;	//������J�E���^

	bool isAvailableCode(int keyCode);			//keyCode���L���ȃL�[�ԍ����₤
};


//! @file Keyboard.h
//! @brief Dxlib�̃L�[���͂��擾����N���X�̎����D
//! @author ���J��

//! @class Keyboard
//! @brief Dxlib�̃L�[���͂��擾����N���X�D
//! @details �ڂ��������̓��e�͂����� https://dixq.net/rp2/08.html <br>
//! <br>
//! Keyboard�̓V���O���g���N���X�Ȃ̂Ŏ��̂𐶐����邱�Ƃ��ł��Ȃ��D<br>
//! getIns()��p���Ċ֐����Ăяo���K�v������D<br>
//! <br>
//! ���ӓ_�Ƃ��ẮC�����܂�Dxlib�̏������s�����߁C�R�}���h���C���ł͌Ăяo���Ȃ��D<br>
//! �Ƃ������Ăяo���邯��ǂ��܂����삷�邩�͕ۏ؂���Ȃ��D�v���I�ȃo�O��G���[��U������\��������̂ŌĂԂׂ��łȂ��D<br>
//! std::cin�Ƃ����g���ׂ��D
//! @author ���J��
