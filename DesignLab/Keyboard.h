#pragma once

#include "Singleton.h"
#include <array>

//Dxlib�̃L�[���͂��擾����N���X�D�ڂ����͂����� https://dixq.net/rp2/08.html
class Keyboard final : public Singleton<Keyboard> 
{
	Keyboard();
	friend Singleton< Keyboard >;

public:
	bool update();	//�X�V
	int getPressingCount(int keyCode);//keyCode�̃L�[��������Ă���t���[�������擾
	int getReleasingCount(int keyCode);//keyCode�̃L�[��������Ă���t���[�������擾

private:
	static const int KEY_NUM = 256;	//�L�[����
    std::array<int, KEY_NUM> _pressingCount;//������J�E���^
    std::array<int, KEY_NUM> _releasingCount;//������J�E���^

	bool isAvailableCode(int keyCode);//keyCode���L���ȃL�[�ԍ����₤
};