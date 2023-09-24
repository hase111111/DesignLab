#include "keyboard.h"

#include <Dxlib.h>


Keyboard::Keyboard()
{
	for (int i = 0; i < kKeyNum; i++)
	{
		key_releasing_counter_[i] = 0;
		key_pressing_counter_[i] = 0;
	}
}


void Keyboard::Update()
{
	char now_key_status[kKeyNum];
	GetHitKeyStateAll(now_key_status);       //���̃L�[�̓��͏�Ԃ��擾

	for (int i = 0; i < kKeyNum; i++)
	{
		if (now_key_status[i] != 0)
		{
			//i�Ԃ̃L�[��������Ă�����

			if (key_releasing_counter_[i] > 0)
			{
				//������J�E���^��0���傫�����
				key_releasing_counter_[i] = 0;   //0�ɖ߂�
			}

			key_pressing_counter_[i]++;          //������J�E���^�𑝂₷
		}
		else
		{
			//i�Ԃ̃L�[��������Ă�����
			if (key_pressing_counter_[i] > 0)
			{
				//������J�E���^��0���傫�����
				key_pressing_counter_[i] = 0;    //0�ɖ߂�
			}

			key_releasing_counter_[i]++;         //������J�E���^�𑝂₷
		}
	}
}


int Keyboard::GetPressingCount(const int key_code) const
{
	if (!IsAvailableCode(key_code))
	{
		return -1;
	}

	return key_pressing_counter_[key_code];
}


int Keyboard::GetReleasingCount(const int key_code) const
{
	if (!IsAvailableCode(key_code))
	{
		return -1;
	}

	return key_releasing_counter_[key_code];
}


bool Keyboard::IsAvailableCode(const int key_code) const
{
	if (!(0 <= key_code && key_code < kKeyNum))
	{
		return false;
	}

	return true;
}
