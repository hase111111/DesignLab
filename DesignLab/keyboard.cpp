#include "keyboard.h"

#include "DxLib.h"


Keyboard::Keyboard()
{
	for (int i = 0; i < KEY_NUM; i++)
	{
		m_releasing_count[i] = 0;
		m_pressing_count[i] = 0;
	}
}


void Keyboard::update()
{
	char now_key_status[KEY_NUM];
	GetHitKeyStateAll(now_key_status);       //���̃L�[�̓��͏�Ԃ��擾

	for (int i = 0; i < KEY_NUM; i++)
	{
		if (now_key_status[i] != 0)
		{
			//i�Ԃ̃L�[��������Ă�����

			if (m_releasing_count[i] > 0)
			{
				//������J�E���^��0���傫�����
				m_releasing_count[i] = 0;   //0�ɖ߂�
			}

			m_pressing_count[i]++;          //������J�E���^�𑝂₷
		}
		else
		{
			//i�Ԃ̃L�[��������Ă�����
			if (m_pressing_count[i] > 0)
			{
				//������J�E���^��0���傫�����
				m_pressing_count[i] = 0;    //0�ɖ߂�
			}

			m_releasing_count[i]++;         //������J�E���^�𑝂₷
		}
	}
}


int Keyboard::getPressingCount(int key_code) const
{
	if (!isAvailableCode(key_code))
	{
		return -1;
	}

	return m_pressing_count[key_code];
}


int Keyboard::getReleasingCount(int key_code) const
{
	if (!isAvailableCode(key_code))
	{
		return -1;
	}

	return m_releasing_count[key_code];
}


bool Keyboard::isAvailableCode(int key_code) const
{
	if (!(0 <= key_code && key_code < KEY_NUM))
	{
		return false;
	}

	return true;
}
