#include "fps.h"

#include <cmath>
#include <string>

#include "DxLib.h"

#include "Define.h"


void Fps::wait()
{
	//�҂ׂ����Ԃ��擾���đ҂�
	int wait_time = 0;

	if (getWaitTime(&wait_time) == true)
	{
		WaitTimer(wait_time);  //�擾�������ԕ��҂�
		//Sleep(wait_time);    //windows API��

		regist(GetNowCount());  //���݂̎������L�^����
	}
	else
	{
		//���ԃI�[�o�[���Ă���̂ŁC�R�}�����̏���������D

		//���̃t���[���͗��z�I�ȏ������������̂Ƃ��āC�L�^����
		regist(m_list.back() + ONE_FRAME_MILLI_SECOND);

		m_do_skip_draw = true;     //�`����΂��t���O�𗧂Ă�
	}
}


bool Fps::skipDrawScene()
{
	//�X�L�b�v�t���O�������Ă���Ȃ�΁C���̃t���O��܂�C�V�[�����X�L�b�v����
	if (m_do_skip_draw == true)
	{
		m_do_skip_draw = false;
		return true;
	}

	return false;
}


void Fps::regist(const int now_time)
{
	m_list.push_back(now_time);   //���݂̎������L��

	if (m_list.size() > LIST_MAX)
	{
		//�킩��R�ꂽ��폜����
		m_list.pop_front();
	}
}


bool Fps::getWaitTime(int* time) const
{
	//������������
	(*time) = 0;

	//�������X�g����Ȃ�CWait���Ԃ�0�b
	if (m_list.empty() == true)
	{
		(*time) = 0;
		return true;
	}

	int actually_took_time = GetNowCount() - m_list.back();   //���ۂɂ����������Ԃ����߂�

	//�v�Z�ォ����ׂ����� - ���ۂɂ����������ԁ@�͂��Ȃ킿�҂ׂ�����
	int wait_time = ONE_FRAME_MILLI_SECOND - actually_took_time;

	//�҂����Ԃ����̒l�ł���Ƃ�(�܂�C������ׂ����Ԃ����ۂɂ����������Ԃ���������)�͂��̂܂ܒl��Ԃ��D
	if (wait_time >= 0)
	{
		(*time) = wait_time;
		return true;
	}

	//�҂����Ԃ����̒l�ł���Ƃ�
	else
	{
		if ((int)abs(wait_time) < ONE_FRAME_MILLI_SECOND)
		{
			//�P�t���[���ȏ�x��Ă��Ȃ��Ȃ�΁C�������s���D
			return false;
		}
	}

	//�ǂ�ɂ�����������Ȃ������ꍇ�O��Ԃ�
	(*time) = 0;
	return true;
}