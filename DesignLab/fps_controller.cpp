#include "fps_controller.h"

#include <cmath>
#include <string>

#include <Dxlib.h>


FpsController::FpsController(const int target_fps)
	: kTargetFpsValue(target_fps), kOneFrameTime((int)(1000.0 / target_fps)), kListMax(target_fps * 2),
	need_skip_draw_screen_(false)
{
}


void FpsController::Wait()
{
	if (!TargetFpsIsVaild()) { return; }


	//�҂ׂ����Ԃ��擾���đ҂�
	int wait_time = 0;

	if (CheckNeedSkipDrawScreen(&wait_time))
	{
		WaitTimer(wait_time);  //�擾�������ԕ��҂�
		//Sleep(wait_time);    //windows API��

		RegistTime(GetNowCount());  //���݂̎������L�^����
	}
	else
	{
		//���ԃI�[�o�[���Ă���̂ŁC�R�}�����̏���������D

		//���̃t���[���͗��z�I�ȏ������������̂Ƃ��āC�L�^����
		RegistTime(time_list_.back() + kOneFrameTime);

		need_skip_draw_screen_ = true;     //�`����΂��t���O�𗧂Ă�
	}
}


bool FpsController::SkipDrawScene()
{
	if (!TargetFpsIsVaild()) { return false; }


	//�X�L�b�v�t���O�������Ă���Ȃ�΁C���̃t���O��܂�C�V�[�����X�L�b�v����
	if (need_skip_draw_screen_)
	{
		need_skip_draw_screen_ = false;
		return true;
	}

	return false;
}


void FpsController::RegistTime(const int now_time)
{
	time_list_.push_back(now_time);   //���݂̎������L��

	if (time_list_.size() > kListMax)
	{
		//�킩��R�ꂽ��폜����
		time_list_.pop_front();
	}
}


bool FpsController::CheckNeedSkipDrawScreen(int* time) const
{
	//������������
	(*time) = 0;

	//�������X�g����Ȃ�CWait���Ԃ�0�b
	if (time_list_.empty())
	{
		(*time) = 0;
		return true;
	}

	int actually_took_time = GetNowCount() - time_list_.back();   //���ۂɂ����������Ԃ����߂�

	//�v�Z�ォ����ׂ����� - ���ۂɂ����������ԁ@�͂��Ȃ킿�҂ׂ�����
	int wait_time = kOneFrameTime - actually_took_time;

	if (wait_time >= 0)
	{
		//�҂����Ԃ����̒l�ł���Ƃ�(�܂�C������ׂ����Ԃ����ۂɂ����������Ԃ���������)�͂��̂܂ܒl��Ԃ��D

		(*time) = wait_time;
		return true;
	}
	else
	{
		//�҂����Ԃ����̒l�ł���Ƃ�

		if ((int)abs(wait_time) < kOneFrameTime)
		{
			//�P�t���[���ȏ�x��Ă��Ȃ��Ȃ�΁C�������s���D
			return false;
		}
	}

	//�ǂ�ɂ�����������Ȃ������ꍇ�O��Ԃ�
	(*time) = 0;
	return true;
}


bool FpsController::TargetFpsIsVaild() const
{
	//�}�C�i�X�̒l�͋��e���Ȃ�
	if (kTargetFpsValue <= 0)
	{
		return false;
	}

	//�P�b�ԂɂP�t���[���ȏ�͋��e���Ȃ�
	if (kTargetFpsValue > 60)
	{
		return false;
	}

	return false;
}
