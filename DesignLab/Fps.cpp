#include "DxLib.h"
#include "Fps.h"
#include <cmath>
#include <string>
#include "Define.h"

void Fps::wait()
{
    //�҂ׂ����Ԃ��擾���đ҂�
    int _wait_time = 0;

    if (getWaitTime(_wait_time) == true)
    {
        WaitTimer(_wait_time);  //�擾�������ԕ��҂�
        //Sleep(_wait_time);    //windows API��

        regist(GetNowCount());  //���݂̎������L�^����
    }
    else
    {
        //���ԃI�[�o�[���Ă���̂ŁC�R�}�����̏���������D

        //���̃t���[���͗��z�I�ȏ������������̂Ƃ��āC�L�^����
        regist(_list.back() + ONE_FRAME_MILLI_SECOND);

        m_skip_draw = true;     //�`����΂��t���O�𗧂Ă�
    }
}

bool Fps::skipDrawScene()
{
    //�X�L�b�v�t���O�������Ă���Ȃ�΁C���̃t���O��܂�C�V�[�����X�L�b�v����
    if (m_skip_draw == true)
    {
        m_skip_draw = false;
        return true;
    }

    return false;
}

void Fps::regist(const int _now_time)
{
    _list.push_back(_now_time);   //���݂̎������L��

    if (_list.size() > LIST_MAX) 
    {  
        //�킩��R�ꂽ��|�b�v
        _list.pop_front();
    }
}

bool Fps::getWaitTime(int& _time) const
{
    //������������
    _time = 0;

    //�������X�g����Ȃ�CWait���Ԃ�0�b
    if (_list.empty() == true)
    {
        _time = 0;
        return true;
    }

    int actuallyTookTime = GetNowCount() - _list.back();   //���ۂɂ����������Ԃ����߂�

    //�v�Z�ォ����ׂ����� - ���ۂɂ����������ԁ@�͂��Ȃ킿�҂ׂ�����
    int waitTime = ONE_FRAME_MILLI_SECOND - actuallyTookTime;

    //�҂����Ԃ����̒l�ł���Ƃ�(�܂�C������ׂ����Ԃ����ۂɂ����������Ԃ���������)�͂��̂܂ܒl��Ԃ��D
    if (waitTime >= 0)
    {
        _time = waitTime;
        return true;
    }

    //�҂����Ԃ����̒l�ł���Ƃ�
    else
    {
        if ((int)abs(waitTime) < ONE_FRAME_MILLI_SECOND)
        {
            //�P�t���[���ȏ�x��Ă��Ȃ��Ȃ�΁C�������s���D
            return false;
        }
    }

    //�ǂ�ɂ�����������Ȃ������ꍇ�O��Ԃ�
    _time = 0;
    return true;
}
