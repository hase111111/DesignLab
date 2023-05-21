#pragma once
#include <list>
#include "GraphicConst.h"

// FPS�����ɃL�[�v���邽�߂̃N���X�D
// ��{�I�ɂ͉������Ȃ��Ƃ����ɂȂ�̂����ǁC144Fps�Ƃ�240Fps���炢�̃Q�[�~���O���j�^�[���g���Ă���ꍇ�C��ʂ����������Ȃ邱�Ƃ�����̂ł��̃N���X���g���Đ��䂵�܂��D
// ������PC�̃��j�^�[�̖��Ȃ̂ŁC�K�v�Ȃ����Ă����Ȃ�Ώ����Ă��܂��đ��v�ł��D
// �Q�l 
//  FPS�̐�����s�� https://dixq.net/rp2/07.html
//  Dxlib��FPS�ƃ��t���b�V�����[�g�ɂ��� https://dixq.net/forum/viewtopic.php?t=20224

class Fps final
{
public:
    Fps() = default;
    ~Fps() = default;

    //��������������ꍇ�CFPS�����ɂ��邽�߂ɑ҂D
    void wait();

    //�������l�܂��ĕ`����΂������Ƃ���true��Ԃ��D
    bool skipDrawScene();

private:
    std::list<int> _list;

    bool m_skip_draw = false;   //�R�}�������������邽�߂̃t���O

    //���݂̎������L�^����֐�
    void regist(const int _now_time);

    //�ǂꂾ���҂Ă΂悢���Ԃ��֐��D
    bool getWaitTime(int& _time) const;

    const int ONE_FRAME_MILLI_SECOND = (int)(1000.0 / GraphicConst::GRAPHIC_FPS);  //�P�t���[��������ɂ����鎞��(�~���b)
    const int LIST_MAX = (int)(GraphicConst::GRAPHIC_FPS * 2);                     //���X�g��2�b���̃f�[�^�����܂��Ă���
};

