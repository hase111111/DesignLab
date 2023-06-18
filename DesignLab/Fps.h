//! @file Fps.h
//! @brief Fps�N���X�̎����D
//! @author ���J��

#pragma once
#include <list>
#include "GraphicConst.h"

//! @class Fps
//! @brief FPS�����ɃL�[�v���邽�߂̃N���X�D
//! @details ��{�I�ɂ͉������Ȃ��Ƃ�FPS�͈��ɂȂ�̂����ǁC144Fps�Ƃ�240Fps���炢�̃Q�[�~���O���j�^�[���g���Ă���ꍇ�C��ʂ����������Ȃ邱�Ƃ�����D
//! ����Ă��̃N���X���g����FPS�𐧌䂷��DFPS�� Frames per Second�F1�b������̉�ʍX�V�񐔂̂��ƁDFirst Person Shooting�̂��Ƃł͂Ȃ��D<br>
//! ������PC�̃��j�^�[�̖��Ȃ̂ŁC�K�v�Ȃ��Ȃ�Ώ����Ă��܂��Ă����܂�Ȃ����C�������Ƃ���Ŗ��͂Ȃ��̂ŕ��u���Ă����Ă�����č\��Ȃ��D<br> <br>
//! [�Q�l] <br> FPS�̐�����s�� https://dixq.net/rp2/07.html �����̃T�C�g�̃v���O�����������ԃp�N���Ă��邪�C�@�\��F�X�ǉ����Ă���D<br>
//!  Dxlib��FPS�ƃ��t���b�V�����[�g�ɂ��� https://dixq.net/forum/viewtopic.php?t=20224
class Fps final
{
public:
    Fps() = default;
    ~Fps() = default;

    //! ��������������ꍇ�CFPS�����ɂ��邽�߂ɑ҂D
    void wait();

    //! 60Hz�ȏ�̃��j�^�[�g�p���ɏ������l�܂��ĉ�ʂ�������Ȃ��悤�ɁC�`�揈�����X�L�b�v���邩�ǂ����̃t���O�D
    //! @return bool �������l�܂��ĕ`����΂������Ƃ���true��Ԃ��D
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

