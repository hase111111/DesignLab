//! @file cassert_define.h
//! @brief �f�o�b�O���[�h�ƃ����[�X���[�h�ŃA�T�[�g��L�����E���������邽�߂̃w�b�_�t�@�C��
//! 
//! @details Visual Studio�ł́C�f�o�b�O���[�h�ł̓A�T�[�g��L�������C�����[�X���[�h�ł̓A�T�[�g�𖳌�������D
//! @n �������C���̃v���O�����͏������d�������ŁC���ɂ���Ă͂��������f�o�b�O���[�h�Ŏ��s���邱�Ƃ����
//! @n �Ȃ̂ŁC�����[�X���[�h�ŃA�T�[�g���o�������ꍇ�́C�ȉ��̃����[�X���[�h�̂Ƃ����
//! @n #undef NDEBUG �ŃA�T�[�g��L�������邱�ƁD
//! @n NDEBUG��define����Ă���Ƃ��́Cassert�𖳌�������D
//! @n #undef ��define����Ă�����̂𖳌�������D
//! @n �܂�C#undef NDEBUG �́Cassert��L��������D
//! @n �t�ɁC#define NDEBUG �́Cassert�𖳌�������D
//! @n �����̏����́Ccassert�̃C���N���[�h���O�ɍs���K�v������D
//! @n ����Ă��̂悤�ȃw�b�_�t�@�C�����쐬�����D


#ifndef DESIGNLAB_CASSERT_DEFINE_H_
#define DESIGNLAB_CASSERT_DEFINE_H_


#ifndef _DEBUG	// if not define _DEBUG �܂�C�����[�X���[�h�̏ꍇ�C

// �A�T�[�g��L��������ꍇ�́C�ȉ��̍s�̃R�����g( // )���͂�������
#undef NDEBUG	

#endif 


#include <cassert>


#endif // !DESIGNLAB_CASSERT_DEFINE_H_