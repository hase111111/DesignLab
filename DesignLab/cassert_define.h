//! @file cassert_define.h
//! @brief �f�o�b�O���[�h�ƃ����[�X���[�h�ŃA�T�[�g��L�����E���������邽�߂̃w�b�_�t�@�C��
//! @details �f�o�b�O���[�h�ł̓A�T�[�g��L��������D�����[�X���[�h�ł̓A�T�[�g�𖳌�������D
//! @n ���̃v���O�����͏������d�������ŁC���ɂ���Ă̓f�o�b�O���[�h�Ŏ��s���邱�Ƃ����
//! @n �Ȃ̂ŁC�����[�X���[�h�ŃA�T�[�g���o�������ꍇ�́C�ȉ��̃����[�X���[�h�̂Ƃ����
//! @n #undef NDEBUG �ŃA�T�[�g��L�������邱�ƁD


#ifndef DESIGNLAB_CASSERT_DEFINE_H_
#define DESIGNLAB_CASSERT_DEFINE_H_

#if defined(_DEBUG)	// �f�o�b�O���[�h�̏ꍇ

#undef NDEBUG		// �A�T�[�g��L��������

#else				// �����[�X���[�h�̏ꍇ

#undef NDEBUG		// �A�T�[�g��L��������
//#define NDEBUG	// �A�T�[�g�𖳌�������

#endif 

#include <cassert>

#endif // !DESIGNLAB_CASSERT_DEFINE_H_