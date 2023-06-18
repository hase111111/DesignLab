//! @file Singleton.h
//! @brief Singleton�N���X�̎����D
//! @author ���J��

#pragma once

//! @class Singleton
//! @brief ���̃N���X���p�������Singleton�N���X�ɂȂ�D�O���t�T���ł͐�΂Ɏg��Ȃ�����!
//! @details Singleton�N���X�Ƃ́CC����ł����Ƃ���̃O���[�o���ϐ��i�ǂ�����ł��l��ύX�ł���ϐ��j�ł���D<br>
//! Singleton�p�^�[���͈������\�z�̈�Ȃ̂łȂ�ׂ��g��Ȃ��ق����悢�D<br>
//! ���̃v���W�F�N�g�ł͉摜�\���N���X�ŃL�[�{�[�h�ƃ}�E�X�̓��͂��Ǘ����邽�߂Ɏg�p���Ă���D<br> <br>
//! �O���t�T���ł�""���""�Ɏg��Ȃ�����!! <br> ��΂���!!!!
//! @author ���J��
template <typename _T>
class Singleton 
{
protected:
    Singleton() = default;
    virtual ~Singleton() = default;
    Singleton(const Singleton& r) = default;
    Singleton& operator=(const Singleton& r) = default;

public:

	static _T* getIns() 
	{
		static _T inst;
		return &inst;
	};

};
