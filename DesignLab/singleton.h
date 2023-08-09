#pragma once


//! @class Singleton
//! @date 2023/08/07
//! @author ���J��
//! @brief ���̃N���X���p�������Singleton�N���X�ɂȂ�D�O���t�T���ł͐�΂Ɏg��Ȃ�����!
//! @details Singleton�N���X�Ƃ́CC����ł����Ƃ���̃O���[�o���ϐ��i�ǂ�����ł��l��ύX�ł���ϐ��j�ł���D
//! @n Singleton�p�^�[���͈������\�z�̈�Ȃ̂łȂ�ׂ��g��Ȃ��ق����悢�D
//! @n ���̃v���W�F�N�g�ł͉摜�\���N���X�ŃL�[�{�[�h�ƃ}�E�X�̓��͂��Ǘ����邽�߂Ɏg�p���Ă���D@n 
//! @n �O���t�T���ł�""���""�Ɏg��Ȃ�����!! @n ��΂���!!!! @n
//! @n �g�����ɂ��Ă� �p����� keyboard.h ���Q�Ƃ��Ă݂Ăق����D
template <typename _T>
class Singleton
{
public:

	//! @brief �C���X�^���X���擾����D@n
	//! ���̃N���X���p�������N���X�� �N���X��::getIns()-> �̌`���Ń����o�֐����Ăяo���D
	//! @return �C���X�^���X�̃|�C���^
	static _T* getIns()
	{
		static _T inst;
		return &inst;
	};

protected:

	//�R���X�g���N�^�C�f�X�g���N�^�C�R�s�[�R���X�g���N�^�C������Z�q�͊O������Ăяo���Ȃ��悤�ɂ���D
	Singleton() = default;
	virtual ~Singleton() = default;
	Singleton(const Singleton& r) = default;
	Singleton& operator=(const Singleton& r) = default;

};


//! @file singleton.h
//! @date 2023/08/07
//! @author ���J��
//! @brief Singleton�N���X�̂��߂̃e���v���[�g�N���X
//! @n �s�� : @lineinfo
