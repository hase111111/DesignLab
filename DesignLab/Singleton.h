#pragma once

//���̃N���X���p�������Singleton�N���X�ɂȂ�D
//Singleton�p�^�[���͈������\�z�̈�Ȃ̂łȂ�ׂ��g��Ȃ��悤�ɂ��悤�D
//���̃v���W�F�N�g�ł͉摜�\���N���X�ŃL�[�{�[�h�ƃ}�E�X�̓��͂��Ǘ����邽�߂Ɏg�p���Ă��܂��D
//�O���t�T���ł͐�΂Ɏg��Ȃ�����!

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
