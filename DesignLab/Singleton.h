#pragma once

//���̃N���X���p�������Singleton�N���X�ɂȂ�D
//Singleton�p�^�[���͈������\�z�̈�Ȃ̂łȂ�ׂ��g��Ȃ��悤�ɂ��悤�D

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
