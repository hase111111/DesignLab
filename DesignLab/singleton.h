//! @file singleton.h
//! @brief Singleton�N���X�쐬�̂��߂̃e���v���[�g�N���X�D

#ifndef DESIGNLAB_SINGLETON_H_
#define DESIGNLAB_SINGLETON_H_


//! @class Singleton
//! @brief Singleton�N���X�쐬�̂��߂̃e���v���[�g�N���X�D
//! @details ���̃N���X���p�������Singleton�N���X�ɂȂ�DSingleton�N���X�Ƃ́CC����ł����Ƃ���̃O���[�o���ϐ��i�ǂ�����ł��l��ύX�ł���ϐ��j�ł���D
//! ���p����ƃv���O���������G�ɂȂ肪���Ȃ̂ŁC�O���t�T���ł͐�΂Ɏg��Ȃ����ƁD
//! ���̃v���W�F�N�g�ł͉摜�\���N���X�ŃL�[�{�[�h�ƃ}�E�X�̓��͂��Ǘ����邽�߂Ɏg�p���Ă���D
//! �Q�l https://dixq.net/rp2/ 
//! @see FontLoader
template <typename _T>
class Singleton
{
public:

	//! @brief �C���X�^���X���擾����D
	//! @n ���̃N���X���p�������N���X�� �N���X��::getIns()-> �̌`���Ń����o�֐����Ăяo���D
	//! @return _T* �C���X�^���X�̃|�C���^
	static _T* GetIns()
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


#endif // DESIGNLAB_SINGLETON_H_