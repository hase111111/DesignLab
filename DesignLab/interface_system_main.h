//! @file interface_system_main.h
//! @brief �V�X�e���̃C���^�[�t�F�[�X�D


#ifndef DESIGNLAB_INTERFACE_SYSTEM_MAIN_H_
#define DESIGNLAB_INTERFACE_SYSTEM_MAIN_H_


class ISystemMain
{
public:

	virtual ~ISystemMain() = default;

	//! @brief ��v�ȏ������s���֐�
	virtual void Main()	= 0;
};


#endif