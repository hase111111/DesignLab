//! @file boot_mode_selecter.h
//! @brief �N�����[�h��I������N���X

#ifndef DESIGNLAB_BOOT_MODE_SELECTER_H_
#define DESIGNLAB_BOOT_MODE_SELECTER_H_

#include "boot_mode.h"


//! @class BootModeSelecter
//! @brief �N�����[�h��I������N���X
class BootModeSelecter final
{
public:

	BootModeSelecter();

	//! @brief �f�t�H���g�̋N�����[�h��ݒ肷��
	inline void SetDefaultBootMode(BootMode default_mode) { default_mode_ = default_mode; };

	//! @brief �N�����[�h��I������
	//! @return BootMode �N�����[�h
	BootMode SelectBootMode();

private:
	const int kBootModeNum;	//!< �N�����[�h�̍ő吔

	BootMode default_mode_;	//!< �f�t�H���g�̋N�����[�h
};

#endif