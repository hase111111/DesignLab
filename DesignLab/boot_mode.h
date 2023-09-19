#pragma once

#include <string>



//! @enum EBootMode
//! @date 2023/08/27
//! @author ���J��
//! @brief �N�����[�h��\���񋓌^
enum class EBootMode : int
{
	SIMULATION,		//!< �V�~�����[�V�������[�h
	VIEWER,			//!< �r���[���[���[�h
	DISPLAY_TEST,	//!< �f�B�X�v���C�e�X�g���[�h
	RESULT_VIEWER	//!< ���U���g�r���[���[���[�h
};



namespace std
{
	//! @brief EBootMode�𕶎���ɕϊ�����
	//! @param [in] boot_mode �ϊ�����EBootMode
	//! @return EBootMode�𕶎���ɕϊ���������
	std::string to_string(const EBootMode boot_mode);



	//! @brief �������EBootMode�ɕϊ�����
	//! @param [in] str �ϊ����镶����
	//! @return �������EBootMode�ɕϊ���������
	EBootMode sToMode(const std::string str);


}	// namespace std



//! @file boot_mode.h
//! @date 2023/08/27
//! @author ���J��
//! @brief �N�����[�h��\���񋓌^
//! @n �s�� : @lineinfo