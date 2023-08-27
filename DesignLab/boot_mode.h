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
};



namespace std
{

	std::string to_string(const EBootMode boot_mode);



	EBootMode sToMode(const std::string str);


}	// namespace std



//! @file boot_mode.h
//! @date 2023/08/27
//! @author ���J��
//! @brief �N�����[�h��\���񋓌^
//! @n �s�� : @lineinfo