//! @file boot_mode.h
//! @brief �N�����[�h��\���񋓌^


#ifndef DESIGNLAB_BOOT_MODE_H_
#define DESIGNLAB_BOOT_MODE_H_


#include <string>


//! @enum BootMode
//! @brief �N�����[�h��\���񋓌^
enum class BootMode
{
	kSimulation,	//!< �V�~�����[�V�������[�h
	kViewer,		//!< �r���[���[���[�h
	kDisplayTest,	//!< �f�B�X�v���C�e�X�g���[�h
	kResultViewer	//!< ���U���g�r���[���[���[�h
};


namespace std
{
	//! @brief EBootMode�𕶎���ɕϊ�����
	//! @param [in] boot_mode �ϊ�����EBootMode
	//! @return EBootMode�𕶎���ɕϊ���������
	std::string to_string(const BootMode boot_mode);

	//! @brief �������EBootMode�ɕϊ�����
	//! @param [in] str �ϊ����镶����
	//! @return �������EBootMode�ɕϊ���������
	BootMode sToMode(const std::string str);

}	// namespace std


#endif	// DESIGNLAB_BOOT_MODE_H_