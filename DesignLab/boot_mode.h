//! @file boot_mode.h
//! @brief �N�����[�h��\���񋓌^


#ifndef DESIGNLAB_BOOT_MODE_H_
#define DESIGNLAB_BOOT_MODE_H_


//! @enum BootMode
//! @brief �N�����[�h��\���񋓌^
enum class BootMode : int
{
	kSimulation = 0,//!< �V�~�����[�V�������[�h
	kViewer,		//!< �r���[���[���[�h
	kDisplayTest,	//!< �f�B�X�v���C�e�X�g���[�h
	kResultViewer	//!< ���U���g�r���[���[���[�h
};


#endif	// DESIGNLAB_BOOT_MODE_H_