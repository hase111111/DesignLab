//! @file discrete_com_pos.h
//! @brief ���U�����ꂽ�d�S�ʒu��\���񋓑�


#ifndef DESIGNLAB_DISCRETE_COM_POS_H_
#define DESIGNLAB_DISCRETE_COM_POS_H_


//! @enum ���U�����ꂽ�d�S�ʒu��\���񋓌^
//! @brief �d�S���ǂ��ɂ��邩��\���D
enum class DiscreteComPos
{
	kFront = 1,		//!< �d�S���O���ɂ���
	kFrontLeft,		//!< �d�S�����O���ɂ���
	kBackLeft,		//!< �d�S��������ɂ���
	kBack,			//!< �d�S������ɂ���
	kBackRight,		//!< �d�S���E����ɂ���
	kFrontRight,	//!< �d�S���E�O���ɂ���
	kCenterFront,	//!< �d�S�������O���ɂ���D�O�p�`
	kCenterBack,	//!< �d�S����������ɂ���D�t�O�p�a
};


#endif	// DESIGNLAB_DISCRETE_COM_POS_H_