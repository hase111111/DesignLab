#pragma once

#include <string>


//! @enum EDiscreteLegPos
//! @date 2023/09/10
//! @author ���J��
//! @breif ���U�����ꂽ�r�ʒu��\��enum
enum class EDiscreteLegPos
{
	LOWER_BACK = 1,		//!< ���݂̈ʒu������������ɂ���
	BACK,				//!< ���݂̈ʒu������ɂ���
	UPPER_BACK,			//!< ���݂̈ʒu�����������ɂ���
	CENTER,				//!< ���݂̈ʒu�ɂ���
	LOWER_FRONT,		//!< ���݂̈ʒu���O���������ɂ���
	FRONT,				//!< ���݂̈ʒu���O���ɂ���
	UPPER_FRONT,		//!< ���݂̈ʒu���O��������ɂ���
	ERROR_POS			//!< �G���[
};


namespace std
{
	std::string to_string(EDiscreteLegPos leg_pos);

}	// namespace std



//! @file discrete_leg_pos.h
//! @date 2023/09/12
//! @author ���J��
//! @breif ���U�����ꂽ�r�ʒu��\���񋓑�
