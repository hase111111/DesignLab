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
};



// �͈̓��[�v�����邽�߂̋L�q�Chttps://sanichi999.hateblo.jp/entry/2014/12/19/011820
inline EDiscreteLegPos begin(EDiscreteLegPos) { return EDiscreteLegPos::LOWER_BACK; };
inline EDiscreteLegPos end(EDiscreteLegPos) { return EDiscreteLegPos::UPPER_FRONT; };
inline EDiscreteLegPos operator*(EDiscreteLegPos leg_pos) { return leg_pos; };
inline EDiscreteLegPos operator++(EDiscreteLegPos& leg_pos)
{
	return leg_pos = static_cast<EDiscreteLegPos>(std::underlying_type<EDiscreteLegPos>::type(leg_pos) + 1);
};



namespace std
{
	std::string to_string(EDiscreteLegPos leg_pos);

}	// namespace std



//! @file discrete_leg_pos.h
//! @date 2023/09/12
//! @author ���J��
//! @breif ���U�����ꂽ�r�ʒu��\���񋓑�
