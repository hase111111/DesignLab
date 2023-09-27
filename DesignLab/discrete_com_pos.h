//! @file discrete_com_pos.h
//! @brief ���U�����ꂽ�d�S�ʒu��\���񋓑�


#ifndef DESIGNLAB_DISCRETE_COM_POS_H_
#define DESIGNLAB_DISCRETE_COM_POS_H_


#include <string>


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
	ERROR_POS		//!< �G���[
};



// �͈̓��[�v�����邽�߂̋L�q�Chttps://sanichi999.hateblo.jp/entry/2014/12/19/011820
inline DiscreteComPos begin(DiscreteComPos) { return DiscreteComPos::kFront; };
inline DiscreteComPos end(DiscreteComPos) { return DiscreteComPos::ERROR_POS; };
inline DiscreteComPos operator*(DiscreteComPos com_pos) { return com_pos; };
inline DiscreteComPos operator++(DiscreteComPos& com_pos)
{
	return com_pos = static_cast<DiscreteComPos>(std::underlying_type<DiscreteComPos>::type(com_pos) + 1);
};


#endif	// DESIGNLAB_DISCRETE_COM_POS_H_