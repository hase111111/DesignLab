#pragma once

#include <string>


//! @enum �d�S�p�^�[���̗񋓌^
//! @brief �d�S���ǂ��ɂ��邩��\���D
enum class EDiscreteComPos
{
	FRONT = 1,		//!< �d�S���O���ɂ���
	FRONT_LEFT,		//!< �d�S�����O���ɂ���
	BACK_LEFT,		//!< �d�S��������ɂ���
	BACK,			//!< �d�S������ɂ���
	BACK_RIGHT,		//!< �d�S���E����ɂ���
	FRONT_RIGHT,	//!< �d�S���E�O���ɂ���
	CENTER_FRONT,	//!< �d�S�������O���ɂ���D�O�p�`
	CENTER_BACK,	//!< �d�S����������ɂ���D�t�O�p�a
	ERROR_POS		//!< �G���[
};



// �͈̓��[�v�����邽�߂̋L�q�Chttps://sanichi999.hateblo.jp/entry/2014/12/19/011820
inline EDiscreteComPos begin(EDiscreteComPos) { return EDiscreteComPos::FRONT; };
inline EDiscreteComPos end(EDiscreteComPos) { return EDiscreteComPos::ERROR_POS; };
inline EDiscreteComPos operator*(EDiscreteComPos com_pos) { return com_pos; };
inline EDiscreteComPos operator++(EDiscreteComPos& com_pos)
{
	return com_pos = static_cast<EDiscreteComPos>(std::underlying_type<EDiscreteComPos>::type(com_pos) + 1);
};


namespace std
{

	//! @brief ���U�����ꂽ�d�S�ʒu�𕶎���ŏo�͂��邽�߂̊֐�
	//! @param[in] com_pos ���U�����ꂽ�d�S�ʒu
	//! @return ���U�����ꂽ�d�S�ʒu��\��������
	std::string to_string(EDiscreteComPos com_pos);
}