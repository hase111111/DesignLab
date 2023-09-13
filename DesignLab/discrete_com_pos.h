#pragma once

#include <string>


//! @enum �d�S�p�^�[���̗񋓌^
//! @brief �d�S���ǂ��ɂ��邩��\���D
enum class EDiscreteComPos : char
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


namespace std
{

	//! @brief ���U�����ꂽ�d�S�ʒu�𕶎���ŏo�͂��邽�߂̊֐�
	//! @param[in] com_pos ���U�����ꂽ�d�S�ʒu
	//! @return ���U�����ꂽ�d�S�ʒu��\��������
	std::string to_string(EDiscreteComPos com_pos);
}