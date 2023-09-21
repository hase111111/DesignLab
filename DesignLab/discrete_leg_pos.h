//! @file discrete_leg_pos.h
//! @breif ���U�����ꂽ�r�ʒu��\���񋓑�

#ifndef DESIGNLAB_DISCRETE_LEG_POS_H_
#define DESIGNLAB_DISCRETE_LEG_POS_H_


#include <string>


//! @enum DiscreteLegPos
//! @breif ���U�����ꂽ�r�ʒu��\��enum
//! @n ��s�����ł� 1�`7��int�^�̐��l�ŕ\������Ă��邪�C�ǐ����グ�邽�߂�enum�ɂ���
//! @n �����̑��x�͕ς���͂�
//! @n ���U�����ꂽ�r�ʒu�� 3bit (0 �` 7)�͈̔͂ŕ\������邽�߁C������g������ꍇ�C
//! @n leg state��\���ϐ��̌^��ύX����K�v������
enum class DiscreteLegPos : int
{
	kLowerBack = 1,		//!< ���݂̈ʒu������������ɂ���
	kBack,				//!< ���݂̈ʒu������ɂ���
	kUpperBack,			//!< ���݂̈ʒu�����������ɂ���
	kCenter,			//!< ���݂̈ʒu�ɂ���
	kLowerFront,		//!< ���݂̈ʒu���O���������ɂ���
	kFront,				//!< ���݂̈ʒu���O���ɂ���
	kUpperFront,		//!< ���݂̈ʒu���O��������ɂ���
};


namespace std
{
	//! @brief DiscreteLegPos�𕶎���ɕϊ�����
	//! @n enum�𕶎���ɕϊ����邽�߁Cstd::to_string()���I�[�o�[���[�h����D
	//! @n DiscreteLegPos�̒l���������炱�������₷�K�v������D
	//! @n ���������e�i���X����ςȂ�ł����Ƃ������@�Ȃ������ˁH
	//! @param[in] leg_pos �ϊ�������DiscreteLegPos
	//! @return DiscreteLegPos�𕶎���ɕϊ���������
	std::string to_string(DiscreteLegPos leg_pos);

}	// namespace std


#endif // DESIGNLAB_DISCRETE_LEG_POS_H_