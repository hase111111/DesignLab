//! @file discrete_leg_pos.h
//! @brief ���U�����ꂽ�r�ʒu��\���񋓑�

#ifndef DESIGNLAB_DISCRETE_LEG_POS_H_
#define DESIGNLAB_DISCRETE_LEG_POS_H_


//! @enum DiscreteLegPos
//! @brief ���U�����ꂽ�r�ʒu��\��enum
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


#endif // DESIGNLAB_DISCRETE_LEG_POS_H_