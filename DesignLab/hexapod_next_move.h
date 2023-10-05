//! @file hexapod_next_move.h
//! @brief ���{�b�g�����ɂǂ̓��������̂���\���񋓑́D


#ifndef DESIGNLAB_HEXAPOD_NEXT_MOVE_H_
#define DESIGNLAB_HEXAPOD_NEXT_MOVE_H_


//! @enum HexapodMove
//! @brief ���{�b�g�����ɂǂ̓��������̂���\���񋓑́D
//! @n ��s�����ł� int�^�̕ϐ� debug ��p���Ă������C�ǐ����Ⴂ�̂ŗ񋓑̂�p����D
enum class HexapodMove : int
{
	kNone,						//!< ������������Ȃ��D
	kLegUpDown,					//!< �r�̏㉺�ړ�.
	kLegUp,						//!< �r���グ��C�V�r����D
	kLegDown,					//!< �r��������C�ڒn����D
	kLegHierarchyChange,		//!< �r�̕��s�ړ��D�r�̊K�w��ύX����D
	kComMove,					//!< �d�S�̕��s�ړ��DCenter Of Mass�ŏd�S�̂��ƁD
	kComUpDown,					//!< �d�S�̏㉺�ړ��D
	kLegUpDownNextComMove,		//!< �r�̏㉺�ړ��D(���͏d�S�̕��s�ړ�)
	kLegUpDownNextComUpDown,	//!< �r�̏㉺�ړ��D(���͏d�S�̏㉺�ړ�)
};


#endif	// DESIGNLAB_HEXAPOD_NEXT_MOVE_H_