//! @file hexapod_next_move.h
//! @brief ���{�b�g�����ɂǂ̓��������̂���\���񋓑́D

#ifndef DESIGNLAB_HEXAPOD_NEXT_MOVE_H_
#define DESIGNLAB_HEXAPOD_NEXT_MOVE_H_


#include <string>


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


namespace std
{
	//! @brief ���{�b�g�̓���𕶎���ɕϊ�����D
	//! @param [in] move ���{�b�g�̓���
	//! @return std::string ���{�b�g�̓����\��������
	//! @details �I�[�o�[���[�h�ɂ��Ă͈ȉ����Q�ƁD
	//! @n �Q�l: https://www.s-cradle.com/developer/sophiaframework/tutorial/Cpp/overload.html
	//! @n std::to_string��C++11����ǉ����ꂽ�֐��D#include <string>�Ŏg�p�\�ɂȂ�D
	//! @n �����e���ʓ|�Ȃ̂ŁC�ǂ����@��W��
	std::string to_string(HexapodMove move);
}


#endif	// DESIGNLAB_HEXAPOD_NEXT_MOVE_H_