#pragma once
#include <string>

//! @enum EHexapodMove
//! @brief ���{�b�g�����ɂǂ̓��������̂���\���񋓑́D�T�C�Y�� 1 byte�Dsizeof�Ŋm�F�ς�
enum class EHexapodMove : char
{
	NONE = 0,							//!< ������������Ȃ�
	LEG_UP_DOWN = 1,					//!< �r�̏㉺�ړ�.
	LEG_HIERARCHY_CHANGE = 2,			//!< �r�̕��s�ړ��D�r�̊K�w��ύX����D
	COM_MOVE = 3,						//!< �d�S�̕��s�ړ��DCenter Of Mass�ŏd�S�̂��ƁD
	COM_UP_DOWN = 4,					//!< �d�S�̏㉺�ړ�
	LEG_UP_DOWN_NEXT_COM_MOVE = 5,		//!< �r�̏㉺�ړ��D(���͏d�S�̕��s�ړ�)
	LEG_UP_DOWN_NEXT_COM_UP_DOWN = 6,	//!< �r�̏㉺�ړ��D(���͏d�S�̏㉺�ړ�)
};

namespace std
{
	//@brief to_string���I�[�o�[���[�h����D
	//@details �I�[�o�[���[�h�ɂ��Ă͈ȉ����Q�ƁD<br> 
	// �Q�l: https://www.s-cradle.com/developer/sophiaframework/tutorial/Cpp/overload.html <br>
	// std::to_string��C++11����ǉ����ꂽ�֐��D#include <string>�Ŏg�p�\�ɂȂ�D<br>
	std::string to_string(const EHexapodMove _move);
}