#pragma once
#include <string>

//���{�b�g�����ɂǂ̓��������̂���\���񋓑́D�T�C�Y�� 1 byte�Dsizeof�Ŋm�F�ς�
enum class EHexapodMove : char
{
	NONE = 0,					// ������������Ȃ�
	LEG_UP_DOWN = 1,			// �r�̏㉺�ړ�.
	LEG_HIERARCHY_CHANGE = 2,	// �r�̕��s�ړ��D�r�̊K�w��ύX����D
	COM_MOVE = 3,				// �d�S�̕��s�ړ��DCenter Of Mass�ŏd�S�̂��ƁD
	COM_UP_DOWN = 4,			// �d�S�̏㉺�ړ�
};

//to_string���I�[�o�[���C�h����
namespace std
{
	std::string to_string(const EHexapodMove _move);
}