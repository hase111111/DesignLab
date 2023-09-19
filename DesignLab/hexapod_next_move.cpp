#include "hexapod_next_move.h"


std::string std::to_string(const EHexapodMove move)
{
	switch (move)
	{
	case EHexapodMove::COM_MOVE:
		return "�d�S�̕��s�ړ�";

	case EHexapodMove::COM_UP_DOWN:
		return "�d�S�̏㉺�ړ�";

	case EHexapodMove::LEG_HIERARCHY_CHANGE:
		return "�r�̊K�w�ύX";

	case EHexapodMove::LEG_UP_DOWN:
		return "�r�̏㉺�ړ�";

	case EHexapodMove::LEG_UP_DOWN_NEXT_COM_MOVE:
		return "�r�̏㉺�ړ����d�S�̕��s�ړ�";

	case EHexapodMove::LEG_UP_DOWN_NEXT_COM_UP_DOWN:
		return "�r�̏㉺�ړ����d�S�̏㉺�ړ�";

	default:
		return "����`����";
		break;
	}
}
