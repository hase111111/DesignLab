#include "HexapodNextMove.h"

std::string std::to_string(const EHexapodMove _move)
{
	switch (_move)
	{
	case EHexapodMove::COM_MOVE:
		return "�d�S�̕��s�ړ�";

	case EHexapodMove::COM_UP_DOWN:
		return "�d�S�̏㉺�ړ�";

	case EHexapodMove::LEG_HIERARCHY_CHANGE:
		return "�r�̊K�w�ύX";

	case EHexapodMove::LEG_UP_DOWN:
		return "�r�̏㉺�ړ�";

	default:
		return "����`����";
		break;
	}
}
