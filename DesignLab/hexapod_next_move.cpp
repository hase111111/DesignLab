#include "hexapod_next_move.h"

#include "cassert_define.h"


std::string std::to_string(const HexapodMove move)
{
	switch (move)
	{
	case HexapodMove::kComMove:
		return "�d�S�̕��s�ړ�";

	case HexapodMove::kComUpDown:
		return "�d�S�̏㉺�ړ�";

	case HexapodMove::kLegHierarchyChange:
		return "�r�̊K�w�ύX";

	case HexapodMove::kLegUpDown:
		return "�r�̏㉺�ړ�";

	case HexapodMove::kLegUpDownNextComMove:
		return "�r�̏㉺�ړ����d�S�̕��s�ړ�";

	case HexapodMove::kLegUpDownNextComUpDown:
		return "�r�̏㉺�ړ����d�S�̏㉺�ړ�";

	default:
		assert(false);	//�����ɗ���ꍇ�C�����e�i���X����K�v������
		return "����`����";
		break;
	}
}
