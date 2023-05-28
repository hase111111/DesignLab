#include "LegUpDownNodeCreator.h"
#include "NodeEdit.h"
#include "ComType.h"
#include "LegState.h"

void LegUpDownNodeCreator::create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph)
{
	//�܂��͏d�S�̕ω�����؂Ȃ����̂�ǉ�����D
	{
		SNode _same_node = _current_node;
		node_edit::changeNextNode(_same_node, _current_num, getNextMove(_current_node.next_move));
		_output_graph.push_back(_same_node);
	}


	//�r�̗V�r�E�ڒn�ɂ���Đ�����Ƃ肤��d�S��comtype�Ƃ��Ďd�����Ă���D(�ڂ�����Comtype.h���Q��)�D�܂��͑S��true�ɂ��Ă����D
	bool _is_able_type[ComType::COM_TYPE_NUM];

	for (int i = 0; i < ComType::COM_TYPE_NUM; i++) 
	{
		_is_able_type[i] = true; 
	}

	//�d�S�����݂ǂ��ɂ��邩(�O��肩�^�񒆂�...)�Ȃǂ̃p�����[�^�͂���com pattern�Ŏd�����Ă���D(�ڂ�����Comtype.h���Q��)�D������擾����D
	int _com_pattern = LegState::getComPatternState(_current_node.leg_state);

	//com pattern���Ƃ邱�Ƃ��ł��Ȃ�com type��S��false�ɂ���D
	ComType::checkAbleComTypeFromComPattern(_com_pattern, _is_able_type);



	//���ɋr���n�ʂɐڒn�\�����ׂ�D
	bool _is_groundable[HexapodConst::LEG_NUM];

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		//���łɐڒn���Ă���r�͐ڒn�\�Ɍ��܂��Ă���̂�true�ɂ���D
		if (LegState::isGrounded(_current_node.leg_state, i) == true) 
		{
			_is_groundable[i] = true; 
		}
		else 
		{
			_is_groundable[i] = false; 
		}
	}



}

EHexapodMove LegUpDownNodeCreator::getNextMove(const EHexapodMove& _last_move) const
{
	//�d�S�̏㉺�ړ������ꁨ�d�S�̕��s�ړ�
	//�r�̕��s�ړ������ꁨ�d�S�̏㉺�ړ�

	if (_last_move == EHexapodMove::COM_UP_DOWN) { return EHexapodMove::COM_MOVE; }
	else { return EHexapodMove::COM_UP_DOWN; }
}

bool LegUpDownNodeCreator::isGroundableLeg(const int _leg_num, const SNode& _current_node, SNode& _output_ground_pos)
{
	if (mp_Map == nullptr)return false;

	//Leg2�Ɣ�r���āC�ǂ��ɂ��邩�ɂ���Ĉȉ��̂悤�ɗ��U�����Ă���D
	switch (LegState::getLegState(_current_node.leg_state, _leg_num))
	{
	case 1:
		//���C��
		break;

	case 2:
		//���C��
		break;

	case 3:
		//���C��
		break;

	case 4:
		//�^��
		break;

	case 5:
		//�@�O�C��
		break;

	case 6:
		//�@�O�C��
		break;

	case 7:
		//�@�O�C��
		break;

	default:
		break;
	}

	return false;
}
