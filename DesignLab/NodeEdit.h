#pragma once
#include "Node.h"


//�m�[�h�̏�Ԃ�ҏW����֐����܂Ƃ߂����́D
namespace node_edit 
{
	//�n���ꂽ�q�m�[�h��e�m�[�h�ɕύX����D
	void changeParentNode(SNode& _node);

	//�n���ꂽ�m�[�h�̐[������[�����āC�e�Ǝ��̓����ݒ肷��D
	void changeNextNode(SNode& _node, const int _parent_num, const EHexapodMove _next_move);
}
