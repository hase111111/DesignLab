#pragma once
#include "Node.h"
#include <string>

//�m�[�h�̏�Ԃ�ҏW����֐����܂Ƃ߂����́D
namespace node_edit 
{
	// �m�[�h������������֐��D�ύX����m�[�h���Q�Ɠn���Ŏ󂯎��C��2�����Ń����_���ɕύX���邩�ǂ������߂�D
	void initNode(SNode& _node, const bool _do_random);

	//�n���ꂽ�q�m�[�h��e�m�[�h�ɕύX����D����_node��depth��0�ɁCparent_num��-1�ɏ���������D
	void changeParentNode(SNode& _node);

	//�n���ꂽ�m�[�h�̐[������[�����āC�e�Ǝ��̓����ݒ肷��D
	void changeNextNode(SNode& _node, const int _parent_num, const EHexapodMove _next_move);

	std::string getTextHexapodMove(const EHexapodMove _move);
}
