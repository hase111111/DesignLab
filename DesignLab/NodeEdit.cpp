#include "NodeEdit.h"

void node_edit::changeParentNode(SNode& _node)
{
	_node.depth = 0;		//�[����0�ɂ���
	_node.parent_num = -1;	//���g���e�̂��߁C���̒l��������D
}
