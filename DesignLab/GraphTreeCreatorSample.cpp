#include "GraphTreeCreatorSample.h"
#include "graph_search_const.h"

EGraphSearchResult GraphTreeCreatorSample::createGraphTree(const SNode& current_node, const MapState* const p_map, std::vector<SNode>* output_graph, int* make_node_num)
{
	//�����ɃO���t���쐬���鏈���������D���̃N���X�̓T���v���Ȃ̂œ�������Ȃ��m�[�h������Ԃ��܂��D

	//���݂̃m�[�h��e�ɂ���D
	SNode parent_node = current_node;

	parent_node.changeParentNode();
	(*output_graph).push_back(parent_node);

	//�ݒ肳�ꂽ�T���[���܂ł̐[�������O���t�����D���ۂɃO���t���쐬���鎞�������炭����Ȋ����Ń��[�v���鏈���������D

	int cnt = 0;	//�J�E���^��p��

	//�J�E���^��vector�̃T�C�Y�𒴂���܂Ń��[�v����D
	while (cnt < (*output_graph).size())
	{
		//�T���[��������Ă��Ȃ��m�[�h�̂݁C����������D
		if ((*output_graph)[cnt].depth < getMaxDepth())
		{
			SNode new_node = (*output_graph)[cnt];

			//�����ɐV�����p���𐶐����鏈���������D����͉��̏����������Ɏ��̃m�[�h�Ƃ���D

			new_node.changeNextNode(cnt, EHexapodMove::NONE);

			//�ǉ�����D
			(*output_graph).emplace_back(new_node);
		}

		cnt++;	//�J�E���^��i�߂�D
	}

	(*make_node_num) = (int)(*output_graph).size();

	if ((*make_node_num) > GraphSearchConst::MAX_NODE_NUM || (*make_node_num) < 0)
	{
		return EGraphSearchResult::FailureByNodeLimitExceeded;
	}

	return EGraphSearchResult::Success;
}
