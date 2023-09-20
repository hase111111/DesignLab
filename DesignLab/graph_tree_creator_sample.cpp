//#include "graph_tree_creator_sample.h"
//
//#include "graph_search_const.h"
//
//
//EGraphSearchResult GraphTreeCreatorSample::CreateGraphTree(const SNode& current_node, const DevideMapState& map_ref, std::vector<SNode>* output_graph)
//{
//	//�����ɃO���t���쐬���鏈���������D���̃N���X�̓T���v���Ȃ̂œ�������Ȃ��m�[�h������Ԃ��܂��D
//
//	(*output_graph).clear();	//�o�͂���O���t�����Z�b�g����D
//
//
//	//���݂̃m�[�h��e�ɂ���D
//	SNode parent_node = current_node;
//
//	parent_node.changeParentNode();
//	(*output_graph).push_back(parent_node);
//
//
//	//�ݒ肳�ꂽ�T���[���܂ł̐[�������O���t�����D���ۂɃO���t���쐬���鎞�������炭����Ȋ����Ń��[�v���鏈���������D
//
//	int cnt = 0;	//�J�E���^��p��
//
//	//�J�E���^��vector�̃T�C�Y�𒴂���܂Ń��[�v����D
//	while (cnt < (*output_graph).size())
//	{
//		//�T���[��������Ă��Ȃ��m�[�h�̂݁C����������D
//		if ((*output_graph)[cnt].depth < getMaxDepth())
//		{
//			SNode new_node = (*output_graph)[cnt];
//
//			//�����ɐV�����p���𐶐����鏈���������D����͉��̏����������Ɏ��̃m�[�h�Ƃ���D
//
//			new_node.changeNextNode(cnt, EHexapodMove::NONE);
//
//			//�ǉ�����D
//			(*output_graph).emplace_back(new_node);
//		}
//
//		cnt++;	//�J�E���^��i�߂�D
//	}
//
//
//	if (GraphSearchConst::MAX_NODE_NUM < static_cast<int>((*output_graph).size()))
//	{
//		return EGraphSearchResult::FailureByNodeLimitExceeded;
//	}
//
//	return EGraphSearchResult::Success;
//}
