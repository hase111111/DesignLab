#include "GraphTreeCreatorThread.h"
#include "Define.h"

EGraphSearchResult GraphTreeCreatorThread::createGraphTree(const SNode& _current_node, const MapState* const _p_map, std::vector<SNode>& _output_graph, int& _make_node_num)
{
	//���݂̃m�[�h��e�ɂ���D
	SNode _parent_node = _current_node;

	_parent_node.changeParentNode();
	_output_graph.clear();					//�o�͂��錋�ʂ���ɂ���D
	_output_graph.push_back(_parent_node);	//�e��ǉ�����D

	int _cnt = 0;	//�J�E���^��p��

	//�J�E���^��vector�̃T�C�Y�𒴂���܂Ń��[�v����D
	while (_cnt < _output_graph.size())
	{
		//�T���[��������Ă��Ȃ��m�[�h�ɂ̂ݏ���������D
		if (_output_graph.at(_cnt).depth < Define::GRAPH_SEARCH_DEPTH)
		{
			std::vector<SNode> _res_vec;	// _cnt�Ԗڂ̃m�[�h�̎q�m�[�h������x�N�^�[

			makeNewNodesByCurrentNode(_output_graph.at(_cnt), _cnt, _res_vec);		//�q�m�[�h�𐶐�����D

			for (const auto& i : _res_vec)
			{
				_output_graph.push_back(i);		//�q�m�[�h�����ʂɒǉ�����D
			}
		}

		_cnt++;	//�J�E���^��i�߂�D
	}

	return EGraphSearchResult::Success;
}

void GraphTreeCreatorThread::makeNewNodesByCurrentNode(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph)
{
	_output_graph.clear();

	if (m_node_creator_map.count(_current_node.next_move) > 0)
	{
		m_node_creator_map.at(_current_node.next_move)->create(_current_node, _current_num, _output_graph);
		return;
	}
	else
	{
		//��`����Ă��Ȃ��Ȃ�΁C�����m�[�h�����̂܂ܒǉ�����D
		SNode _new_node = _current_node;

		_new_node.changeNextNode(_current_num, _current_node.next_move);

		_output_graph.push_back(_new_node);
	}
}
