#include "GraphTreeCreatorHato.h"
#include "NodeEdit.h"
#include "Define.h"

bool GraphTreeCreatorHato::createGraphTree(const SNode& _current_node, const MapState* const _p_map, std::vector<SNode>& _output_graph)
{
	//�}�b�v�̃|�C���^���󂯎��D
	mp_Map = _p_map;

	//���݂̃m�[�h��e�ɂ���D
	SNode _parent_node = _current_node;

	node_edit::changeParentNode(_parent_node);
	_output_graph.clear();					//�o�͂��錋�ʂ���ɂ���D
	_output_graph.push_back(_parent_node);	//�e��ǉ�����D

	int _cnt = 0;	//�J�E���^��p��

	//�J�E���^��vector�̃T�C�Y�𒴂���܂Ń��[�v����D
	while (_cnt < _output_graph.size())
	{
		//�T���[��������Ă��Ȃ��m�[�h�ɂ̂ݏ���������D
		if (_output_graph.at(_cnt).depth < Define::GRAPH_SEARCH_DEPTH)
		{
			//���݂̃m�[�h���玟�̓���𐶐����āC���ɂǂ�ǂ�ǉ�����D
			pushNewNodesByCurrentNode(_output_graph.at(_cnt), _cnt, _output_graph);
		}

		_cnt++;	//�J�E���^��i�߂�D
	}

	return true;
}


void GraphTreeCreatorHato::pushNewNodesByCurrentNode(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph)
{
	switch (_current_node.next_move)
	{
	case EHexapodMove::LEG_UP_DOWN:

		//�r���㉺�ړ������C�ڒn������V�r�����肷��D
		break;

	case EHexapodMove::LEG_HIERARCHY_CHANGE:

		//�r�̊K�w��ύX����DLegState��ύX���C�r�𕽍s�ړ�����D
		m_LegHierarchy.create(_current_node, _current_num, _output_graph);
		break;

	case EHexapodMove::COM_TRANSLATION:

		//�d�S�𕽍s�ړ�����D
		break;

	case EHexapodMove::COM_UP_DOWN:

		m_ComUpDown.create(_current_node, _current_num, _output_graph);
		break;

	default:

		//��`����Ă��Ȃ��Ȃ�΁C�����m�[�h�����̂܂ܒǉ�����D
		SNode _new_node = _current_node;

		node_edit::changeNextNode(_new_node, _current_num, _current_node.next_move);

		_output_graph.push_back(_new_node);
		break;
	}
}
