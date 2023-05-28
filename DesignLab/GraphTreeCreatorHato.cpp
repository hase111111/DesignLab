#include "GraphTreeCreatorHato.h"
#include "NodeEdit.h"
#include "Define.h"

bool GraphTreeCreatorHato::createGraphTree(const SNode& _current_node, const MapState* const _p_map, std::vector<SNode>& _output_graph)
{
	//�}�b�v�̃|�C���^���󂯎��D
	mp_Map = _p_map;

	m_ComUpDown.init(mp_Map);

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
			std::vector<SNode> _res_vec;	// _cnt�Ԗڂ̃m�[�h�̎q�m�[�h������x�N�^�[

			makeNewNodesByCurrentNode(_output_graph.at(_cnt), _cnt, _res_vec);		//�q�m�[�h�𐶐�����D

			for (const auto &i : _res_vec)
			{
				//�[��������ŁC�e���������ݒ肳��Ă�����̂̂ݒǉ�����D
				if (i.depth == (_output_graph.at(_cnt).depth + 1) && i.parent_num == _cnt)
				{
					_output_graph.push_back(i);		//�q�m�[�h�����ʂɒǉ�����D
				}
			}
		}

		_cnt++;	//�J�E���^��i�߂�D
	}

	return true;
}


void GraphTreeCreatorHato::makeNewNodesByCurrentNode(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph)
{
	_output_graph.clear();

	switch (_current_node.next_move)
	{

	//case EHexapodMove::LEG_UP_DOWN:
	//	//�r���㉺�ړ������C�ڒn������V�r�����肷��D

	//	break;


	//case EHexapodMove::LEG_HIERARCHY_CHANGE:
	//	//�r�̊K�w��ύX����DLegState��ύX���C�r�𕽍s�ړ�����D
	//	m_LegHierarchy.create(_current_node, _current_num, _output_graph);
	//	break;


	//case EHexapodMove::COM_MOVE:
	//	//�d�S�𕽍s�ړ�����D
	//	break;


	case EHexapodMove::COM_UP_DOWN:
		m_ComUpDown.create(_current_node, _current_num, _output_graph);
		break;

	case EHexapodMove::LEG_UP_DOWN:
	case EHexapodMove::LEG_HIERARCHY_CHANGE:
	case EHexapodMove::COM_MOVE:
	default:

		//��`����Ă��Ȃ��Ȃ�΁C�����m�[�h�����̂܂ܒǉ�����D
		SNode _new_node = _current_node;

		node_edit::changeNextNode(_new_node, _current_num, _current_node.next_move);

		_output_graph.push_back(_new_node);
		break;
	}
}