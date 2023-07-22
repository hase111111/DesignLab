#include "GraphTreeCreatorSample.h"
#include "Define.h"

bool GraphTreeCreatorSample::createGraphTree(const SNode& _current_node, const MapState* const _p_map, std::vector<SNode>& _output_graph)
{
	//�����ɃO���t���쐬���鏈���������D���̃N���X�̓T���v���Ȃ̂œ�������Ȃ��m�[�h������Ԃ��܂��D

	//���݂̃m�[�h��e�ɂ���D
	SNode _parent_node = _current_node;

	_parent_node.changeParentNode();
	_output_graph.push_back(_parent_node);

	//�ݒ肳�ꂽ�T���[���܂ł̐[�������O���t�����D���ۂɃO���t���쐬���鎞�������炭����Ȋ����Ń��[�v���鏈���������D

	int _cnt = 0;	//�J�E���^��p��

	//�J�E���^��vector�̃T�C�Y�𒴂���܂Ń��[�v����D
	while (_cnt < _output_graph.size())
	{
		//�T���[��������Ă��Ȃ��m�[�h�̂݁C����������D
		if (_output_graph.at(_cnt).depth < Define::GRAPH_SEARCH_DEPTH)
		{
			SNode _new_node = _output_graph.at(_cnt);

			//�����ɐV�����p���𐶐����鏈���������D����͉��̏����������Ɏ��̃m�[�h�Ƃ���D

			_new_node.depth++;				//�[������[�����āC
			_new_node.parent_num = _cnt;	//�e�͌��ݏ������Ă���vector�ƂȂ�D

			//�ǉ�����D
			_output_graph.push_back(_new_node);
		}

		_cnt++;	//�J�E���^��i�߂�D
	}

	return true;
}
