#include "GraphSearchHato.h"
#include "GraphSearchInitializer.h"

bool GraphSearchHato::getNextNodebyGraphSearch(const SNode& _current_node, const MapState* const _p_map, const STarget& _target, SNode& _output_node)
{
	//�C�j�V�����C�U�[�N���X�ɏ����������Ă��炤�D
	GraphSearchInitializer _initializer;

	_initializer.init(mp_GraphTreeCreator, mp_GraphSearcher);

	//�������^�[���D2�̃N���X�̏������Ɏ��s�����Ȃ�΁C�����ɏI������D
	if (!mp_GraphTreeCreator) { return false; }
	if (!mp_GraphSearcher) { return false; }


	std::vector<SNode> _graph_tree;		//�O���t�T���ɗp����O���t�D

	//�܂��̓O���t���쐬����D_graph_tree �ϐ��Ɍ��ʂ��Q�Ɠn�������D
	if (mp_GraphTreeCreator->createGraphTree(_current_node, _p_map, _graph_tree) == false) { return false; }
	
	//���ɃO���t��]�����āC���̓�������肷��Dput_node �ϐ��Ɍ��ʂ��Q�Ɠn�������D
	if (mp_GraphSearcher->searchGraphTree(_graph_tree, _target, _output_node) == false) { return false; }

	return true;
}
