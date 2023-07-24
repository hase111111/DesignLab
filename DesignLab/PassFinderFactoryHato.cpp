#include "PassFinderFactoryHato.h"
#include "GraphTreeCreatorHato.h"
#include "GraphSearcherHato.h"
#include "GraphSearchHato.h"
#include "LegUpDownNodeCreator.h"
#include "LegHierarchyNodeCreator.h"
#include "ComUpDownNodeCreator.h"
#include "ComMoveNodeCreator.h"

void PassFinderFactoryHato::createPassFinder(std::unique_ptr<IGraphTreeCreator>& _tree, std::unique_ptr<IGraphSearcher>& _searcher, const MapState* const _map)
{
	//�؂��쐬����N���X�̃}�b�v���쐬�D
	std::map<EHexapodMove, std::unique_ptr<INodeCreator>> _node_creator_map;
	_node_creator_map.emplace(EHexapodMove::LEG_HIERARCHY_CHANGE, std::make_unique<LegHierarchyNodeCreator>(_map, EHexapodMove::LEG_UP_DOWN_NEXT_COM_UP_DOWN));
	_node_creator_map.emplace(EHexapodMove::LEG_UP_DOWN_NEXT_COM_UP_DOWN, std::make_unique<LegUpDownNodeCreator>(_map, EHexapodMove::COM_UP_DOWN));
	_node_creator_map.emplace(EHexapodMove::COM_UP_DOWN, std::make_unique<ComUpDownNodeCreator>(_map, EHexapodMove::LEG_UP_DOWN_NEXT_COM_MOVE));
	_node_creator_map.emplace(EHexapodMove::LEG_UP_DOWN_NEXT_COM_MOVE, std::make_unique<LegUpDownNodeCreator>(_map, EHexapodMove::COM_MOVE));
	_node_creator_map.emplace(EHexapodMove::COM_MOVE, std::make_unique<ComMoveNodeCreator>(_map, EHexapodMove::LEG_HIERARCHY_CHANGE));

	//�؂��쐬����N���X�ƁC�؂�T������N���X���쐬�D
	std::unique_ptr<IGraphTreeCreator> p_creator = std::make_unique<GraphTreeCreatorHato>(_node_creator_map);
	std::unique_ptr<IGraphSearcher> p_searcher = std::make_unique<GraphSearcherHato>();

	//���������N���X�������ɑ���D
	_tree = std::move(p_creator);
	_searcher = std::move(p_searcher);
}
