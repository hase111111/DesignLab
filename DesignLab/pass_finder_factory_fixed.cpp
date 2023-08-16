#include "PassFinderFactoryFixed.h"

#include <iostream>

#include "GraphTreeCreatorHato.h"
#include "GraphSearcherHato.h"
#include "GraphSearchConst.h"
#include "leg_up_down_node_creator.h"
#include "leg_hierarchy_node_creator.h"
#include "com_up_down_node_creator.h"
#include "com_move_node_creator_hato.h"


void PassFinderFactoryFixed::createGraphTreeCreator(const MapState* const p_map, std::unique_ptr<IGraphTreeCreator>& tree)
{
	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[Factory] PassFinderFactoryHato: createGraphTreeCreator() �O���t�쐬�N���X�̍쐬�J�n\n\n"; }

	//�؂��쐬����N���X�̃}�b�v���쐬�D
	std::map<EHexapodMove, std::unique_ptr<INodeCreator>> node_creator_map;
	node_creator_map.emplace(EHexapodMove::LEG_HIERARCHY_CHANGE, std::make_unique<LegHierarchyNodeCreator>(p_map, EHexapodMove::LEG_UP_DOWN));
	node_creator_map.emplace(EHexapodMove::LEG_UP_DOWN, std::make_unique<LegUpDownNodeCreator>(p_map, EHexapodMove::COM_UP_DOWN));
	node_creator_map.emplace(EHexapodMove::COM_UP_DOWN, std::make_unique<ComUpDownNodeCreator>(p_map, EHexapodMove::COM_MOVE));
	node_creator_map.emplace(EHexapodMove::COM_MOVE, std::make_unique<ComMoveNodeCreatorHato>(p_map, EHexapodMove::LEG_HIERARCHY_CHANGE));

	//�؂��쐬����N���X�ƁC�؂�T������N���X���쐬�D
	std::unique_ptr<IGraphTreeCreator> p_creator = std::make_unique<GraphTreeCreatorHato>(node_creator_map);

	//���������N���X�������ɑ���D
	tree = std::move(p_creator);

	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[Factory] PassFinderFactoryHato: �O���t�쐬�N���X�̍쐬�I��\n"; }
}


void PassFinderFactoryFixed::createGraphSearcher(std::unique_ptr<IGraphSearcher>& searcher)
{
	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[Factory] PassFinderFactoryHato: createGraphSearcher() �O���t�T���N���X�̍쐬�J�n\n\n"; }

	std::unique_ptr<IGraphSearcher> p_searcher = std::make_unique<GraphSearcherHato>();
	searcher = std::move(p_searcher);

	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "\n[Factory] PassFinderFactoryHato: �O���t�T���N���X�̍쐬�I��\n"; }
}
