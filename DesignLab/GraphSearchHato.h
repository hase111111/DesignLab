#pragma once
#include "MapState.h"
#include "Node.h"
#include "InterfaceGraphTreeCreator.h"
#include "InterfaceGraphSearcher.h"
#include "InterfaceGraphSearch.h"
#include <memory>

class GraphSearchHato final : public IGraphSearch
{
public:
	GraphSearchHato() = default;
	~GraphSearchHato() = default;

	EGraphSearchResult getNextNodebyGraphSearch(const SNode& _current_node, const MapState* const _p_map, const STarget& _target, SNode& _output_node) override;

private:
	std::unique_ptr<IGraphTreeCreator> mp_GraphTreeCreator;
	std::unique_ptr<IGraphSearcher> mp_GraphSearcher;
};


//! @file GraphSearchHato.h 
//! @brief �g������̎�@�ŃO���t�T�����s���N���X�̎����D
//! @author ���J��
//! @date 2023/07/23

//! @class GraphSearchHato
//! @brief �g����y�̎�@�ŁC�O���t�T�����s���N���X�D
//! @author ���J��
//! @date 2023/07/23