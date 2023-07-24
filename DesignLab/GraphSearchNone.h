#pragma once
#include "MapState.h"
#include "Node.h"
#include "InterfaceGraphSearch.h"
#include <memory>

class GraphSearchNone final :public IGraphSearch
{
public:
	GraphSearchNone(std::unique_ptr<AbstractPassFinderFactory>&& _factory) : IGraphSearch(std::move(_factory)) {};
	~GraphSearchNone() = default;

	EGraphSearchResult getNextNodebyGraphSearch(const SNode& _current_node, const MapState* const _p_map, const STarget& _target, SNode& _output_node) override;

private:

};


//! @file GraphSearchNone.h 
//! @brief �O���t�T�����s��Ȃ��N���X�̎����D
//! @author ���J��

//! @class GraphSearchNone
//! @brief �O���t�T�����s��Ȃ��N���X�DGraphic�N���X�̃f�o�b�O���s���ۂɗp����D
//! @author ���J��