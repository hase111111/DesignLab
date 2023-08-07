#pragma once
#include "map_state.h"
#include "Node.h"
#include "InterfacePassFinder.h"
#include <memory>

class PassFinderNone final :public IPassFinder
{
public:
	PassFinderNone(std::unique_ptr<AbstractPassFinderFactory>&& _factory) : IPassFinder(std::move(_factory)) {};
	~PassFinderNone() = default;

	EGraphSearchResult getNextNodebyGraphSearch(const SNode& current_node, const MapState* const p_map, const STarget& target, SNode& output_node) override;
};


//! @file PassFinderNone.h 
//! @brief �O���t�T�����s��Ȃ��N���X�̎����D
//! @author ���J��
//! @date 2023/07/24

//! @class PassFinderNone
//! @brief �O���t�T�����s��Ȃ��N���X�DGraphic�N���X�̃f�o�b�O���s���ۂɗp����D
//! @author ���J��
//! @date 2023/07/24
