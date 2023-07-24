#pragma once
#include "MapState.h"
#include "Node.h"
#include "InterfacePassFinder.h"
#include <memory>

class PassFinderHato final : public IPassFinder
{
public:
	PassFinderHato(std::unique_ptr<AbstractPassFinderFactory>&& _factory) : IPassFinder(std::move(_factory)) {};
	~PassFinderHato() = default;

	EGraphSearchResult getNextNodebyGraphSearch(const SNode& _current_node, const MapState* const _p_map, const STarget& _target, SNode& _output_node) override;

private:

};


//! @file PassFinderHato.h 
//! @brief �g������̎�@�ŃO���t�T�����s���N���X�̎����D
//! @author ���J��
//! @date 2023/07/24

//! @class PassFinderHato
//! @brief �g����y�̎�@�ŁC�O���t�T�����s���N���X�D
//! @author ���J��
//! @date 2023/07/24