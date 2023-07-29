#pragma once
#include "InterfacePassFinder.h"


class PassFinderHato final : public IPassFinder
{
public:
	PassFinderHato(std::unique_ptr<AbstractPassFinderFactory>&& factory);
	~PassFinderHato();

	EGraphSearchResult getNextNodebyGraphSearch(const SNode& current_node, const MapState* const p_map, const STarget& target, SNode& output_node) override;

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