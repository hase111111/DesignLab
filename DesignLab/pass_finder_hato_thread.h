#pragma once

#include <memory>

#include "map_state.h"
#include "node.h"
#include "abstract_pass_finder.h"


//! @class PassFinderHatoThread
//! @date 2023/08/14
//! @author ���J��
//! @brief �p�X�T���N���X
class PassFinderHatoThread final : public AbstractPassFinder
{
public:
	PassFinderHatoThread() = default;
	~PassFinderHatoThread() = default;

	EGraphSearchResult getNextNodebyGraphSearch(const SNode& current_node, const MapState* const p_map, const STarget& target, SNode& output_node) override;
};


//! @file pass_finder_hato_thread.h
//! @date 2023/08/21
//! @author ���J��
//! @brief �O���t�T�������ōs���N���X
//! @n �s�� : @lineinfo
