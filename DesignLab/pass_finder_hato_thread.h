#pragma once

#include <memory>

#include "map_state.h"
#include "node.h"
#include "interface_pass_finder.h"


//! @class PassFinderHatoThread
//! @date 2023/08/14
//! @author ���J��
//! @brief �p�X�T���N���X
class PassFinderHatoThread final : public IPassFinder
{
public:
	PassFinderHatoThread(std::unique_ptr<AbstractPassFinderFactory>&& factory);
	~PassFinderHatoThread() = default;

	EGraphSearchResult getNextNodebyGraphSearch(const SNode& current_node, const MapState* const p_map, const STarget& target, SNode& output_node) override;
};
