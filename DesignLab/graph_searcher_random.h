#pragma once

#include "abstract_graph_searcher.h"


// ���S�����_���Ɏ��̓����I��ŕԂ��܂��D
class GraphSearcherRandom final : public AbstractGraphSearcher
{
public:

	GraphSearcherRandom(const std::shared_ptr<const AbstractHexapodStateCalculator>& calc) : AbstractGraphSearcher(calc) {}

	EGraphSearchResult searchGraphTree(const std::vector<SNode>& graph, const STarget& target, SNode* output_result) override;
};
