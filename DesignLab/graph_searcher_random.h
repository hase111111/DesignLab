#pragma once

#include "interface_graph_searcher.h"


// ���S�����_���Ɏ��̓����I��ŕԂ��܂��D
class GraphSearcherRandom final : public IGraphSearcher
{
public:

	GraphSearchResult SearchGraphTree(const std::vector<RobotStateNode>& graph, const STarget& target, RobotStateNode* output_result) override;
};
