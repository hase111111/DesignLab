#pragma once
#include "InterfaceGraphSearcher.h"


// ���S�����_���Ɏ��̓����I��ŕԂ��܂��D
class GraphSearcherRandom final : public IGraphSearcher
{
public:

	EGraphSearchResult searchGraphTree(const std::vector<SNode>& graph, const STarget& target, SNode* output_result) override;

};
