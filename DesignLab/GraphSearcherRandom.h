#pragma once
#include "InterfaceGraphSearcher.h"


// ���S�����_���Ɏ��̓����I��ŕԂ��܂��D
class GraphSearcherRandom final : public IGraphSearcher
{
public:

	EGraphSearchResult searchGraphTree(const std::vector<SNode>& _graph, const STarget& _target, SNode& _output_result) override;

private:

};
