#pragma once
#include "IGraphSearcher.h"


// ���S�����_���Ɏ��̓����I��ŕԂ��܂��D
class GraphSearcherRandom final : public IGraphSearcher
{
public:

	bool searchGraphTree(const std::vector<SNode>& _graph, const STarget& _target, SNode& _output_result) override;

private:

};
