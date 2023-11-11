//! @file graph_searcher_random.h
//! @brief �����_���ɃO���t�T�����s���N���X�D


#ifndef DESIGNLAB_GRAPH_SEARCHER_RANDOM_H_
#define DESIGNLAB_GRAPH_SEARCHER_RANDOM_H_


#include "interface_graph_searcher.h"


//! @class GraphSearcherRandom
//! @brief ���S�����_���Ɏ��̓����I��ŕԂ��܂��D
//! @n �p���̕��@�̐����p�D
class GraphSearcherRandom final : public IGraphSearcher
{
public:

	// �p�����ɂ́C�߂�l�C�֐����C�����̌^(���O�͈���Ă��悢)�C��S�ē����ɂ���K�v������D
	// �܂��C�Ō�ɕK��override������D

	GraphSearchResult SearchGraphTree(const std::vector<RobotStateNode>& graph, const TargetRobotState& target, RobotStateNode* output_result) override;
};


#endif // !DESIGNLAB_GRAPH_SEARCHER_RANDOM_H_