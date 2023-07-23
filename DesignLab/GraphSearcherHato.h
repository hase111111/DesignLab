#pragma once
#include "InterfaceGraphSearcher.h"


class GraphSearcherHato final : public IGraphSearcher
{
public:

	EGraphSearchResult searchGraphTree(const std::vector<SNode>& _graph, const STarget& _target, SNode& _output_result) override;

private:

};

//! @file GraphSearcherHato
//! @brief �g������̎�@�ŃO���t�T�����s���N���X�̎����D
//! @date 2023/07/23
//! @auther ���J��

//! @class GraphSearcherHato
//! @brief �g����y�̎�@�ŁC�O���t�T�����s���N���X�D
//! @date 2023/07/23
//! @auther ���J��