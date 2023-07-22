#pragma once
#include "IGraphTreeCreator.h"
#include "IGraphSearcher.h"
#include <memory>

//GraphSearch�N���X������������N���X�D�C�j�V�����C�U�[�Ƃ�Initialize(������)����l�̂���
class GraphSearchInitializer
{
public:
	GraphSearchInitializer() = default;
	~GraphSearchInitializer() = default;

	//�T������O���t�؂��쐬����N���X�ƁC�O���t�T��������N���X�����ꂼ�ꏉ��������D
	bool init(std::unique_ptr<IGraphTreeCreator>& _p_tree_creator, std::unique_ptr<IGraphSearcher>& _p_graph_searcher);

private:

};
