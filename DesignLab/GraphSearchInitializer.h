#pragma once
#include "InterfaceGraphTreeCreator.h"
#include "InterfaceGraphSearcher.h"
#include <memory>

//! @brief GraphSearch�N���X������������N���X�D�C�j�V�����C�U�[�Ƃ�Initialize(������)����l�̂���
//! @details ���̃N���X���g���ƁC�O���t�T��������N���X�ƁC�O���t�؂��쐬����N���X�𓯎��ɏ������ł���D
//! @note �݌v�������������̂ŁC��ŏ�������
class GraphSearchInitializer
{
public:
	GraphSearchInitializer() = default;
	~GraphSearchInitializer() = default;

	//! @brief �T������O���t�؂��쐬����N���X�ƁC�O���t�T��������N���X�����ꂼ�ꏉ��������D
	//! @param _p_tree_creator �O���t�؂��쐬����N���X�̃C���X�^���X�D
	//! @param _p_graph_searcher �O���t�T��������N���X�̃C���X�^���X�D
	//! @return �������ɐ���������true�C���s������false
	bool init(std::unique_ptr<IGraphTreeCreator>& _p_tree_creator, std::unique_ptr<IGraphSearcher>& _p_graph_searcher);

private:

};
