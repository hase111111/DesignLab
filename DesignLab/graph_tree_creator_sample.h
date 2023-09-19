#pragma once
#include "interface_graph_tree_creator.h"


class GraphTreeCreatorSample final : public IGraphTreeCreator
{
public:
	GraphTreeCreatorSample(std::map<EHexapodMove, std::unique_ptr<INodeCreator>>& map) : IGraphTreeCreator(map) {};
	~GraphTreeCreatorSample() = default;

	EGraphSearchResult CreateGraphTree(const SNode& current_node, const DevideMapState& map_ref, std::vector<SNode>* output_graph) override;

};


//! @file graph_tree_creator_sample.h
//! @brief �O���t�؂��쐬����N���X�̃T���v��
//! @author ���J��

//! @class GraphTreeCreatorSample
//! @brief �O���t���쐬����N���X�̃T���v���ł��D�e�L�g�[�ɃO���t���쐬���܂��D
//! @details IGraphTreeCreator ���p���������ꍇ�D�ȉ��̂悤��<br> <br>
//!	class �D���Ȗ��O final : public IGraphTreeCreator <br>
//!	{ <br>
//!	} <br> <br>
//! �Ɛ錾����D<br>final�͂�������ȏ�p�����Ȃ���Ƃ����Ӗ��Dpublic IGraphTreeCreator �͂��̃N���X���p��������Ƃ����Ӗ��D<br>
//! IGraphTreeCreator���p�������N���X�ɉۂ����鐧��͂�����CIGraphTreeCreator �̏������z�֐��CcreateGraphTree���������邱�ƁD<br>
//! �ȉ��̂悤��createGraphTree��public�ȂƂ���ɐ錾���āC����override�Ƃ���悤�ɁD<br>
