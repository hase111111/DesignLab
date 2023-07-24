#pragma once
#include "InterfaceGraphTreeCreator.h"


class GraphTreeCreatorSample final : public IGraphTreeCreator
{
public:
	GraphTreeCreatorSample(std::map<EHexapodMove, std::unique_ptr<INodeCreator>>& _map) : IGraphTreeCreator(_map) {};
	~GraphTreeCreatorSample() = default;

	EGraphSearchResult createGraphTree(const SNode& _current_node, const MapState* const _p_map, std::vector<SNode>& _output_graph, int& _make_node_num) override;

};


//! @file GraphTreeCreatorSample.h
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
