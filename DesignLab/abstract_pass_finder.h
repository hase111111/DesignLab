#pragma once

#include <vector>

#include "map_state.h"
#include "node.h"
#include "Target.h"
#include "graph_search_result.h"
#include "application_setting_recorder.h"
#include "interface_graph_tree_creator.h"
#include "abstract_graph_searcher.h"
#include "abstract_pass_finder_factory.h"
#include "abstract_hexapod_state_calculator.h"


//! @class AbstractPassFinder
//! @brief �O���t�T�����s���N���X�̒��ۃN���X�D���͍̂쐬�ł��Ȃ��̂ł�����p�����Ă��N���X���g�����ƁD
//! @date 2023/09/03
//! @author ���J��
//! @details �p��������N���X�̃f�X�g���N�^��virtual�ɂ��Ă����D
//! @n �Q�l https://www.yunabe.jp/docs/cpp_virtual_destructor.html
class AbstractPassFinder
{
public:

	AbstractPassFinder() = default;
	virtual ~AbstractPassFinder() = default;


	void init(std::unique_ptr<AbstractPassFinderFactory>&& factory, std::shared_ptr<AbstractHexapodStateCalculator> calc, const SApplicationSettingRecorder* const setting);


	//! @brief �O���t�T�����s���C���̓���Ƃ��čœK�ȃm�[�h��Ԃ��D
	//! @param [in] current_node ���݂̏�Ԃ�\���m�[�h
	//! @param [in] p_map ���݂̃}�b�v�̏��
	//!	@param [in] target �ڕW�̏��
	//! @param [out] output_node ���ʂ̃m�[�h
	//! @return EGraphSearchResult �O���t�T���̌��ʂ�Ԃ��D
	virtual EGraphSearchResult getNextNodebyGraphSearch(const SNode& current_node, const MapState* const p_map, const STarget& target, SNode& output_node) = 0;


	//! @brief �쐬�����O���t�̐���Ԃ�
	//! @return int �쐬�����O���t�̐�
	int getMadeNodeNum() const { return m_made_node_num; }

	//! @brief �쐬�����O���t�؂�Ԃ��D
	//! @n ���̊֐��̓f�o�b�O�p�Ȃ̂ŁC�T���ɂ͎g��Ȃ����ƁD
	//! @param [out] output_graph �쐬�����O���t�؂�n���D
	void getGraphTree(std::vector<SNode>* output_graph) const
	{
		(*output_graph).clear();

		for (auto& i : m_graph_tree)
		{
			(*output_graph).emplace_back(i);
		}
	}

protected:

	int m_made_node_num = 0;	//!< �쐬�����O���t�̐�

	std::vector<SNode> m_graph_tree;	//!< �O���t��


	std::unique_ptr<AbstractPassFinderFactory> mp_factory;	//!< �p�X�T���N���X�̃t�@�N�g���[

	std::shared_ptr<AbstractHexapodStateCalculator> mp_calculator;	//!< �w�L�T�|�b�h�̏�Ԃ��v�Z����N���X

	const SApplicationSettingRecorder* mp_setting;	//!< �ݒ���L�^����N���X
};


//! @file interface_pass_finder.h
//! @date 2023/08/14
//! @author ���J��
//! @brief �O���t�T�����s���N���X�̃C���^�[�t�F�C�X�̎���
//! @n �s�� : @lineinfo
