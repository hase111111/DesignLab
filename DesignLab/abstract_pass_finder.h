//! @file abstract_pass_finder.h
//! @brief �p�X�T�����s���N���X�̒��ۃN���X�D

#ifndef DESIGNLAB_ABSTRACT_PASS_FINDER_H_
#define DESIGNLAB_ABSTRACT_PASS_FINDER_H_

#include <vector>

#include "abstract_graph_searcher.h"
#include "abstract_hexapod_state_calculator.h"
#include "application_setting_recorder.h"
#include "graph_search_result.h"
#include "interface_graph_tree_creator.h"
#include "interface_pass_finder_factory.h"
#include "map_state.h"
#include "node.h"
#include "target.h"


//! @class AbstractPassFinder
//! @brief �O���t�T�����s���N���X�̒��ۃN���X�D���͍̂쐬�ł��Ȃ��̂ł�����p�����Ă��N���X���g�����ƁD
//! @details �p��������N���X�̃f�X�g���N�^��virtual�ɂ��Ă����D
//! @n �Q�l https://www.yunabe.jp/docs/cpp_virtual_destructor.html

class AbstractPassFinder
{
public:

	AbstractPassFinder() = default;
	virtual ~AbstractPassFinder() = default;


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

	int m_made_node_num = 0;			//!< �쐬�����O���t�̐�

	std::vector<SNode> m_graph_tree;	//!< �O���t��


	//! @brief �O���t�؂̐����ɕK�v�ȃN���X�𐶐�����D
	//! @param [in] map �}�b�v���D
	//! @param [in] calculator_ptr_ �w�L�T�|�b�h�̏�Ԃ��v�Z����N���X�D
	//! @return std::unique_ptr<IGraphTreeCreator> �O���t�؂̐����ɕK�v�ȃN���X�D
	virtual std::unique_ptr<IGraphTreeCreator> createGraphTreeCreator(const MapState* const map, const std::shared_ptr<const AbstractHexapodStateCalculator>& calculator_ptr_) = 0;

	//! @brief �O���t�T�����s���N���X�𐶐�����D
	//! @param [in] calculator_ptr_ �w�L�T�|�b�h�̏�Ԃ��v�Z����N���X�D
	//! @return std::unique_ptr<AbstractGraphSearcher> �O���t�T�����s���N���X�D
	virtual std::unique_ptr<AbstractGraphSearcher> createGraphSearcher(const std::shared_ptr<const AbstractHexapodStateCalculator>& calculator_ptr_) = 0;


	//std::unique_ptr<IPassFinderFactory> mp_factory;	//!< �p�X�T���N���X�̃t�@�N�g���[

	//std::shared_ptr<AbstractHexapodStateCalculator> mp_calculator;	//!< �w�L�T�|�b�h�̏�Ԃ��v�Z����N���X

	//const SApplicationSettingRecorder* mp_setting;	//!< �ݒ���L�^����N���X
};


#endif	// DESIGNLAB_ABSTRACT_PASS_FINDER_H_