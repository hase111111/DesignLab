//! @file interface_pass_finder.h
//! @brief �O���t�T���ɂ����e�p�^�[���������s���N���X�̃C���^�[�t�F�[�X�D

#ifndef DESIGNLAB_INTERFACE_PASS_FINDER_H_
#define DESIGNLAB_INTERFACE_PASS_FINDER_H_

#include <vector>

#include "abstract_hexapod_state_calculator.h"
#include "graph_search_result.h"
#include "map_state.h"
#include "node.h"
#include "target.h"


//! @class IPassFinder
//! @brief �O���t�T���ɂ����e�p�^�[���������s���N���X�̃C���^�[�t�F�[�X�D
//! @n �g������̃v���O�����Ō����Ƃ����PassFinding�N���X�D
//! @details ���͍̂쐬�ł��Ȃ��̂ł�����p�����Ă��N���X���g�����ƁD
//! @n �p��������N���X�̃f�X�g���N�^��virtual�ɂ��Ă����D
//! @n �Q�l https://www.yunabe.jp/docs/cpp_virtual_destructor.html
class IPassFinder
{
public:

	IPassFinder() = default;
	virtual ~IPassFinder() = default;


	//! @brief �O���t�T�����s���C���̓���Ƃ��čœK�ȃm�[�h��Ԃ��D
	//! @param [in] current_node ���݂̏�Ԃ�\���m�[�h
	//! @param [in] map ���݂̃}�b�v�̏��
	//!	@param [in] target �ڕW�̏��
	//! @param [out] output_node ���ʂ̃m�[�h
	//! @return EGraphSearchResult �O���t�T���̌��ʂ�Ԃ��D���������s��
	virtual EGraphSearchResult GetNextNodebyGraphSearch(const SNode& current_node, const MapState& map, const STarget& target, SNode* output_node) = 0;

	//! @brief �쐬�����O���t�̃m�[�h�̐���Ԃ�
	//! @n ���̊֐��̓f�o�b�O�p�Ȃ̂ŁC�T���ɂ͎g��Ȃ����ƁD
	//! @return int �쐬�����O���t�̐�
	virtual int GetMadeNodeNum() const = 0;

	//! @brief �쐬�����O���t�؂�Ԃ��D
	//! @n ���̊֐��̓f�o�b�O�p�Ȃ̂ŁC�T���ɂ͎g��Ȃ����ƁD
	//! @param [out] output_graph �쐬�����O���t�؂�n���D
	virtual void GetGraphTree(std::vector<SNode>* output_graph) const = 0;
};


#endif	// DESIGNLAB_INTERFACE_PASS_FINDER_H_