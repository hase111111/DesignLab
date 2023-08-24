#pragma once

#include <vector>
#include <fstream>

#include "node.h"
#include "graph_search_result.h"
#include "simulation_result.h"



//! @struct SSimulationRecord
//! @date 2023/08/24
//! @author ���J��
//! @brief �V�~�����[�V�����̌��ʂ��i�[����\���́D�ϐ��������Ⴒ���Ⴓ�������Ȃ��̂ō쐬
//! @n �ŏ���S��Struct��S
struct SSimulationRecord final
{
	std::vector<SNode> m_node;	//!< ����̋L�^
	std::vector<double> m_time;	//!< �O���t�T���ɂ�����������
	std::vector<EGraphSearchResult> m_graph_search_result;	//!< �O���t�T���̌���
	ESimulationResult m_simulation_result;	//!< �V�~�����[�V�����S�̂̌���

	//int m_clear_num;							//��苗�����s�ł��ăV�~�����[�V�������I��������
	//int m_failed_by_gate_pattern_loop;			//����������J��Ԃ��ăV�~�����[�V�������I��������
	//int m_failed_by_no_gate_pattern;			//���e�p�^�[��������ꂸ�ɃV�~�����[�V�������I��������
	//int m_gate_pattern_generate_sum;			//�S�V�~�����[�V�����ŏo�͂��ꂽ���e�p�^�[���̑���

	//int m_distance_move_Y_sum;					//�S�V�~�����[�V�����Ői�񂾋���
	//int m_distance_move_Y_max;
	//int m_distance_move_Y_min;

	//double m_gate_parttern_generate_time_sum;	//�S�V�~�����[�V�����ŕ��e�p�^�[�������ɂ����������Ԃ̑��a[s]
	//double m_gate_parttern_generate_time_max;
	//double m_gate_parttern_generate_time_min;
};


std::ofstream& operator<<(std::ofstream& ofs, const SSimulationRecord& record);

//! @file simulation_record.h
//! @date 2023/08/24
//! @author ���J��
//! @brief �V�~�����[�V�����̌��ʂ��L�^����N���X�D
//! @n �s�� : @lineinfo
