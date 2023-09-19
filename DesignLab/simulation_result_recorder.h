#pragma once

#include <string>
#include <vector>
#include <fstream>

#include "node.h"
#include "graph_search_result.h"



//! @enum ESimulationResult
//! @date 2023/08/24
//! @author ���J��
//! @brief �V�~�����[�V�����̌��ʂ�\���񋓌^
enum class ESimulationResult
{
	SUCCESS,						//!< �V�~�����[�V�����ɐ�������
	FAILURE,						//!< �V�~�����[�V�����Ɏ��s����
	FAILURE_BY_GRAPH_SEARCH,		//!< �O���t�T���Ɏ��s�����߁C�V�~�����[�V�����Ɏ��s����
	FAILURE_BY_LOOP_MOTION,			//!< ���삪���[�v���Ă��܂������߁C�V�~�����[�V�����Ɏ��s����
	FAILURE_BY_NODE_LIMIT_EXCEEDED,	//!< �m�[�h���̏���ɒB�������߁C�V�~�����[�V�����Ɏ��s����
};



namespace std
{
	//! @brief ESimulationResult�^�𕶎���ɕϊ�����֐�
	//! @param [in] result ESimulationResult�^�̕ϐ�
	//! @return std::string ESimulationResult�^�̕ϐ��𕶎���ɂ�������
	std::string to_string(ESimulationResult result);

} // namespace std



//! @struct SSimulationResultRecorder
//! @date 2023/08/24
//! @author ���J��
//! @brief �V�~�����[�V�����̌��ʂ��i�[����\���́D�ϐ��������Ⴒ���Ⴓ�������Ȃ��̂ō쐬
//! @n �ŏ���S��Struct��S
struct SSimulationResultRecorder final
{
	std::vector<SNode> result_nodes;						//!< ����̋L�^
	std::vector<double> computation_time;					//!< �O���t�T���ɂ�����������
	std::vector<EGraphSearchResult> graph_search_results;	//!< �O���t�T���̌���
	ESimulationResult simulation_result;					//!< �V�~�����[�V�����S�̂̌���
};


std::ofstream& operator<<(std::ofstream& ofs, const SSimulationResultRecorder& record);



//! @file simulation_result_recorder.h
//! @date 2023/08/24
//! @author ���J��
//! @brief �V�~�����[�V�����̌��ʂ��L�^����N���X�D
//! @n �s�� : @lineinfo
