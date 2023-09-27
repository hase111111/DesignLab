//! @file simulation_result_recorder.h
//! @brief �V�~�����[�V�����̌��ʂ��L�^����N���X�D


#ifndef DESIGNLAB_SIMULATION_RESULT_RECORDER_H_
#define DESIGNLAB_SIMULATION_RESULT_RECORDER_H_


#include <fstream>
#include <string>
#include <vector>

#include "node.h"
#include "graph_search_result.h"



//! @enum SimulationResult
//! @brief �V�~�����[�V�����S�̂̌��ʂ�\���񋓌^
enum class SimulationResult
{
	SUCCESS,						//!< �ڕW���W�C�p���𖞂����C�V�~�����[�V�����ɐ��������D
	FAILURE_BY_GRAPH_SEARCH,		//!< �O���t�T���Ɏ��s�����߁C�V�~�����[�V�����Ɏ��s�����D
	FAILURE_BY_LOOP_MOTION,			//!< ���삪���[�v���Ă��܂������߁C�V�~�����[�V�����Ɏ��s�����D
	FAILURE_BY_NODE_LIMIT_EXCEEDED,	//!< �m�[�h���̏���ɒB�������߁C�V�~�����[�V�����Ɏ��s�����D
};


//! @struct SimulationResultRecorder
//! @brief �V�~�����[�V�����̌��ʂ��i�[����\���́D�ϐ��������Ⴒ���Ⴓ�������Ȃ��̂ō쐬
//! @n �ŏ���S��Struct��S
struct SimulationResultRecorder final
{
	std::vector<SNode> result_nodes;						//!< ����̋L�^
	std::vector<double> computation_time;					//!< �O���t�T���ɂ�����������
	std::vector<GraphSearchResult> graph_search_results;	//!< �O���t�T���̌���
	SimulationResult simulation_result;					//!< �V�~�����[�V�����S�̂̌���
};


std::ofstream& operator<<(std::ofstream& ofs, const SimulationResultRecorder& record);



#endif	// !DESIGNLAB_SIMULATION_RESULT_RECORDER_H_