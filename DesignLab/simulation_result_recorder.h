//! @file simulation_result_recorder.h
//! @brief �V�~�����[�V�����̌��ʂ��L�^����N���X�D


#ifndef DESIGNLAB_SIMULATION_RESULT_RECORDER_H_
#define DESIGNLAB_SIMULATION_RESULT_RECORDER_H_


#include <string>
#include <vector>

#include "graph_search_result_recoder.h"
#include "map_state.h"


//! @enum SimulationResult
//! @brief �V�~�����[�V�����S�̂̌��ʂ�\���񋓌^
enum class SimulationResult
{
	kSuccess,						//!< �ڕW���W�C�p���𖞂����C�V�~�����[�V�����ɐ��������D
	kFailureByGraphSearch,			//!< �O���t�T���Ɏ��s�����߁C�V�~�����[�V�����Ɏ��s�����D
	kFailureByLoopMotion,			//!< ���삪���[�v���Ă��܂������߁C�V�~�����[�V�����Ɏ��s�����D
	kFailureByNodeLimitExceeded,	//!< �m�[�h���̏���ɒB�������߁C�V�~�����[�V�����Ɏ��s�����D
};


//! @struct SimulationResultRecorder
//! @brief �V�~�����[�V�����̌��ʂ��i�[����\���́D�ϐ��������Ⴒ���Ⴓ�������Ȃ��̂ō쐬
struct SimulationResultRecorder final
{
	//! @brief ���̃N���X�̃f�[�^��csv�t�@�C���ɏo�͂���p�̌`���ŕ�����ɕϊ�����
	//! @return csv�t�@�C���ɏo�͂���p�̌`���̕�����
	std::string ToCsvString() const;


	//!< �O���t�T���̌��ʂ��i�[����\���̂̔z��
	std::vector<GraphSearchResultRecoder> graph_search_result_recoder;	

	MapState map_state;					//!< �ŐV�̒n�ʂ̏��

	SimulationResult simulation_result;	//!< �V�~�����[�V�����S�̂̌���
};


#endif	// !DESIGNLAB_SIMULATION_RESULT_RECORDER_H_