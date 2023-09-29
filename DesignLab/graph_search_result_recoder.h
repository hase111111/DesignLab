//! @file graph_search_result_recoder.h
//! @brief �O���t�T���̌��ʂ��i�[����\���́D


#ifndef DESIGNLAB_GRAPH_SEARCH_RESULT_RECODER_H_
#define DESIGNLAB_GRAPH_SEARCH_RESULT_RECODER_H_


#include "robot_state_node.h"


//! @enum GraphSearchReslut
//! @brief �O���t�T���̌��ʂ�\���񋓌^
enum class GraphSearchResult
{
	kSuccess,							//!< �O���t�T���ɐ�������
	kFailure,							//!< �O���t�T���Ɏ��s����
	kFailureByInitializationFailed,		//!< �O���t�T�����s���N���X�̏������Ɏ��s����
	kFailureByNodeLimitExceeded,		//!< �m�[�h���̏���ɒB�������߃O���t�T���Ɏ��s����
	kFailureByNoNode,					//!< �O���t�؂��쐬�������C�m�[�h��1�������ł��Ȃ������D
	kFailureByNotReachedDepth,			//!< �O���t�؂��쐬�������C�ڕW�[���ɓ��B�ł��Ȃ������D
	kFailureByLegPathGenerationError,	//!< �r�̋O�������Ɏ��s����
};


//! @struct GraphSearchResultRecoder
//! @brief �O���t�T���̌��ʂ��i�[����\���́D�ϐ��������Ⴒ���Ⴓ�������Ȃ��̂ō쐬
struct GraphSearchResultRecoder final
{
	GraphSearchResultRecoder() : 
		result_node{}, 
		computation_time(0.0), 
		graph_search_result(GraphSearchResult::kFailure),
		did_reevaluation(false)
	{
	};

	GraphSearchResultRecoder(const RobotStateNode& node,const double time, const GraphSearchResult result) : 
		result_node(node), 
		computation_time(time), 
		graph_search_result(result),
		did_reevaluation(false)
	{
	};


	RobotStateNode result_node;		//!< �O���t�T���ɂ���đI�����ꂽ����̋L�^

	double computation_time;		//!< �O���t�T���ɂ�����������

	GraphSearchResult graph_search_result;	//!< �O���t�T���̌��ʁC���������s��

	bool did_reevaluation;			//!< �ĕ]�����s�������ǂ���

};



#endif // !DESIGNLAB_GRAPH_SEARCH_RESULT_RECODER_H_