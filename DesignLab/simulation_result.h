#pragma once

#include <string>


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
	std::string to_string(ESimulationResult result)
	{
		switch (result)
		{
		case ESimulationResult::SUCCESS:
			return "SUCCESS";
		case ESimulationResult::FAILURE:
			return "FAILURE";
		case ESimulationResult::FAILURE_BY_GRAPH_SEARCH:
			return "FAILURE_BY_GRAPH_SEARCH";
		case ESimulationResult::FAILURE_BY_LOOP_MOTION:
			return "FAILURE_BY_LOOP_MOTION";
		case ESimulationResult::FAILURE_BY_NODE_LIMIT_EXCEEDED:
			return "FAILURE_BY_NODE_LIMIT_EXCEEDED";
		default:
			return "UNKNOWN";
		}
	}

}