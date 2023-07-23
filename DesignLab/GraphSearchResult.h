#pragma once
#include <string>

//! @enum EGraphSearchReslut
//! @brief �O���t�T���̌��ʂ�\���񋓌^
enum class EGraphSearchResult : int
{
	Success,						//!< �O���t�T���ɐ�������
	Failure,						//!< �O���t�T���Ɏ��s����
	FailureByInitializationFailed,	//!< �O���t�T�����s���N���X�̏������Ɏ��s����
	FailureByNodeLimitExceeded,		//!< �m�[�h���̏���ɒB�������߃O���t�T���Ɏ��s����
	FailureByNoNode,				//!< �O���t�؂��쐬�������C�m�[�h��1�������ł��Ȃ������D
	FailureByNotReachedDepth,		//!< �O���t�؂��쐬�������C�ڕW�[���ɓ��B�ł��Ȃ������D
	FailureByLegPathGenerationError,//!< �r�̋O�������Ɏ��s����

	SuccessByReevaluation,			//!< �ĕ]���ɂ��O���t�T���ɐ�������	
	FailureByReevaluation,			//!< �ĕ]�����s�������C�O���t�T���Ɏ��s����
	FailureByReevaluationAndNodeLimitExceeded,		//!< �ĕ]�����s�������C�m�[�h���̏���ɒB�������߃O���t�T���Ɏ��s����
	FailureByReevaluationAndNoNode,					//!< �ĕ]�����s�������C�m�[�h��1�������ł��Ȃ������D
	FailureByReevaluationAndNotReachedDepth,		//!< �ĕ]�����s�������C�ڕW�[���ɓ��B�ł��Ȃ������D
	FailureByReevaluationAndLegPathGenerationError,	//!< �ĕ]�����s�������C�r�̋O�������Ɏ��s����
};

namespace std
{
	//! @brief EGraphSearchResult�^�𕶎���ɕϊ�����֐�
	//! @param [in] _result EGraphSearchResult�^�̕ϐ�
	//! @return std::string EGraphSearchResult�^�̕ϐ��𕶎���ɂ�������
	std::string to_string(EGraphSearchResult _result);
}

//! @brief �O���t�T���������������ǂ�����Ԃ��֐�
//! @param [in] _result �O���t�T���̌���
//! @return bool ������true�C���s��false
inline bool graphSeachResultIsSuccessful(EGraphSearchResult _result)
{
	return _result == EGraphSearchResult::Success || _result == EGraphSearchResult::SuccessByReevaluation;
}