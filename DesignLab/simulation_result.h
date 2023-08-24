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
	std::string to_string(ESimulationResult result);

} // namespace std