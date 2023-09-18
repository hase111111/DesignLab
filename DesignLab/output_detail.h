//! @file output_detail.h
//! @brief �R�}���h���C���ɕ������o�͂���ۂ̏ڍׁD

#ifndef DESIGNLAB_OUTPUT_DETAIL_H_
#define DESIGNLAB_OUTPUT_DETAIL_H_

#include <string>


//! @enum OutputDetail
//! @brief �R�}���h���C���ɕ������o�͂���ۂ̏ڍׁD
//! @details Setting�t�@�C���Ŏw�肳�ꂽ�D��x�ȏ�̃��b�Z�[�W�̂ݏo�͂���D

enum class OutputDetail
{
	kSystem = 0,	//!< �V�X�e�����b�Z�[�W�C��ɏo�͂���
	kError,			//!< �G���[���b�Z�[�W
	kWarning,		//!< �x�����b�Z�[�W�C�G���[�ł͂Ȃ������ӂ��K�v�ȃ��b�Z�[�W
	kInfo,			//!< �D��x��߂̏��
	kDebug,			//!< �f�o�b�O���̂ݏo�́C��ԗD��x���Ⴂ
};


namespace std
{

	//! @brief EOutputPriority�𕶎���ɕϊ�����
	//! @param[in] detail OutputDetail
	//! @return EOutputPriority�𕶎���ɕϊ���������
	std::string to_string(OutputDetail detail);


	//! @fn OutputDetail toOutputPriority(const std::string& str)
	//! @brief �������EOutputPriority�ɕϊ�����
	//! @param[in] str ������
	//! @return OutputDetail 
	OutputDetail toOutputPriority(const std::string& str);

} // namespace std


#endif	// DESIGNLAB_OUTPUT_DETAIL_H_