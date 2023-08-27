#pragma once

#include <string>



//! @enum EOutputPriority
//! @date 2023/08/27
//! @author ���J��
//! @brief �R�}���h���C���ɕ������o�͂���ۂ̗D��x�D
enum class EOutputPriority
{
	SYSTEM = 0,		//!< ��ɏo�͂���
	ERROR_MES = 1,		//!< �G���[�o��
	WARNING = 2,	//!< �x���o��
	INFO = 3,		//!< �D��x��߂̏��
	DEBUG = 4,		//!< �f�o�b�O���̂ݏo��
};


namespace std
{

	//! @brief EOutputPriority�𕶎���ɕϊ�����
	//! @param[in] priority EOutputPriority
	//! @return EOutputPriority�𕶎���ɕϊ���������
	std::string to_string(EOutputPriority priority);


	//! @fn EOutputPriority toOutputPriority(const std::string& str)
	//! @brief �������EOutputPriority�ɕϊ�����
	//! @param[in] str ������
	//! @return EOutputPriority 
	EOutputPriority toOutputPriority(const std::string& str);

} // namespace std




//! @file output_priority.h
//! @date 2023/08/27
//! @author ���J��
//! @brief �R�}���h���C���ɕ������o�͂���ۂ̗D��x�D
//! @n �s�� : @lineinfo
