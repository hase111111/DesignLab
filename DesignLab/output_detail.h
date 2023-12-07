//! @file output_detail.h
//! @brief �R�}���h���C���ɕ������o�͂���ۂ̏ڍׂ��܂Ƃ߂��񋓑́D

#ifndef DESIGNLAB_OUTPUT_DETAIL_H_
#define DESIGNLAB_OUTPUT_DETAIL_H_


//! @enum OutputDetail
//! @brief �R�}���h���C���ɕ������o�͂���ۂ̏ڍׁD
//! @details Setting�t�@�C���Ŏw�肳�ꂽ�D��x�ȏ�̃��b�Z�[�W�̂ݏo�͂���D
enum class OutputDetail
{
	kSystem = 0,	//!< �V�X�e�����b�Z�[�W�C��ɏo�͂���D
	kError,			//!< �G���[���b�Z�[�W�D
	kWarning,		//!< �x�����b�Z�[�W�C�G���[�ł͂Ȃ������ӂ��K�v�ȃ��b�Z�[�W�D
	kInfo,			//!< �D��x��߂̏��D
	kDebug,			//!< �f�o�b�O���̂ݏo�́C��ԗD��x���Ⴂ�D
};


#endif	// DESIGNLAB_OUTPUT_DETAIL_H_