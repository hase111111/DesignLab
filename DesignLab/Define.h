//! @file define.h
//! @brief �v���W�F�N�g�S�̂Ŏg�p����萔���܂Ƃ߂�����

#ifndef DESIGNLAB_DEFINE_H_
#define DESIGNLAB_DEFINE_H_

//#define DESIGNLAB_DONOT_USE_DXLIB	//!< DX���C�u�������g�p���Ȃ��ꍇ�̓R�����g�A�E�g���O��

#include <string>


//! @class Define
//! @brief �v���W�F�N�g�S�̂Ŏg�p����萔���܂Ƃ߂��萔�N���X
//! @details Effective C++ �Ƃ����{ (���������Ă���̂͂��Ȃ�Â��ł̂��̂Ȃ̂Ő����������̂��͂悭�킩��Ȃ�����) �ɂ��ƁC
//! @n C++�ɂ����Ă͂��܂�萔��錾���邽�߂� #define���g�p����ׂ��ł͂Ȃ��悤�ł���D���\�����ȃT�C�g�ł��������Ƃ������Ă���D
//! @n https://qiita.com/jonichonpa/items/595ed7914751787ebaee
//! @n https://myon.info/blog/2015/12/18/avoid-defining-macros/
//! @n
//! @n const statis�Ȓ萔�Dinline �֐��Dconstexpr�萔���g���ׂ��D
//! @n
//! @n �ȉ��Q�l�����D 
//! @n �N���X�����o�� constexpr static �ϐ��͂������߂��Ȃ� �\ ���ۂƑ΍�
//! @n https://qiita.com/Nabetani/items/d8a3ebccaef03cd18d81
//! @attention GRAPH_SEARCH_DEPTH�̒l��傫����������Ɠ��삵�Ȃ��Ȃ�D
//! @n ��s�����ł͐[��4�`5������œ��삹�Ă����̂ł��̂�����ŁC
//! @n ���Ȃ݂ɁC�[����1�[�����������ŁC�T������͈͖͂c��ɑ�����̂Œ��ӂ��K�v�D�[��6�ȏ�������ԓ��ɒT�����I���̂́C���������̐��\��PC���K�v�D
class Define final
{
public:

	// �R���X�g���N�^��S�č폜���āC�C���X�^���X���ł��Ȃ��悤�ɂ���
	Define() = delete;
	Define(const Define& _other) = delete;
	Define(const Define&& _other) = delete;
	Define& operator = (const Define& _other) = delete;


	const static int kSimurateNum;	//!< �A���ŃV�~�����[�V�������s����

	const static int kGaitPatternGenerationLimit;	//!< 1�V�~�����[�V����������̍ő���e������

	const static int kGoalTape;	//!< ���i�̂Ƃ��ɁAY�����ɂ��̒l�����i�߂���1�V�~�����[�V�����I��

	const static std::string kResultFilePath;	//!< �V�~�����[�V�������ʂ�ۑ�����t�@�C���̃p�X
};


#endif // !DESIGNLAB_DEFINE_H_