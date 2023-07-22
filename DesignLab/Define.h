#pragma once


class Define final
{
public:

	const static int SIMURATE_NUM;	//!< �A���ŃV�~�����[�V�������s����

	const static char GRAPH_SEARCH_DEPTH;	//!< �O���t�T���̒T���[��

	const static int GATE_PATTERN_GENERATE_NUM;	//!< 1�V�~�����[�V����������̍ő���e������

	const static int GOAL_TAPE;	//!< ���i�̂Ƃ��ɁAY�����ɂ��̒l�����i�߂���1�V�~�����[�V�����I��

	const static bool FLAG_GRAPHIC_AVAILABLE;	//!< �O���t�B�b�N��\������Ƃ���true

	const static bool FLAG_DO_PRUNING;	//!< �O���t�T���ɂ����Ď}������s���Ƃ���true

private:

	Define() = delete;
	Define(const Define& _other) = delete;
};


//! @file Define.h
//! @brief �v���W�F�N�g�S�̂̒萔
//! @author ���J��
//! @date 2023/06/17

//! @class Define
//! @brief
//! @details Effective C++ �Ƃ����{ (���������Ă���̂͂��Ȃ�Â��ł̂��̂Ȃ̂Ő����������̂��͂悭�킩��Ȃ�����) �ɂ��ƁC<br>
//! C++�ɂ����Ă͂��܂� #define���g�p����ׂ��ł͂Ȃ��悤�ł���D���\�����ȃT�C�g�ł��������Ƃ������Ă���D<br> 
//! https://qiita.com/jonichonpa/items/595ed7914751787ebaee <br>
//! https://myon.info/blog/2015/12/18/avoid-defining-macros/ <br>
//! <br>
//! const statis�Ȓ萔�Dinline �֐��Dconstexpr�萔���g���ׂ��D<br>
//! <br>
//! �ȉ��Q�l�����D<br> 
//! �N���X�����o�� constexpr static �ϐ��͂������߂��Ȃ� �\ ���ۂƑ΍�<br>
//! https://qiita.com/Nabetani/items/d8a3ebccaef03cd18d81
//! @attention GRAPH_SEARCH_DEPTH�̒l��傫����������Ɠ��삵�Ȃ��Ȃ�D<br>
//! ��s�����ł͐[��4�`5������œ��삹�Ă����̂ł��̂�����ŁC<br>
//! ���Ȃ݂ɁC�[����1�[�����������ŁC�T������͈͖͂c��ɑ�����̂Œ��ӂ��K�v�D�[��6�ȏ�������ԓ��ɒT�����I���̂́C���������̐��\��PC���K�v�D
//! @author ���J��
//! @date 2023/06/17