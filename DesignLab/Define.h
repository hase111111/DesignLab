#pragma once

// Effective C++ �Ƃ����{ (���������Ă���̂͂��Ȃ�Â��ł̂��̂Ȃ̂Ő����������̂��͂悭�킩��Ȃ�����) �ɂ��ƁC
// C++�ɂ����Ă͂��܂� #define���g�p����ׂ��ł͂Ȃ��悤�ł��D
// ���\�����ȃT�C�g�ł��������Ƃ������Ă��܂�
// https://qiita.com/jonichonpa/items/595ed7914751787ebaee
// https://myon.info/blog/2015/12/18/avoid-defining-macros/
// const statis�Ȓ萔�Dinline �֐��Dconstexpr�萔���g���ׂ��ł��D
// �ȉ��Q�l�����D
//
//	�N���X�����o�� constexpr static �ϐ��͂������߂��Ȃ� �\ ���ۂƑ΍�
//	https://qiita.com/Nabetani/items/d8a3ebccaef03cd18d81

class Define 
{
	Define() = delete;
	~Define() = delete;

public:
//main.cpp

	//�A���ŃV�~�����[�V�������s����
	const static int SIMURATE_NUM;

	//1�V�~�����[�V����������̍ő���e������
	const static int GATE_PATTERN_GENERATE_NUM;

	//���i�̂Ƃ��ɁAY�����ɂ��̒l�����i�߂���1�V�~�����[�V�����I��	//���v1000����O����z���ƃG���[���o��H���ł�A�����̏ڍׂ͕�����Ȃ����Afstream���֌W���Ă�H20201117hato
	const static int GOAL_TAPE;

	//�O���t�B�b�N��\������Ƃ���true
	const static bool FLAG_GRAPHIC_AVAILABLE;

	//����ȏ㏬�����l��0�Ƃ݂Ȃ��Dallowable error�C���e�덷�̂���
	inline constexpr static double ALLOWABLE_ERROR = 0.0001;

	// �~����
	const static double MY_PI;

//PassFinding.h

	//�O���t�T���ɂ����Ď}������s���Ƃ���true
	const static bool FLAG_DO_PRUNING;
};
