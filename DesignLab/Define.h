#pragma once

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

//mapData.h
	enum { MAPDATA3D_MAX = 15000 };		//enum�n�b�N�D�׈��ȉ����@

//PassFinding.h

	//�O���t�T���ɂ����Ď}������s���Ƃ���true
	const static bool FLAG_DO_PRUNING;

//Hexapod.h
	inline constexpr static int LEG_NUM = 6;

};
