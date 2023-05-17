#pragma once
#include <string>

class MapConst final
{
public:
	//�f�t�H���g�R���X�g���N�^�ƃR�s�[�R���X�g���N�^���폜����D���̂������ł��Ȃ��悤����
	MapConst() = delete;
	MapConst(MapConst& _other) = delete;

	const static std::string OUTPUT_FILE_NAME;
	
	constexpr static int FOOT_HOLD_XY_DIST = 20;		//z������݂��Ƃ��̑���i�r�ڒn�\�_�j�̊Ԋu[mm]�A�i�q�_��
	constexpr static int START_ROUGH_TARRAIN_Y = 400;	//���i�ړ��̂Ƃ��́A�s���n�Ɛ����ʂ̋��E[mm]

	constexpr static int HEIGHT_STEP = -140;	//�i������[mm]
	constexpr static int  DEPTH_STEP = 500;		//���s��[mm]

	constexpr static double THETA_SLOPE = 20;	//�ő�X�Ίp[��]	�Ζʂ̍ő�X�Ίp
	constexpr static double XI_SLOPE = 0;		//�ő�X�Ε��ʊp��[��]�@y��(�O���[�o��)�ƍő�X�Ε����ւ̃x�N�g���i���z�x�N�g���j�̂Ȃ��p �}90����89.9���炢�ɂ��Ƃ�

	constexpr static int WIDE_TRI = 400;		//2���ӎO�p�`�̒�ӂ̒���/2[mm]
	constexpr static int DEPTH_TRI = 400;		//2���ӎO�p�`�̉��s��
	constexpr static int THETA_TRI = 10;		//��p[��]

	constexpr static int LP_DIVIDE_NUM = 40;	//�r�ڒn�\�_�𕽕���������ۂ̂P�ӂ̕�����
	constexpr static int MAP_X_MIN = -1000;
	constexpr static int MAP_X_MAX = 1000;
	constexpr static int MAP_Y_MIN = -400;
	constexpr static int MAP_Y_MAX = 2600;
	constexpr static int MAPDATA3D_MAX = (MAP_X_MAX - MAP_X_MIN) / FOOT_HOLD_XY_DIST * (MAP_Y_MAX - MAP_Y_MIN) / FOOT_HOLD_XY_DIST;

	constexpr static int HOLE_RATE = 60;				//�s���n��̑�������O���銄���B�z�[����[%]
	constexpr static int HEIGHT_MAGNIFICATION = 10;		//���������̃����_���Ȕ{���ƍ��ݐ�[-]�@������Ŗ��O�ς���A�L�肻��������
	constexpr static int START_RANDOM_R = 200;			//�����ʒu�̃����_���Ŕz�u�����͈́A���a[mm]�B
	constexpr static int INVALID_FOOT_HOLD = -10000;	//�z�[�����ɂ������r�ݒu�\�_���΂����W

	constexpr static int SQUARE_SIZE = 100;		//��FOOT_HOLD_XY_DIST*n(n=1,2,3...)n=1�Ȃ�1�_���A2�Ȃ�4�_�An�Ȃ�n^2�_�̐����`���ΏۂɂȂ鐳���`�̈�Ђ̒���[mm]
};

