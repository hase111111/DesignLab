#pragma once

class MapConst
{
public:
	//�f�t�H���g�R���X�g���N�^�ƃR�s�[�R���X�g���N�^���폜����D���̂������ł��Ȃ��悤����
	MapConst() = delete;
	MapConst(MapConst& _other) = delete;

	
	constexpr static int FOOT_HOLD_XY_DIST = 20;		//z������݂��Ƃ��̑���i�r�ڒn�\�_�j�̊Ԋu[mm]�A�i�q�_��
	constexpr static int START_ROUGH_TARRAIN_Y = 400;	//���i�ړ��̂Ƃ��́A�s���n�Ɛ����ʂ̋��E[mm]

	constexpr static int  LP_DIVIDE_NUM = 40;			//�r�ڒn�\�_�𕽕���������ۂ̂P�ӂ̕�����
	constexpr static int MAP_X_MIN = -1000;
	constexpr static int MAP_X_MAX = 1000;
	constexpr static int MAP_Y_MIN = -400;
	constexpr static int MAP_Y_MAX = 2600;
};

