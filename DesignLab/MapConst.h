#pragma once

class MapConst
{
public:
	//�f�t�H���g�R���X�g���N�^�ƃR�s�[�R���X�g���N�^���폜����D���̂������ł��Ȃ��悤����
	MapConst() = delete;
	MapConst(MapConst& _other) = delete;

	//constexpr int 
	constexpr static int FOOT_HOLD_XY_DIST = 20;		//z������݂��Ƃ��̑���i�r�ڒn�\�_�j�̊Ԋu[mm]�A�i�q�_��
	constexpr static int START_ROUGH_TARRAIN_Y = 400;	//���i�ړ��̂Ƃ��́A�s���n�Ɛ����ʂ̋��E[mm]
};

