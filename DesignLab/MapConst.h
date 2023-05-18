#pragma once
#include <string>

// �����ɒ�`����Ă��鐔�l�͕ύX�͂��܂肵�Ȃ����́D
// �悭�ύX����ł��낤���l�� .cpp�̂ق��Œ�`���Ă���

class MapConst final
{
public:
	const static std::string INPUT_FILE_NAME;
	const static std::string OUTPUT_FILE_NAME;

	constexpr static int FOOT_HOLD_XY_DIST = 20;	//z������݂��Ƃ��̑���i�r�ڒn�\�_�j�̊Ԋu[mm]�A�i�q�_��
	constexpr static int MAP_X_MIN = -1000;			//�}�b�v�̉��̍ŏ��l�_
	constexpr static int MAP_X_MAX = 1000;			//�}�b�v�̉��̍ő�l�_
	constexpr static int MAP_Y_MIN = -400;			//�}�b�v�̏c�̍ŏ��l�_
	constexpr static int MAP_Y_MAX = 2600;			//�}�b�v�̏c�̍ő�l�_
	constexpr static int MAPDATA3D_MAX = (MAP_X_MAX - MAP_X_MIN) / FOOT_HOLD_XY_DIST * (MAP_Y_MAX - MAP_Y_MIN) / FOOT_HOLD_XY_DIST;

	constexpr static int START_ROUGH_TARRAIN_Y = 400;	//���i�ړ��̂Ƃ��́A�s���n�Ɛ����ʂ̋��E[mm]
	constexpr static int STRIPE_INTERVAL = 5;			//�e��͗l�⌊���쐬����ہC����Ŏw�肵���}�X����1�ӂ��������`��ɂ��Ȃ�������D

	const static unsigned int HOLE_RATE;	//�s���n��̑�������O���銄���B�z�[����[%]
	const static double STEP_HEIGHT;		//�i������[mm]�D���̒l�ɂ���Ɖ���̊K�i�ɂȂ�D
	const static double STEP_LENGTH;		//�K�i�c��[mm]
	const static double SLOPE_ANGLE;		//�Ζʂ̌X�Ίp[deg]�D
	const static double TILT_ANGLE;			//�n�`���X����p�x[deg]�D
	const static double ROUGH_MAX_HEIGHT;	//�f�R�{�R�Ȓn�`�̍ő卂��[mm]
	const static double ROUGH_MIN_HEIGHT;	//�f�R�{�R�Ȓn�`�̍ŏ�����[mm]

	constexpr static int LP_DIVIDE_NUM = 40;			//�r�ڒn�\�_�𕽕���������ۂ̂P�ӂ̕�����
	constexpr static int START_RANDOM_R = 200;			//�����ʒu�̃����_���Ŕz�u�����͈́A���a[mm]�B
	constexpr static int INVALID_FOOT_HOLD = -10000;	//�z�[�����ɂ������r�ݒu�\�_���΂����W

private:
	//�f�t�H���g�R���X�g���N�^�ƃR�s�[�R���X�g���N�^���폜����D���̂������ł��Ȃ��悤����
	MapConst() = delete;
	MapConst(MapConst& _other) = delete;
};

