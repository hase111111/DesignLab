#include "map_const.h"

const std::string MapConst::INPUT_FILE_NAME = "map.csv";
const std::string MapConst::OUTPUT_FILE_NAME = "map.csv";

const unsigned int MapConst::HOLE_RATE = 20;

const float MapConst::STEP_HEIGHT = -120.0f;		//�i������[mm]�D���̒l�ɂ���Ɖ���̊K�i�ɂȂ�D
const float MapConst::STEP_LENGTH = 500.0f;		//�K�i�c��[mm]

const float MapConst::SLOPE_ANGLE = 5.0f;		//�X�Ίp[deg]�D
const float MapConst::TILT_ANGLE = 10.0f;		//�n�`���X����p�x[deg]�D

const float MapConst::ROUGH_MAX_HEIGHT = 30.0f;	//�f�R�{�R�Ȓn�`�̍ő卂��
const float MapConst::ROUGH_MIN_HEIGHT = -30.0f;//�f�R�{�R�Ȓn�`�̍ŏ�����
