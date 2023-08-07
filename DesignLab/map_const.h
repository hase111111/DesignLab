#pragma once

#include <string>

//! @class MapConst
//! @date 2023/08/06
//! @author ���J��
//! @brief �n�`��Ԃ�\���萔�D
//! @details �n�`�̃f�[�^�̕\�����@��C�ǂݎ������`���Ă���D 
//! @n �w�b�_�[�ɒ�`����Ă��鐔�l�͕ύX�͂��܂肵�Ȃ����́D
//! @n �悭�ύX����ł��낤���l�� .cpp�̂ق��Œ�`���Ă���D
class MapConst final
{
public:
	const static std::string INPUT_FILE_NAME;			//!< �o�͂����t�@�C���̖��O�D�p��̖��O�ɂ��邱�Ƃ𐄏��D
	const static std::string OUTPUT_FILE_NAME;			//!< �}�b�v�f�[�^���t�@�C������ǂݎ��Ƃ��ɁC�ǂݎ��t�@�C���̖��O�D

	constexpr static int FOOT_HOLD_XY_DIST = 20;		//!< z������݂��Ƃ��̑���i�r�ڒn�\�_�j�̊Ԋu[mm]�A�i�q�_��

	constexpr static int MAP_MIN_HORIZONTAL = -1000;	//!< �}�b�v�̉��̍ŏ��l�_
	constexpr static int MAP_MAX_HORIZONTAL = 1000;		//!< �}�b�v�̉��̍ő�l�_
	constexpr static int MAP_MIN_FORWARD = -400;		//!< �}�b�v�̏c�̍ŏ��l�_
	constexpr static int MAP_START_ROUGH = 400;			//!< �}�b�v�̏c�̕s���n�Ɛ����ʂ̋��E[mm]
	constexpr static int MAP_MAX_FORWARD = 2600;		//!< �}�b�v�̏c�̍ő�l�_
	constexpr static float MAX_Z_BASE = 0.0f;			//!< ���{�b�g���ŏ��ɑҋ@���Ă���ꏊ�� Z ���W�D

	constexpr static int STRIPE_INTERVAL = 5;			//!< �e��͗l�⌊���쐬����ہC����Ŏw�肵���}�X����1�ӂ��������`��ɂ��Ȃ�������D

	const static unsigned int HOLE_RATE;		//!< �s���n��̑�������O���銄���B�z�[����[%]
	const static float STEP_HEIGHT;				//!< �i������[mm]�D���̒l�ɂ���Ɖ���̊K�i�ɂȂ�D
	const static float STEP_LENGTH;				//!< �K�i�c��[mm]
	const static float SLOPE_ANGLE;				//!< �Ζʂ̌X�Ίp[deg]�D
	const static float TILT_ANGLE;				//!< �n�`���X����p�x[deg]�D
	const static float ROUGH_MAX_HEIGHT;		//!< �f�R�{�R�Ȓn�`�̍ő卂��[mm]
	const static float ROUGH_MIN_HEIGHT;		//!< �f�R�{�R�Ȓn�`�̍ŏ�����[mm]

	constexpr static int LP_DIVIDE_NUM = 40;	//!< �r�ڒn�\�_�𕽕���������ۂ̂P�ӂ̕�����

private:

	//�f�t�H���g�R���X�g���N�^�ƃR�s�[�R���X�g���N�^���폜����D���̂������ł��Ȃ��悤����
	MapConst() = delete;
	MapConst(MapConst& _other) = delete;
};

//! @file map_const.h
//! @date 2023/08/06
//! @author ���J��
//! @brief �n�`��Ԃ�\���萔�̎����D
//! @n �s�� : @lineinfo