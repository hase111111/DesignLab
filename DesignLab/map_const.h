//! @file map_const.h
//! @brief �n�`��Ԃ�\���萔�̎����D


#pragma once

#include <string>

//! @todo : ��ŏ���

//! @class MapConst
//! @brief �n�`��Ԃ�\���萔�D
//! @details �n�`�̃f�[�^�̕\�����@��C�ǂݎ������`���Ă���D 
//! @n �w�b�_�[�ɒ�`����Ă��鐔�l�͕ύX�͂��܂肵�Ȃ����́D
//! @n �悭�ύX����ł��낤���l�� .cpp�̂ق��Œ�`���Ă���D
class MapConst final
{
public:

	constexpr static int STRIPE_INTERVAL = 5;			//!< �e��͗l�⌊���쐬����ہC����Ŏw�肵���}�X����1�ӂ��������`��ɂ��Ȃ�������D

	const static unsigned int HOLE_RATE;		//!< �s���n��̑�������O���銄���B�z�[����[%]
	const static float STEP_HEIGHT;				//!< �i������[mm]�D���̒l�ɂ���Ɖ���̊K�i�ɂȂ�D
	const static float STEP_LENGTH;				//!< �K�i�c��[mm]
	const static float SLOPE_ANGLE;				//!< �Ζʂ̌X�Ίp[deg]�D
	const static float TILT_ANGLE;				//!< �n�`���X����p�x[deg]�D
	const static float ROUGH_MAX_HEIGHT;		//!< �f�R�{�R�Ȓn�`�̍ő卂��[mm]
	const static float ROUGH_MIN_HEIGHT;		//!< �f�R�{�R�Ȓn�`�̍ŏ�����[mm]

private:

	//�f�t�H���g�R���X�g���N�^�ƃR�s�[�R���X�g���N�^���폜����D���̂������ł��Ȃ��悤����
	MapConst() = delete;
	MapConst(MapConst& _other) = delete;
};