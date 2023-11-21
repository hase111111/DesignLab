//! @file hexapod_const.h
//! @brief Hexapod�̒萔���܂Ƃ߂��N���X�D


#ifndef DESIGNLAB_HEXAPOD_CONST_H_
#define DESIGNLAB_HEXAPOD_CONST_H_


#include "designlab_math_util.h"


//! @class HexapodConst
//! @brief Hexapod�̒萔���܂Ƃ߂��N���X�D
//! @n ��s�����̃}�N�����܂Ƃ߂����́C���X�ɂ�������l�������Ă����D
//! @details �R���X�g���N�^���폜�����̂ŁC���̂͐����ł��Ȃ��D( HexapodConst::kLegNum �݂����ɒl���Ăяo������ )
class HexapodConst final
{
public:

	//�R���X�g���N�^�ƃR�s�[�R���X�g���N�^���폜���C���̂𐶐��ł��Ȃ��悤�ɂ���D
	HexapodConst() = delete;
	HexapodConst(const HexapodConst& other) = delete;
	HexapodConst& operator=(const HexapodConst& other) = delete;
	HexapodConst(HexapodConst&& other) = delete;


	constexpr static int kLegNum = 6;	//!< Hexapod�̋r�̖{����\���D�����ύX���Ă��r�̖{�����ύX�ł���킯�ł͂Ȃ��D�}�W�b�N�i���o�[���Ȃ������Ƃ��ړI�D

};


#endif	// DESIGNLAB_HEXAPOD_CONST_H_