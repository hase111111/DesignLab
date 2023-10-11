//! @file graphic_const.h
//! @brief �摜�\�������p�̒萔�D

#ifndef DESIGNLAB_GRAPHIC_CONST_H_
#define DESIGNLAB_GRAPHIC_CONST_H_

#include <string>

#include "cassert_define.h"


//! @class GraphicConst
//! @brief �摜�\�������p�̒萔���܂Ƃ߂����́D
//! @details �萔�N���X�̏ڍׂ�define.h���Q�ƁD
//! @n �J�����Ɋւ���l��ύX���鎞�͐T�d�ɍs�����ƁC����������Ɖ��������Ȃ��Ȃ邵�C�グ������Ɛq�킶��Ȃ��d���Ȃ�D
//! @n �l��ҏW�������ꍇ�� graphic_const cpp ��ҏW���邱�ƁD
class GraphicConst final
{
public:

	//�R���X�g���N�^���폜���āC���̂𐶐��ł��Ȃ��悤�ɂ���D
	GraphicConst() = delete;
	GraphicConst(GraphicConst& other) = delete;
	GraphicConst(GraphicConst&& other) = delete;
	GraphicConst& operator=(GraphicConst& other) = delete;


	const static std::string kWindowName;	//!< �E�B���h�E�̖��O�D
	const static int kColorBit;				//!< �F��\������bit���D�ʏ�32�ŗǂ����y������Ȃ�16�ɂ���D

	const static int kBackColorRed;		//!< �E�B���h�E�w�i�F�D �ԐF�����D0�`255�͈̔͂Ŏw��D
	const static int kBackColorGreen;	//!< �E�B���h�E�w�i�F�D �ΐF�����D0�`255�͈̔͂Ŏw��D
	const static int kBackColorBlue;	//!< �E�B���h�E�w�i�F�D �F�����D0�`255�͈̔͂Ŏw��D

	const static float kCameraFar;		//!< �J�������\���ł���ł��������W�܂ł̋����D
	const static float kCameraNear;		//!< �J�������\���ł���ł��߂����W�܂ł̋����D

	const static float kCameraToTargetMax;	//!< �J�����ƒ����ڕW�̍ő勗���DCAMERA_FAR��CAMERA_NEAR�̊Ԃ̒l����Ȃ��ƂȂɂ��\������Ȃ��Ȃ�D
	const static float kCameraToTargetMin;	//!< �J�����ƒ����ڕW�̍ŏ������DCAMERA_FAR��CAMERA_NEAR�̊Ԃ̒l����Ȃ��ƂȂɂ��\������Ȃ��Ȃ�D
};


#endif // !DESIGNLAB_GRAPHIC_CONST_H_