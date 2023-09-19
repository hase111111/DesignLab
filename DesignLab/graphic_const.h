//! @file graphic_const.h
//! @brief �摜�\�������p�̒萔�D

#ifndef DESIGNLAB_GRAPHIC_CONST_H_
#define DESIGNLAB_GRAPHIC_CONST_H_

#include <string>


//! @class GraphicConst
//! @brief �摜�\�������p�̒萔���܂Ƃ߂����́D
//! @details �萔�N���X�̏ڍׂ�define.h���Q�ƁD
//! @n �J�����Ɋւ���l��ύX���鎞�͐T�d�ɍs�����ƁC����������Ɖ��������Ȃ��Ȃ邵�C�グ������Ɛq�킶��Ȃ��d���Ȃ�܂��D

class GraphicConst final
{
public:

	//�R���X�g���N�^���폜���āC���̂𐶐��ł��Ȃ��悤�ɂ���D
	GraphicConst() = delete;
	GraphicConst(GraphicConst& other) = delete;
	GraphicConst(GraphicConst&& other) = delete;
	GraphicConst& operator=(GraphicConst& other) = delete;


	const static std::string WIN_NAME;	//!< �E�B���h�E�̖��O�D
	const static int COLOR_BIT;			//!< �F��\������bit���D�ʏ�32�ŗǂ����y������Ȃ�16�ɂ���D

	const static int BACK_COLOR_R;		//!< �E�B���h�E�w�i�F�D �ԐF�����D0�`255�͈̔͂Ŏw��D
	const static int BACK_COLOR_G;		//!< �E�B���h�E�w�i�F�D �ΐF�����D0�`255�͈̔͂Ŏw��D
	const static int BACK_COLOR_B;		//!< �E�B���h�E�w�i�F�D �F�����D0�`255�͈̔͂Ŏw��D

	const static float CAMERA_FAR;		//!< �J�������\���ł���ł��������W�D
	const static float CAMERA_NEAR;		//!< �J�������\���ł���ł��߂����W�D

	const static float CAMERA_TO_TARGET_MAX;	//!< �J�����ƒ����ڕW�̍ő勗���DCAMERA_FAR��CAMERA_NEAR�̊Ԃ̒l����Ȃ��ƂȂɂ��\������Ȃ��Ȃ�D
	const static float CAMERA_TO_TARGET_MIN;	//!< �J�����ƒ����ڕW�̍ŏ������DCAMERA_FAR��CAMERA_NEAR�̊Ԃ̒l����Ȃ��ƂȂɂ��\������Ȃ��Ȃ�D
};


#endif // !DESIGNLAB_GRAPHIC_CONST_H_