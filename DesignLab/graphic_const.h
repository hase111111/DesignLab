#pragma once

#include <string>


//! @class GraphicConst
//! @date 2023/08/08
//! @author ���J��
//! @brief �摜�\�������p�̒萔���܂Ƃ߂����́D
//! @details �萔�N���X�̏ڍׂ�Define.h���Q�ƁD@n 
//! @n ���̃N���X�̒��ӓ_�Ƃ��ẮC�܂��CWIN_X, WIN_Y�ɑ傫������l�������Ȃ����ƁD�܂��C16:9�̉�ʔ�͕����Ȃ��ق��������Ǝv���D
//! @n ���ɃJ�����Ɋւ���l��ύX���鎞�͐T�d�ɍs�����ƁC����������Ɖ��������Ȃ��Ȃ邵�C�グ������Ɛq�킶��Ȃ��d���Ȃ�܂��D
class GraphicConst final
{
public:
	const static std::string WIN_NAME;	//!< �E�B���h�E�̖��O�D
	const static int WIN_X;				//!< �E�B���h�E�̉����Ddxlib�ł̓E�B���h�E�̉������ɁC�E�����𐳂Ƃ��� X �����Ƃ�܂��D
	const static int WIN_Y;				//!< �E�B���h�E�̏c���Ddxlib�ł̓E�B���h�E�̏c�����ɁC�������𐳂Ƃ��� Y �����Ƃ�܂��D
	const static int COLOR_BIT;			//!< �F��\������bit���D�ʏ�32�ŗǂ����y������Ȃ�16�ɂ���D

	const static int BACK_COLOR_R;		//!< �E�B���h�E�w�i�F�D �ԐF����
	const static int BACK_COLOR_G;		//!< �E�B���h�E�w�i�F�D �ΐF����
	const static int BACK_COLOR_B;		//!< �E�B���h�E�w�i�F�D �F����

	const static float CAMERA_FAR;		//!< �J�������\���ł���ł��������W�D
	const static float CAMERA_NEAR;		//!< �J�������\���ł���ł��߂����W�D

	const static float CAMERA_TO_TARGET_MAX;	//!< �J�����ƒ����ڕW�̍ő勗���DCAMERA_FAR��CAMERA_NEAR�̊Ԃ̒l����Ȃ��ƂȂɂ��\������Ȃ��Ȃ�D

	//! �E�B���h�E��FPS(�t���[�� / �b�D�b�ԉ�ʂ�����X�V���邩�DTV��30fps�CNintendo Switch��60fps)�D
	//! 60���傫���l�ɂ��Ă��Ӗ��͂Ȃ��D30�Ƃ��ɂ��Ă��������ǂ����܂ŏ������ׂ͕ς��Ȃ��Ǝv���D
	constexpr static unsigned int GRAPHIC_FPS = 60;

private:

	//�R���X�g���N�^�ƃR�s�[�R���X�g���N�^���폜���Ď��̂𐶐��ł��Ȃ��悤�ɂ���D
	GraphicConst() = delete;
	GraphicConst(GraphicConst& other) = delete;
};


//! @file graphic_const.h
//! @date 2023/08/08
//! @author ���J��
//! @brief �摜�\�������p�̒萔�D
//! @n �s�� : @lineinfo