#pragma once

#include "DxLib.h"

#include "my_vector.h"
#include "my_math.h"


enum class ECameraMode : int
{
	TopView,			//!< �ォ��̌����낵���_�D
	OverheadView,		//!< �E�΂ߏォ��̌����낵���_�D
	SideView,			//!< �E����^���̎��_�D
	OverheadViewFlip,	//!< ���΂ߏォ��̌����낵���_�D
	SideViewFlip,		//!< ������^���̎��_�D
};


class CameraController final
{
private:
	ECameraMode m_mode;			//�J�����̎��_�����肷��D

	VECTOR m_goal_upvec;		//�J�����̏�����̃x�N�g���̖ڕW�l
	VECTOR m_goal_pos;			//�J�����̈ʒu�̖ڕW�l
	VECTOR m_goal_target_pos;	//�J��������������ڕW�̍��W�D

	VECTOR m_now_target_pos;	//�J��������������ڕW�̍��W�D
	VECTOR m_now_camera_upvec;	//�J�����̌��݂̏�����D
	VECTOR m_now_camera_pos;	//�J�����̌��݂̍��W�D
	float m_length_camera_to_target;	//�J�����ƖڕW���Ƃ̋����D

public:

	CameraController();
	~CameraController() = default;

	//! @brief �J�����̈ʒu�Ȃǂ̍X�V���s���D���t���[�����s���邱�ƁD
	void update();

	//! @brief �J�����̒�������ڕW�̍��W���Z�b�g����D
	//! @param [in] _pos �J�����̒�������ڕW�̍��W
	inline void setTargetPos(const VECTOR _pos) { m_goal_target_pos = _pos; }

	//! @brief �J�����̃��[�h���Z�b�g����D
	//! @param [in] _mode �J�����̎��_�̃��[�h
	inline void setCameraMode(const ECameraMode _mode) { m_mode = _mode; }

private:
	const float CAMERA_SLOW_SPEED = 0.02f;				//�J�����̏�����̃x�N�g���͒P�ʃx�N�g���ł���̂ŁC��2�̒l���Ƒ�������D
	const float CAMERA_SPEED = 3.0f;					//�J�������ړ����鑬���D
	const float CAMERA_HIGH_SPEED = 25.0f;				//�J�����������ňړ�����Ƃ��̑���.
	const float CAMERA_HIGH_DIF = 150.0f;				//�ڕW���W�Ƃ��̒l�ȏ�ɗ���Ă���Ȃ�΁CCAMERA_HIGH_SPEED�̒l�ŋ}���ŋ߂Â��D
	const float CAMERA_ANGLE = my_math::MY_FLT_PI / 4;	//�΂߂���̃J�����̊p�x

	//�J�����̖ڕW���W��ݒ肷��D���̍��W�ɏ��X�ɋ߂Â��悤�ɃJ�����͈ړ�����D
	void setGoalCameraPos();

	//�J������ڕW�l�ɏ��X�ɋ߂Â���̂Ɏg�p����D�߂�l�͈ړ����VECTOR
	VECTOR approachTargetVECTOR(const VECTOR _current, const VECTOR _target) const;

	//�J������ڕW�l�ɏ��X�ɋ߂Â���̂Ɏg�p����D��̊֐����X�ɂ������߂Â��D
	VECTOR approachSlowlyTargetVECTOR(const VECTOR _current, const VECTOR _target) const;

	//�����̒l�����X�ɖڕW�l�ɋ߂Â���D
	float approachTargetValue(const float _current, const float _target) const;

	//�����̒l�����X�ɖڕW�l�ɋ߂Â���D��̊֐����X�ɂ������߂Â��D
	float approachSlowlyTargetValue(const float _current, const float _target) const;
};


//! @file CameraController.h
//! @brief �摜�\�������ɂ����ăJ�����𐧌䂷��N���X�D
//! @author ���J��

//! @enum ECameraMode
//! @brief �J�����̎��_��\������񋓑́D
//! @details CameraController�N���X�ł̂ݎg�p����Ă���D
//! ��̓I�ȏ�����CameraController�N���X���Q�Ƃ��邱�ƁD
//! @author ���J��

//! @class CameraController
//! @brief Dxlib��3D�̃J�����̏������s���N���X�D
//! @details �J�������C�ォ�猩��̂��C�����猩��̂��C�؂�ւ���̂�DXlib�̏ꍇ���삪���X����D<br>
//! ���̃N���X�͂��̏������܂Ƃ߂����ƂŁC�������ȒP�ɂ��Ă���D
//! @author ���J��
