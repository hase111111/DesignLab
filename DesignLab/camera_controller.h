#pragma once

#include "DxLib.h"

#include "my_vector.h"
#include "my_math.h"


//! @enum ECameraMode
//! @date 2023/08/14
//! @author ���J��
//! @brief �J�����̎��_��\������񋓑́D
//! @details CameraController�N���X�ł̂ݎg�p����Ă���D
//! ��̓I�ȏ�����CameraController�N���X���Q�Ƃ��邱�ƁD
enum class ECameraMode : int
{
	TOP_VIEW,			//!< �ォ��̌����낵���_�D
	OVERHEAD_VIEW,		//!< �E�΂ߏォ��̌����낵���_�D
	SIDE_VIEW,			//!< �E����^���̎��_�D
	OVERHEAD_VIEW_FLIP,	//!< ���΂ߏォ��̌����낵���_�D
	SIDE_VIEW_FLIP,		//!< ������^���̎��_�D
};


//! @class CameraController
//! @date 2023/08/14
//! @author ���J��
//! @brief Dxlib��3D�̃J�����̏������s���N���X�D
//! @details �J�������C�ォ�猩��̂��C�����猩��̂��C�؂�ւ���̂�DXlib�̏ꍇ���삪���X����D
//! @n ���̃N���X�͂��̏������܂Ƃ߂����ƂŁC�������ȒP�ɂ��Ă���D
class CameraController final
{
public:

	CameraController();
	~CameraController() = default;

	//! @brief �J�����̈ʒu�Ȃǂ̍X�V���s���D���t���[�����s���邱�ƁD
	void update();

	//! @brief �J�����̒�������ڕW�̍��W���Z�b�g����D
	//! @param [in] pos �J�����̒�������ڕW�̍��W
	inline void setTargetPos(const VECTOR pos) { m_goal_target_pos = pos; }

	//! @brief �J�����̃��[�h���Z�b�g����D
	//! @param [in] mode �J�����̎��_�̃��[�h
	inline void setCameraMode(const ECameraMode mode) { m_mode = mode; }

private:

	const float CAMERA_SLOW_SPEED = 0.02f;				//�J�����̏�����̃x�N�g���͒P�ʃx�N�g���ł���̂ŁC��2�̒l���Ƒ�������D
	const float CAMERA_SPEED = 3.0f;					//�J�������ړ����鑬���D
	const float CAMERA_HIGH_SPEED = 25.0f;				//�J�����������ňړ�����Ƃ��̑���.
	const float CAMERA_HIGH_DIF = 150.0f;				//�ڕW���W�Ƃ��̒l�ȏ�ɗ���Ă���Ȃ�΁CCAMERA_HIGH_SPEED�̒l�ŋ}���ŋ߂Â��D
	const float CAMERA_ANGLE = my_math::MY_FLT_PI / 4;	//�΂߂���̃J�����̊p�x


	//�J�����̖ڕW���W��ݒ肷��D���̍��W�ɏ��X�ɋ߂Â��悤�ɃJ�����͈ړ�����D
	void setGoalCameraPos();

	//�J������ڕW�l�ɏ��X�ɋ߂Â���̂Ɏg�p����D�߂�l�͈ړ����VECTOR
	VECTOR approachTargetVECTOR(const VECTOR& current, const VECTOR& target) const;

	//�J������ڕW�l�ɏ��X�ɋ߂Â���̂Ɏg�p����D��̊֐����X�ɂ������߂Â��D
	VECTOR approachSlowlyTargetVECTOR(const VECTOR& current, const VECTOR& target) const;

	//�����̒l�����X�ɖڕW�l�ɋ߂Â���D
	float approachTargetValue(const float current, const float target) const;

	//�����̒l�����X�ɖڕW�l�ɋ߂Â���D��̊֐����X�ɂ������߂Â��D
	float approachSlowlyTargetValue(const float current, const float target) const;


	ECameraMode m_mode;			//�J�����̎��_�����肷��D

	VECTOR m_goal_upvec;		//�J�����̏�����̃x�N�g���̖ڕW�l
	VECTOR m_goal_pos;			//�J�����̈ʒu�̖ڕW�l
	VECTOR m_goal_target_pos;	//�J��������������ڕW�̍��W�D

	VECTOR m_now_target_pos;	//�J��������������ڕW�̍��W�D
	VECTOR m_now_camera_upvec;	//�J�����̌��݂̏�����D
	VECTOR m_now_camera_pos;	//�J�����̌��݂̍��W�D
	float m_length_camera_to_target;	//�J�����ƖڕW���Ƃ̋����D
};


//! @file camera_controller.h
//! @date 2023/08/14
//! @author ���J��
//! @brief �摜�\�������ɂ����ăJ�����𐧌䂷��N���X�D
//! @n �s�� : @lineinfo
