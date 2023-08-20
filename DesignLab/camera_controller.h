#pragma once

#include "DxLib.h"

#include "my_rotator.h"
#include "my_math.h"


//! @enum ECameraMode
//! @date 2023/08/14
//! @author ���J��
//! @brief �J�����̎��_��\���񋓑́D
//! @details CameraController�N���X�Ŏg�p����Ă���D
//! ��̓I�ȏ�����CameraController�N���X���Q�Ƃ��邱�ƁD
enum class ECameraMode : int
{
	FRONT_VIEW,			//!< ���ʐ}
	BACK_VIEW,			//!< �w�ʐ}
	TOP_VIEW,			//!< �ォ��̌����낵���_�D
	RIGHT_SIDE_VIEW,	//!< �E����^���̎��_�D
	LEFT_SIDE_VIEW,		//!< �E����^���̎��_�D
	FREE_CONTROLLED,	//!< ���R�ɑ���\
};


//! @class CameraController
//! @date 2023/08/14
//! @author ���J��
//! @brief Dxlib��3D�̃J�����̏������s���N���X
//! @details �J�������C�ォ�猩��̂��C�����猩��̂��C�؂�ւ���̂�DXlib�̏ꍇ���삪���X���
//! @n ���̃N���X�͂��̏������܂Ƃ߂����ƂŁC�������ȒP�ɂ��Ă���
class CameraController final
{
public:

	CameraController();
	~CameraController() = default;

	//! @brief �J�����̈ʒu�Ȃǂ̍X�V���s���D���t���[�����s���邱��
	void update();

	//! @brief �J�����̒�������ڕW�̍��W���Z�b�g����
	//! @param [in] pos �J�����̒�������ڕW�̍��W
	void setTargetPos(const VECTOR pos) { m_goal_target_pos = pos; }

	//! @brief �J�����̃��[�h���Z�b�g����
	//! @param [in] mode �J�����̎��_�̃��[�h
	void setCameraViewMode(const ECameraMode mode);

	//! @brief �J�����̒�������ڕW�̍��W�ƃJ�����̋����𑝂₷
	//! @param [in] length_dif ���₷����
	void addCameraToTargetLength(const float length_dif);

private:

	//�J������ڕW�l�ɏ��X�ɋ߂Â���̂Ɏg�p����D�߂�l�͈ړ����VECTOR
	VECTOR approachTargetVECTOR(const VECTOR& current, const VECTOR& target) const;

	//�J������ڕW�l�ɏ��X�ɋ߂Â���̂Ɏg�p����D��̊֐����X�ɂ������߂Â��D
	VECTOR approachSlowlyTargetVECTOR(const VECTOR& current, const VECTOR& target) const;

	my_vec::SRotator approachTargetRotator(const my_vec::SRotator& current, const my_vec::SRotator& target) const;

	//�����̒l�����X�ɖڕW�l�ɋ߂Â���D
	float approachTargetValue(const float current, const float target) const;

	//�����̒l�����X�ɖڕW�l�ɋ߂Â���D��̊֐����X�ɂ������߂Â��D
	float approachSlowlyTargetValue(const float current, const float target) const;


	const float CAMERA_SLOW_SPEED = 0.1f;				//�J�����̏�����̃x�N�g���͒P�ʃx�N�g���ł���̂ŁC��2�̒l���Ƒ�������D
	const float CAMERA_SPEED = 3.0f;					//�J�������ړ����鑬���D
	const float CAMERA_HIGH_SPEED = 25.0f;				//�J�����������ňړ�����Ƃ��̑���.
	const float CAMERA_HIGH_DIF = 150.0f;				//�ڕW���W�Ƃ��̒l�ȏ�ɗ���Ă���Ȃ�΁CCAMERA_HIGH_SPEED�̒l�ŋ}���ŋ߂Â��D
	const float CAMERA_ANGLE = my_math::MY_FLT_PI / 4;	//�΂߂���̃J�����̊p�x
	const my_vec::SVector kDefaultCameraFrontVec = { 1.0f, 0.0f, 0.0f };	//�f�t�H���g�̃J�����̕�����\���P�ʃx�N�g��

	ECameraMode m_camera_view_mode;			//�J�����̎��_�����肷��D

	VECTOR m_goal_target_pos;	//�J����������������W�̖ڕW�l
	float m_goal_length_camera_to_target;	//�J�����ƒ�������ΏۂƂ̋����̖ڕW�l
	VECTOR m_goal_upvec;		//�J�����̏�����̃x�N�g���̖ڕW�l
	my_vec::SRotator m_goal_camera_rot;	//�J�����̉�]�̖ڕW�l

	VECTOR m_target_pos;	//�J���������ݒ������Ă�����W
	float m_length_camera_to_target;	//�J�����ƒ�������ΏۂƂ̋����̌��ݒl
	VECTOR m_camera_upvec;	//�J�����̏�����̃x�N�g���̌��ݒl
	my_vec::SRotator m_camera_rot;	//�J�����̉�]�̖ڕW�l
};


//! @file camera_controller.h
//! @date 2023/08/14
//! @author ���J��
//! @brief �摜�\�������ɂ����ăJ�����𐧌䂷��N���X�D
//! @n �s�� : @lineinfo
