#pragma once

#include "DxLib.h"

#include "designlab_quaternion.h"
#include "designlab_math.h"


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

	//�����̃N�H�[�^�j�I�������X�ɖڕW�l�ɋ߂Â���D��ԕ��@�͐��`��� https://www.f-sp.com/entry/2017/06/30/221124
	dl_vec::SQuaternion approachTargetQuat(const dl_vec::SQuaternion& current, const dl_vec::SQuaternion& target) const;

	//�����̃x�N�g�������X�ɖڕW�l�ɋ߂Â���
	VECTOR approachTargetVECTOR(const VECTOR& current, const VECTOR& target) const;

	//�����̒l�����X�ɖڕW�l�ɋ߂Â���D
	float approachTargetValue(const float current, const float target) const;


	const dl_vec::SVector kDefaultCameraFrontVec = { 1.0f, 0.0f, 0.0f };	//�f�t�H���g�̃J�����̕�����\���P�ʃx�N�g��
	const dl_vec::SVector kDefaultCameraUpVec = { 0.0f, 0.0f, 1.0f };		//�f�t�H���g�̃J�����̕�����\���P�ʃx�N�g��

	ECameraMode m_camera_view_mode;			//�J�����̎��_�����肷��D

	VECTOR m_goal_target_pos;					//�J����������������W�̖ڕW�l
	dl_vec::SQuaternion m_goal_camera_rot_quat;	//�J�����̉�]��\���N�H�[�^�j�I��
	float m_goal_length_camera_to_target;		//�J�����ƒ�������ΏۂƂ̋����̖ڕW�l

	VECTOR m_target_pos;					//�J���������ݒ������Ă�����W
	dl_vec::SQuaternion m_camera_rot_quat;	//�J�����̉�]��\���N�H�[�^�j�I��
	float m_length_camera_to_target;		//�J�����ƒ�������ΏۂƂ̋����̌��ݒl
};


//! @file camera_controller.h
//! @date 2023/08/14
//! @author ���J��
//! @brief �摜�\�������ɂ����ăJ�����𐧌䂷��N���X�D
//! @n �s�� : @lineinfo
