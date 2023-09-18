#pragma once

#include "DxLib.h"

#include "designlab_quaternion.h"
#include "designlab_math.h"


//! @enum CameraViewMode
//! @brief �J�����̎��_��\���񋓑́D
//! @details CameraController�N���X�Ŏg�p����Ă���D
//! @n �J�����̎��_��؂�ւ���ۂɎg�p����D
//! @n ��̓I�ȏ�����CameraController�N���X���Q�Ƃ��邱�ƁD

enum class CameraViewMode
{
	kFrontView,			//!< ���ʂ���̎��_�D
	kBackView,			//!< �w�ʂ���̎��_�D
	kTopView,			//!< �ォ��̌����낵���_�D
	kRightSideView,		//!< �E����^���̎��_�D
	kLeftSideView,		//!< �E����^���̎��_�D
	FREE_CONTROLLED,	//!< ���R�ɑ���\
	FREE_CONTROLLED_TARGET	//!< ���R�ɑ���\�������_��ݒ�\
};


//! @class CameraStateManager
//! @brief Dxlib��3D�̃J�����̏������s���N���X
//! @details �J�������C�ォ�猩��̂��C�����猩��̂��C�؂�ւ���̂�DXlib�̏ꍇ���삪���X���
//! @n ���̃N���X�͂��̏������܂Ƃ߂����ƂŁC�������ȒP�ɂ��Ă���
//! @n �܂��C�J�����̎p���̓N�H�[�^�j�I���ŕ\���Ă���D

class CameraStateManager final
{
public:

	CameraStateManager();

	//! @brief �J�����̈ʒu�Ȃǂ̍X�V���s���D���t���[�����s���邱��
	void Update();


	//! @brief �J�����̃��[�h���Z�b�g����D�����ɃJ�����̖ڕW��]�p�x�Ȃǂ�ݒ肷��
	//! @param [in] mode �J�����̎��_�̃��[�h
	void setCameraViewMode(const CameraViewMode mode);

	//! @brief �J�����̃��[�h���擾����
	//! @return CameraViewMode �J�����̎��_�̃��[�h
	CameraViewMode getCameraViewMode() const { return m_camera_view_mode; }


	//! @brief �J�����ƒ����_�Ƃ̋���������������
	void initCaneraTargetLength();

	//! @brief �J�����̒�������ڕW�̍��W�ƃJ�����̋����𑝂₷
	//! @param [in] length_dif ���₷����
	void addCameraToTargetLength(const float length_dif);


	//! @brief �J�����̒�������ڕW�̍��W���Z�b�g����
	//! @n camera��mode��FREE_CONTROLLED_TARGET�̎��̓Z�b�g�ł��Ȃ�
	//! @param [in] pos �J�����̒�������ڕW�̍��W
	void setTargetPos(const VECTOR pos) { m_goal_target_pos = pos; }


	//! @brief �J�����̃N�H�[�^�j�I�����擾����
	//! @return �J�����̃N�H�[�^�j�I��
	dl_vec::SQuaternion getCameraRotQuat() const { return m_goal_camera_rot_quat; }

	//! @brief �J�����̃N�H�[�^�j�I�����Z�b�g����
	//! @param [in] quat �J�����̃N�H�[�^�j�I��
	void setCameraRotQuat(const dl_vec::SQuaternion& quat) { m_goal_camera_rot_quat = quat; }


	//! @brief �����_�𑀍삷��ۂ́C�J�����̒���������W���Z�b�g����
	//! @param [in] pos �J�����̒���������W
	void setFreeTargetPos(const VECTOR pos) { m_free_controlled_target = pos; }

	//! @brief �����_�𑀍삷��ۂ́C�J�����̒���������W���擾����
	VECTOR getFreeTargetPos() const { return m_free_controlled_target; }


private:

	//�J�����̈ʒu�Ǝp����dxlib�̊֐��ŃZ�b�g����	
	void setCameraPosAndRot();

	//�����̃N�H�[�^�j�I�������X�ɖڕW�l�ɋ߂Â���D��ԕ��@�͐��`��� https://www.f-sp.com/entry/2017/06/30/221124
	dl_vec::SQuaternion approachTargetQuat(const dl_vec::SQuaternion& current, const dl_vec::SQuaternion& target) const;

	//�����̃x�N�g�������X�ɖڕW�l�ɋ߂Â���
	VECTOR approachTargetVECTOR(const VECTOR& current, const VECTOR& target) const;

	//�����̒l�����X�ɖڕW�l�ɋ߂Â���D
	float approachTargetValue(const float current, const float target) const;


	const dl_vec::SVector kDefaultCameraFrontVec = { 1.0f, 0.0f, 0.0f };	//�f�t�H���g�̃J�����̕�����\���P�ʃx�N�g��

	const dl_vec::SVector kDefaultCameraUpVec = { 0.0f, 0.0f, 1.0f };		//�f�t�H���g�̃J�����̕�����\���P�ʃx�N�g��


	CameraViewMode m_camera_view_mode;				//�J�����̎��_�����肷��D

	VECTOR m_goal_target_pos;					//�J����������������W�̖ڕW�l

	VECTOR m_target_pos;						//�J���������ݒ������Ă�����W

	dl_vec::SQuaternion m_goal_camera_rot_quat;	//�J�����̉�]��\���N�H�[�^�j�I��

	dl_vec::SQuaternion m_camera_rot_quat;		//�J�����̉�]��\���N�H�[�^�j�I���̌��ݒl

	float m_goal_length_camera_to_target;		//�J�����ƒ�������ΏۂƂ̋����̖ڕW�l

	float m_length_camera_to_target;			//�J�����ƒ�������ΏۂƂ̋����̌��ݒl

	VECTOR m_free_controlled_target;			//�J�����̒����_�𑀍삷��ۂ̒����_�̍��W
};


//! @file camera_manager.h
//! @date 2023/08/14
//! @author ���J��
//! @brief �摜�\�������ɂ����ăJ�����𐧌䂷��N���X�D
//! @n �s�� : @lineinfo
