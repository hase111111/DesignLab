//! @file camera_state_manager.h
//! @brief Dxlib��3D�̃J�����̏�Ԃ��Ǘ�����N���X�D


#ifndef DESIGNLAB_CAMERA_MANAGER_H_
#define DESIGNLAB_CAMERA_MANAGER_H_


#include <Dxlib.h>

#include "designlab_quaternion.h"


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
	kFreeControlled,	//!< ���R�ɑ���\
	kFreeControlledAndMovableTarget	//!< ���R�ɑ���\�������_��ݒ�\
};


//! @class CameraStateManager
//! @brief Dxlib��3D�̃J�����̏�Ԃ��Ǘ�����N���X�D
//! @details �J�������C�ォ�猩��̂��C�����猩��̂��C�؂�ւ���̂�DXlib�̏ꍇ���삪���X����D
//! @n �܂��CDxlib�͊֐��ŏ������������߁C���݂̃J�����̏�Ԃ�ێ����Ă܂Ƃ߂Ă��������̓f�t�H���g�ɂȂ��D
//! @n ���̃N���X�͂��̏������܂Ƃ߂����ƂŁC�������ȒP�ɂ��Ă���
//! @n �܂��C�J�����̎p���̓N�H�[�^�j�I���ŕ\���Ă���D
//! @n �f�t�H���g�� kDefaultCameraFrontVec�̕��������āCkDefaultCameraUpVec���J�����̏������\���D
class CameraStateManager final
{
public:

	CameraStateManager();

	//! @brief �J�����̈ʒu�Ȃǂ̍X�V���s���D���t���[�����s���邱��
	void Update();

	//! @brief �J�����ƒ����_�Ƃ̋���������������
	void InitCaneraTargetLength();

	//! @brief �J�����̒�������ڕW�̍��W����J�����܂ł̋����𑝂₷
	//! @param [in] length_dif ���₷����
	void AddCameraToTargetLength(float length_dif);


	//! @brief �J�����̃��[�h���Z�b�g����D�����ɃJ�����̖ڕW��]�p�x�Ȃǂ�ݒ肷��
	//! @param [in] mode �J�����̎��_�̃��[�h
	void SetCameraViewMode(CameraViewMode mode);

	//! @brief �J�����̃��[�h���擾����
	//! @return CameraViewMode �J�����̎��_�̃��[�h
	inline CameraViewMode GetCameraViewMode() const { return camera_view_mode_; }

	//! @brief �J�����̒�������ڕW�̍��W���Z�b�g����
	//! @n camera��mode��FREE_CONTROLLED_TARGET�̎��̓Z�b�g�ł��Ȃ�
	//! @param [in] pos �J�����̒�������ڕW�̍��W
	inline void SetTargetPos(const designlab::Vector3& pos) { goal_camera_state_.target_pos = pos; }

	//! @brief �J�����̉�]��\���N�H�[�^�j�I�����擾����D
	//! @return �J�����̃N�H�[�^�j�I��
	inline designlab::Quaternion GetCameraRotQuat() const { return goal_camera_state_.camera_rot_quat; }

	//! @brief �J�����̃N�H�[�^�j�I�����Z�b�g����
	//! @param [in] quat �J�����̃N�H�[�^�j�I��
	inline void SetCameraRotQuat(const designlab::Quaternion& quat) { goal_camera_state_.camera_rot_quat = quat; }

	//! @brief �����_�𑀍삷��ۂ́C�J�����̒���������W���Z�b�g����
	//! @param [in] pos �J�����̒���������W
	inline void SetFreeTargetPos(const designlab::Vector3& pos) { free_controlled_target_pos_ = pos; }

	//! @brief �����_�𑀍삷��ۂ́C�J�����̒���������W���擾����
	//! @return designlab::Vector3 �J�����̒���������W
	inline designlab::Vector3 GetFreeTargetPos() const { return free_controlled_target_pos_; }


private:


	struct CameraState final
	{
		CameraState() : camera_rot_quat{}, target_pos{},length_camera_to_target(0) {}

		designlab::Quaternion camera_rot_quat;	//!< �J�����̉�]��\���N�H�[�^�j�I��
		designlab::Vector3 target_pos;			//!< �J�����̒����_
		float length_camera_to_target;	//!< �J�����ƒ����_�Ƃ̋���
	};


	// now_camera_state_�̒l�ŁC�J�����̈ʒu�Ǝp����dxlib�̊֐��ŃZ�b�g����	
	void SetCameraPosAndRot();

	const designlab::Vector3 kDefaultCameraFrontVec;	//!< �f�t�H���g�̃J�����̕�����\���P�ʃx�N�g��

	const designlab::Vector3 kDefaultCameraUpVec;		//!< �f�t�H���g�̃J�����̕�����\���P�ʃx�N�g��


	CameraViewMode camera_view_mode_;	//!< �J�����̎��_�����肷��D

	designlab::Vector3 free_controlled_target_pos_;		//!< �J�����̒����_�����R�ɑ��삷��ۂ̒����_�̍��W

	CameraState now_camera_state_;		//!< ���݂̃J�����̏�Ԃ��܂Ƃ߂��\����

	CameraState goal_camera_state_;		//!< �ڕW�Ƃ���J�����̏�Ԃ��܂Ƃ߂��\����
};


#endif // !DESIGNLAB_CAMERA_MANAGER_H_