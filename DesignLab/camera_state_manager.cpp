#include "camera_state_manager.h"

#include <cmath>

#include <Dxlib.h>

#include "cassert_define.h"
#include "designlab_math_util.h"
#include "dxlib_util.h"
#include "graphic_const.h"


namespace dl = designlab;
namespace dlm = designlab::math_util;
namespace dldu = designlab::dxlib_util;


CameraStateManager::CameraStateManager() :
	kDefaultCameraFrontVec{ dl::Vector3::GetFrontVec() },
	kDefaultCameraUpVec{ dl::Vector3::GetUpVec() },
	camera_view_mode_(CameraViewMode::kTopView),
	free_controlled_target_pos_{},
	now_camera_state_{},
	goal_camera_state_{}
{
	SetCameraViewMode(CameraViewMode::kTopView);	//�J�����̏����ʒu���Z�b�g����D

	SetCameraPosAndRot();		//�J�����ʒu���Z�b�g����D

	InitCaneraTargetLength();	//�J�����̋���������������D
}


void CameraStateManager::Update()
{
	//�J�����̋�����ڕW�l�ɋ߂Â���D
	now_camera_state_.length_camera_to_target = dlm::ApproachTarget(
		now_camera_state_.length_camera_to_target, 
		goal_camera_state_.length_camera_to_target,
		0.2f
	);	

	//�J�����̉�]��ڕW�l�ɋ߂Â���D
	now_camera_state_.camera_rot_quat = dlm::ApproachTarget(
		now_camera_state_.camera_rot_quat, 
		goal_camera_state_.camera_rot_quat,
		0.2f
	);

	now_camera_state_.camera_rot_quat = now_camera_state_.camera_rot_quat.Normalize();


	//�J�����̒����_��ڕW�l�ɋ߂Â���D
	if (camera_view_mode_ != CameraViewMode::kFreeControlledAndMovableTarget)
	{
		now_camera_state_.target_pos = dlm::ApproachTarget(
			now_camera_state_.target_pos, 
			goal_camera_state_.target_pos,
			0.2f
		);
	}
	else
	{
		now_camera_state_.target_pos = dlm::ApproachTarget(
			now_camera_state_.target_pos, 
			free_controlled_target_pos_,
			0.2f
		);
	}


	//�J�����ʒu���Z�b�g����D
	SetCameraPosAndRot();
}


void CameraStateManager::SetCameraViewMode(const CameraViewMode mode)
{
	camera_view_mode_ = mode;

	// �S�[�����W���X�V����
	switch (mode)
	{
	case CameraViewMode::kFrontView:
	{
		goal_camera_state_.camera_rot_quat =
			dl::Quaternion::MakeByAngleAxis(dlm::ConvertDegToRad(0.0f), dl::Vector3::GetUpVec());
		return;
	}

	case CameraViewMode::kBackView:
	{
		goal_camera_state_.camera_rot_quat =
			dl::Quaternion::MakeByAngleAxis(dlm::ConvertDegToRad(180.0f), dl::Vector3::GetUpVec());
		return;
	}

	case CameraViewMode::kTopView:
	{
		dl::Quaternion quat1 = dl::Quaternion::MakeByAngleAxis(dlm::ConvertDegToRad(-90.0f), dl::Vector3::GetLeftVec());
		dl::Quaternion quat2 = dl::Quaternion::MakeByAngleAxis(dlm::ConvertDegToRad(180.0f), dl::Vector3::GetFrontVec());
		goal_camera_state_.camera_rot_quat = quat1 * quat2;
		return;
	}

	case CameraViewMode::kRightSideView:
	{
		goal_camera_state_.camera_rot_quat =
			dl::Quaternion::MakeByAngleAxis(dlm::ConvertDegToRad(270.0f), dl::Vector3::GetUpVec());
		return;
	}

	case CameraViewMode::kLeftSideView:
	{
		goal_camera_state_.camera_rot_quat =
			dl::Quaternion::MakeByAngleAxis(dlm::ConvertDegToRad(90.0f), dl::Vector3::GetUpVec());
		return;
	}

	case CameraViewMode::kFreeControlled:
	{
		return;
	}

	case CameraViewMode::kFreeControlledAndMovableTarget:
	{
		free_controlled_target_pos_ = goal_camera_state_.target_pos;
		return;
	}

	default:
		assert(false);	//�����ɗ��邱�Ƃ͂Ȃ��D
		break;
	}
}


void CameraStateManager::InitCaneraTargetLength()
{
	//�ő�ƍŏ��̒��Ԓl�������l�Ƃ���D
	goal_camera_state_.length_camera_to_target = (GraphicConst::kCameraToTargetMin + GraphicConst::kCameraToTargetMax) * 0.5f;
}


void CameraStateManager::AddCameraToTargetLength(const float length_dif)
{
	goal_camera_state_.length_camera_to_target += length_dif;

	// �J�����̋������ő�l�𒴂�����ő�l�ɂ���D
	if (GraphicConst::kCameraToTargetMax < goal_camera_state_.length_camera_to_target)
	{
		goal_camera_state_.length_camera_to_target = GraphicConst::kCameraToTargetMax;
	}

	// �J�����̋������ŏ��l�����������ŏ��l�ɂ���D
	if (goal_camera_state_.length_camera_to_target < GraphicConst::kCameraToTargetMin)
	{
		goal_camera_state_.length_camera_to_target = GraphicConst::kCameraToTargetMin;
	}
}


void CameraStateManager::SetCameraPosAndRot()
{
	//�J�����̈ʒu���Z�b�g����D�N�H�[�^�j�I����p���ĉ�]�����Cdl_vec::vector����dxlib::VECTOR�ɕϊ�����D

	dl::Vector3 camera_target_dif = dl::RotateVector3(kDefaultCameraFrontVec, now_camera_state_.camera_rot_quat, true) * now_camera_state_.length_camera_to_target;
	VECTOR camera_pos = dldu::ConvertToDxlibVec(camera_target_dif + now_camera_state_.target_pos);

	VECTOR camera_upvec = dldu::ConvertToDxlibVec(dl::RotateVector3(kDefaultCameraUpVec, now_camera_state_.camera_rot_quat, true));

	SetCameraPositionAndTargetAndUpVec(
		camera_pos, 
		dldu::ConvertToDxlibVec(now_camera_state_.target_pos), 
		camera_upvec
	);
}