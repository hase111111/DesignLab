#include "camera_manager.h"

#include <cmath>

#include <Dxlib.h>

#include "designlab_math_util.h"
#include "dxlib_util.h"
#include "graphic_const.h"


namespace dl = designlab;
namespace dlm = designlab::math_util;
namespace dldu = designlab::dxlib_util;


CameraStateManager::CameraStateManager() :
	m_camera_view_mode(CameraViewMode::kTopView),
	m_goal_target_pos(VGet(0, 0, 0)), 
	m_goal_length_camera_to_target(GraphicConst::kCameraToTargetMax),
	m_target_pos(VGet(0, 0, 0)), 
	m_length_camera_to_target(GraphicConst::kCameraToTargetMax)
{
	setCameraViewMode(CameraViewMode::kTopView);	//�J�����̏����ʒu���Z�b�g����D

	setCameraPosAndRot();		//�J�����ʒu���Z�b�g����D

	initCaneraTargetLength();	//�J�����̋���������������D
}


void CameraStateManager::Update()
{
	m_length_camera_to_target = approachTargetValue(m_length_camera_to_target, m_goal_length_camera_to_target);	//�J�����̋�����ڕW�l�ɋ߂Â���D
	m_camera_rot_quat = approachTargetQuat(m_camera_rot_quat, m_goal_camera_rot_quat);							//�J�����̉�]��ڕW�l�ɋ߂Â���D

	//�J�����̒����_��ڕW�l�ɋ߂Â���D
	if (m_camera_view_mode != CameraViewMode::FREE_CONTROLLED_TARGET)
	{
		m_target_pos = approachTargetVECTOR(m_target_pos, m_goal_target_pos);
	}
	else
	{
		m_target_pos = approachTargetVECTOR(m_target_pos, m_free_controlled_target);
	}


	//�J�����ʒu���Z�b�g����D
	setCameraPosAndRot();
}


void CameraStateManager::setCameraViewMode(const CameraViewMode mode)
{
	m_camera_view_mode = mode;	//�ꉞ�J�����̃��[�h�������o�Ŏ����Ă��邪�C���̂Ƃ���g���Ă��Ȃ��D�K�v�Ȃ�����

	dl::Quaternion quat1{0, 0, 0, 0}, quat2{ 0,0,0,0 };	//switch���̒��Ő錾����ƃG���[���o��̂ŁC�O���錾����D


	switch (mode)
	{
	case CameraViewMode::kFrontView:
		m_goal_camera_rot_quat = dl::Quaternion::MakeByRotAngleAndAxis(dlm::ConvertDegToRad(0.0f), dl::Vector3::GetUpVec());
		break;

	case CameraViewMode::kBackView:
		m_goal_camera_rot_quat = dl::Quaternion::MakeByRotAngleAndAxis(dlm::ConvertDegToRad(180.0f), dl::Vector3::GetUpVec());
		break;

	case CameraViewMode::kTopView:
		quat1 = dl::Quaternion::MakeByRotAngleAndAxis(dlm::ConvertDegToRad(-90.0f), dl::Vector3::GetLeftVec());
		quat2 = dl::Quaternion::MakeByRotAngleAndAxis(dlm::ConvertDegToRad(180.0f), dl::Vector3::GetFrontVec());
		m_goal_camera_rot_quat = (quat1 * quat2).Normalize();
		break;

	case CameraViewMode::kRightSideView:
		m_goal_camera_rot_quat = dl::Quaternion::MakeByRotAngleAndAxis(dlm::ConvertDegToRad(270.0f), dl::Vector3::GetUpVec());
		break;

	case CameraViewMode::kLeftSideView:
		m_goal_camera_rot_quat = dl::Quaternion::MakeByRotAngleAndAxis(dlm::ConvertDegToRad(90.0f), dl::Vector3::GetUpVec());
		break;

	case CameraViewMode::FREE_CONTROLLED_TARGET:
		m_free_controlled_target = m_goal_target_pos;
		break;

	default:
		break;
	}
}


void CameraStateManager::initCaneraTargetLength()
{
	//�ő�ƍŏ��̒��Ԓl�������l�Ƃ���D
	m_goal_length_camera_to_target = (GraphicConst::kCameraToTargetMin + GraphicConst::kCameraToTargetMax) * 0.5f;
}


void CameraStateManager::addCameraToTargetLength(const float length_dif)
{
	m_goal_length_camera_to_target += length_dif;

	if (GraphicConst::kCameraToTargetMax < m_goal_length_camera_to_target) { m_goal_length_camera_to_target = GraphicConst::kCameraToTargetMax; }

	if (m_goal_length_camera_to_target < GraphicConst::kCameraToTargetMin) { m_goal_length_camera_to_target = GraphicConst::kCameraToTargetMin; }
}


void CameraStateManager::setCameraPosAndRot()
{
	//�J�����̈ʒu���Z�b�g����D�N�H�[�^�j�I����p���ĉ�]�����Cdl_vec::vector����dxlib::VECTOR�ɕϊ�����D

	VECTOR camera_target_dif = dldu::ConvertToDxlibVec(dl::rotVecByQuat(kDefaultCameraFrontVec, m_camera_rot_quat) * m_length_camera_to_target);
	VECTOR camera_pos = VAdd(camera_target_dif, m_target_pos);

	VECTOR camera_upvec = dldu::ConvertToDxlibVec(dl::rotVecByQuat(kDefaultCameraUpVec, m_camera_rot_quat));

	SetCameraPositionAndTargetAndUpVec(camera_pos, m_target_pos, camera_upvec);
}


designlab::Quaternion CameraStateManager::approachTargetQuat(const designlab::Quaternion& current, const designlab::Quaternion& target) const
{
	const float dif = 0.2f;
	return ((1 - dif) * current + dif * target).Normalize();
}


VECTOR CameraStateManager::approachTargetVECTOR(const VECTOR& current, const VECTOR& target) const
{
	return VGet(approachTargetValue(current.x, target.x), approachTargetValue(current.y, target.y), approachTargetValue(current.z, target.z));
}


float CameraStateManager::approachTargetValue(const float current, const float target) const
{
	const float dif = 0.2f;
	return (1 - dif) * current + dif * target;
}
