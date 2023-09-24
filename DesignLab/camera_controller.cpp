#include "camera_controller.h"

#include "mouse.h"
#include "dxlib_util.h"

namespace dl = designlab;
namespace dldu = designlab::dxlib_util;


CameraController::CameraController(CameraStateManager& camera_manager_ref) : 
	camera_manager_ref_(camera_manager_ref) 
{
}


void CameraController::Update()
{
	//�z�C�[���������Ă�����J����������ύX����
	if (Mouse::GetIns()->GetWheelRot() != 0)
	{
		camera_manager_ref_.addCameraToTargetLength(kCameraZoomSpeed * Mouse::GetIns()->GetWheelRot() * -1);
	}


	if (Mouse::GetIns()->GetPressingCount(MOUSE_INPUT_MIDDLE) > 0)
	{
		//�z�C�[���N���b�N�����Ă�����J��������]������DX��Y�̂ǂ��炩�̈ړ��ʂ��傫��������������

		if (abs(Mouse::GetIns()->GetDiffPosX()) > abs(Mouse::GetIns()->GetDiffPosY()))
		{
			//�J�����̉�]���}�E�X�̉��ړ��ʂɍ��킹�ĕύX
			dl::Quaternion move_quatx = {0, 0, 0, 0};

			move_quatx = dl::Quaternion::MakeByRotAngleAndAxis(Mouse::GetIns()->GetDiffPosX() * kCameraMoveSpeed * -1, { 0,0,1 });

			dl::Quaternion res = camera_manager_ref_.getCameraRotQuat() * move_quatx;

			res = res.Normalize();

			camera_manager_ref_.setCameraRotQuat(res);
		}
		else
		{
			//�J�����̉�]���}�E�X�̏c�ړ��ʂɍ��킹�ĕύX
			dl::Quaternion move_quaty{ 0, 0, 0, 0 };

			move_quaty = dl::Quaternion::MakeByRotAngleAndAxis(Mouse::GetIns()->GetDiffPosY() * kCameraMoveSpeed * -1, { 0,1,0 });

			dl::Quaternion res = camera_manager_ref_.getCameraRotQuat() * move_quaty;

			res = res.Normalize();

			camera_manager_ref_.setCameraRotQuat(res);
		}

	}
	else if (Mouse::GetIns()->GetPressingCount(MOUSE_INPUT_LEFT) > 0)
	{

		//���N���b�N���Ă�����J�����̃r���[���_�̒��S������]���Ƃ�����]

		dl::Quaternion move_quat = { 0, 0, 0, 0 };

		int mouse_move = (abs(Mouse::GetIns()->GetDiffPosX()) > abs(Mouse::GetIns()->GetDiffPosY())) ? Mouse::GetIns()->GetDiffPosX() : Mouse::GetIns()->GetDiffPosY();

		move_quat = dl::Quaternion::MakeByRotAngleAndAxis(mouse_move * kCameraMoveSpeed * -1.0f, { 1,0,0 });

		dl::Quaternion res = camera_manager_ref_.getCameraRotQuat() * move_quat;

		res = res.Normalize();

		camera_manager_ref_.setCameraRotQuat(res);

	}
	else if (Mouse::GetIns()->GetPressingCount(MOUSE_INPUT_RIGHT) > 0 && Mouse::GetIns()->GetDiffPos() > kMouseMoveMargin)
	{

		//�E�N���b�N���Ă�����J�����̕��s�ړ�

		if (camera_manager_ref_.getCameraViewMode() != CameraViewMode::FREE_CONTROLLED_TARGET)
		{
			//�\�����[�h�𒍎��_�����R�ɓ������郂�[�h�ɕύX
			camera_manager_ref_.setCameraViewMode(CameraViewMode::FREE_CONTROLLED_TARGET);
		}


		dl::Vector3 move_vec;

		if (abs(Mouse::GetIns()->GetDiffPosX()) > abs(Mouse::GetIns()->GetDiffPosY()))
		{
			//X�̈ړ��ʂ��傫���ꍇ�͉��ړ��ʂ��ړ��ʂƂ���

			move_vec = { 0, Mouse::GetIns()->GetDiffPosX() * kCameraTargetMoveSpeed * -1, 0 };

			move_vec = dl::rotVecByQuat(move_vec, camera_manager_ref_.getCameraRotQuat());
		}
		else
		{
			//Y�̈ړ��ʂ��傫���ꍇ�͏c�ړ��ʂ��ړ��ʂƂ���

			move_vec = { 0, 0, Mouse::GetIns()->GetDiffPosY() * kCameraTargetMoveSpeed };

			move_vec = dl::rotVecByQuat(move_vec, camera_manager_ref_.getCameraRotQuat());
		}

		VECTOR now_target_pos = camera_manager_ref_.getFreeTargetPos();				//���݂̃^�[�Q�b�g���W���擾

		now_target_pos = VAdd(now_target_pos, dldu::ConvertToDxlibVec(move_vec));	//�ړ��ʂ����Z

		camera_manager_ref_.setFreeTargetPos(now_target_pos);						//�^�[�Q�b�g���W���X�V	

	}
}
