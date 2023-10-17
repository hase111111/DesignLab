#include "camera_input_controller.h"

#include "cassert_define.h"
#include "mouse.h"
#include "dxlib_util.h"


namespace dl = designlab;
namespace dldu = designlab::dxlib_util;


CameraInputController::CameraInputController() : 
	kCameraZoomSpeed(50.0f),
	kCameraMoveSpeed(0.007f),
	kCameraTargetMoveSpeed(3.0f),
	kMouseMoveMargin(0.5)
{
}


void CameraInputController::ChangeCameraState(CameraStateManager* camera_manager)
{
	assert(camera_manager != nullptr);

	//�z�C�[���������Ă�����J����������ύX����
	if (Mouse::GetIns()->GetWheelRot() != 0)
	{
		camera_manager->AddCameraToTargetLength(kCameraZoomSpeed * Mouse::GetIns()->GetWheelRot() * -1);
	}

	// �J�[�\���������Ă�����C�J�����̉�]��ύX����
	if (Mouse::GetIns()->GetPressingCount(MOUSE_INPUT_MIDDLE) > 0)
	{
		//�z�C�[���N���b�N�����Ă�����J��������]������DX��Y�̂ǂ��炩�̈ړ��ʂ��傫��������������

		if (abs(Mouse::GetIns()->GetDiffPosX()) > 0)
		{
			//�J�����̉�]���}�E�X�̉��ړ��ʂɍ��킹�ĕύX
			dl::Quaternion move_quatx = {0, 0, 0, 0};

			move_quatx = dl::Quaternion::MakeByAngleAxis(Mouse::GetIns()->GetDiffPosX() * kCameraMoveSpeed * -1, { 0,0,1 });

			dl::Quaternion res = camera_manager->GetCameraQuat() * move_quatx;

			res = res.GetNormalized();

			camera_manager->SetCameraQuat(res);
		}

		if (abs(Mouse::GetIns()->GetDiffPosY()) > 0)
		{
			//�J�����̉�]���}�E�X�̏c�ړ��ʂɍ��킹�ĕύX
			dl::Quaternion move_quaty = dl::Quaternion::MakeByAngleAxis(Mouse::GetIns()->GetDiffPosY() * kCameraMoveSpeed * -1, { 0,1,0 });

			dl::Quaternion res = camera_manager->GetCameraQuat() * move_quaty;

			res = res.GetNormalized();

			camera_manager->SetCameraQuat(res);
		}

	}
	else if (Mouse::GetIns()->GetPressingCount(MOUSE_INPUT_LEFT) > 0)
	{
		//���N���b�N���Ă�����J�����̃r���[���_�̒��S������]���Ƃ�����]

		int mouse_move = (abs(Mouse::GetIns()->GetDiffPosX()) > abs(Mouse::GetIns()->GetDiffPosY())) ? Mouse::GetIns()->GetDiffPosX() : Mouse::GetIns()->GetDiffPosY();

		dl::Quaternion move_quat = dl::Quaternion::MakeByAngleAxis(mouse_move * kCameraMoveSpeed * -1.0f, { 1,0,0 });

		dl::Quaternion res = camera_manager->GetCameraQuat() * move_quat;

		res = res.GetNormalized();

		camera_manager->SetCameraQuat(res);

	}
	else if (Mouse::GetIns()->GetPressingCount(MOUSE_INPUT_RIGHT) > 0 && Mouse::GetIns()->GetDiffPos() > kMouseMoveMargin)
	{

		//�E�N���b�N���Ă�����J�����̕��s�ړ�

		if (camera_manager->GetCameraViewMode() != CameraViewMode::kFreeControlledAndMovableTarget)
		{
			//�\�����[�h�𒍎��_�����R�ɓ������郂�[�h�ɕύX
			camera_manager->SetCameraViewMode(CameraViewMode::kFreeControlledAndMovableTarget);
		}


		dl::Vector3 move_vec_x;

		if (abs(Mouse::GetIns()->GetDiffPosX()) > 0)
		{
			//X�̈ړ��ʂ��傫���ꍇ�͉��ړ��ʂ��ړ��ʂƂ���

			move_vec_x = { 0, Mouse::GetIns()->GetDiffPosX() * kCameraTargetMoveSpeed * -1, 0 };

			move_vec_x = dl::RotateVector3(move_vec_x, camera_manager->GetCameraQuat(), true);
		}

		dl::Vector3 move_vec_y;

		if (abs(Mouse::GetIns()->GetDiffPosY()) > 0)
		{
			//Y�̈ړ��ʂ��傫���ꍇ�͏c�ړ��ʂ��ړ��ʂƂ���

			move_vec_y = { 0, 0, Mouse::GetIns()->GetDiffPosY() * kCameraTargetMoveSpeed };

			move_vec_y = dl::RotateVector3(move_vec_y, camera_manager->GetCameraQuat(), true);
		}

		dl::Vector3 now_target_pos = camera_manager->GetFreeTargetPos();	//���݂̃^�[�Q�b�g���W���擾

		now_target_pos = now_target_pos + move_vec_x + move_vec_y;			//�ړ��ʂ����Z

		camera_manager->SetFreeTargetPos(now_target_pos);					//�^�[�Q�b�g���W���X�V	
	}
}