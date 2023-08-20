#include "camera_controller.h"

#include "mouse.h"


CameraController::CameraController(CameraManager* p_camera_manager) : mp_camera_manager(p_camera_manager) {}


void CameraController::update()
{
	//�������^�[���D�|�C���^��nullptr�Ȃ牽�����Ȃ�
	if (mp_camera_manager == nullptr) { return; }


	//�z�C�[���������Ă�����J����������ύX����
	if (Mouse::getIns()->getWheelRot() != 0)
	{
		mp_camera_manager->addCameraToTargetLength(kCameraZoomSpeed * Mouse::getIns()->getWheelRot() * -1);
	}


	//�z�C�[���N���b�N�����Ă�����J��������]������
	if (Mouse::getIns()->getPushingCountMiddle() > 0)
	{
		if (abs(Mouse::getIns()->getDiffPosX()) > abs(Mouse::getIns()->getDiffPosY()))
		{
			//�J�����̉�]���}�E�X�̉��ړ��ʂɍ��킹�ĕύX
			dl_vec::SQuaternion move_quatx{0, 0, 0, 0};

			move_quatx.setRotAngleAndAxis(Mouse::getIns()->getDiffPosX() * kCameraMoveSpeed * -1, { 0,0,1 });

			dl_vec::SQuaternion res = mp_camera_manager->getCameraRotQuat() * move_quatx;

			mp_camera_manager->setCameraRotQuat(res);
		}
		else
		{
			//�J�����̉�]���}�E�X�̏c�ړ��ʂɍ��킹�ĕύX
			dl_vec::SQuaternion move_quaty{ 0, 0, 0, 0 };

			move_quaty.setRotAngleAndAxis(Mouse::getIns()->getDiffPosY() * kCameraMoveSpeed * -1, { 0,1,0 });

			dl_vec::SQuaternion res = mp_camera_manager->getCameraRotQuat() * move_quaty;

			mp_camera_manager->setCameraRotQuat(res);
		}
	}


	//�J�����̃r���[���_�̒��S������]���Ƃ�����]
	if (Mouse::getIns()->getPushingCountRight() > 0)
	{
		dl_vec::SQuaternion move_quat{ 0, 0, 0, 0 };

		int mouse_move = (abs(Mouse::getIns()->getDiffPosX()) > abs(Mouse::getIns()->getDiffPosY())) ? Mouse::getIns()->getDiffPosX() : Mouse::getIns()->getDiffPosY();

		move_quat.setRotAngleAndAxis(mouse_move * kCameraMoveSpeed * -1.0f, { 1,0,0 });

		dl_vec::SQuaternion res = mp_camera_manager->getCameraRotQuat() * move_quat;

		mp_camera_manager->setCameraRotQuat(res);
	}
}
