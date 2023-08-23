#include "camera_controller.h"

#include "mouse.h"
#include "designlab_dxlib.h"


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



	if (Mouse::getIns()->getPushingCountMiddle() > 0)
	{

		//�z�C�[���N���b�N�����Ă�����J��������]������DX��Y�̂ǂ��炩�̈ړ��ʂ��傫��������������

		if (abs(Mouse::getIns()->getDiffPosX()) > abs(Mouse::getIns()->getDiffPosY()))
		{
			//�J�����̉�]���}�E�X�̉��ړ��ʂɍ��킹�ĕύX
			dl_vec::SQuaternion move_quatx{0, 0, 0, 0};

			move_quatx.setRotAngleAndAxis(Mouse::getIns()->getDiffPosX() * kCameraMoveSpeed * -1, { 0,0,1 });

			dl_vec::SQuaternion res = mp_camera_manager->getCameraRotQuat() * move_quatx;

			res = res.normalize();

			mp_camera_manager->setCameraRotQuat(res);
		}
		else
		{
			//�J�����̉�]���}�E�X�̏c�ړ��ʂɍ��킹�ĕύX
			dl_vec::SQuaternion move_quaty{ 0, 0, 0, 0 };

			move_quaty.setRotAngleAndAxis(Mouse::getIns()->getDiffPosY() * kCameraMoveSpeed * -1, { 0,1,0 });

			dl_vec::SQuaternion res = mp_camera_manager->getCameraRotQuat() * move_quaty;

			res = res.normalize();

			mp_camera_manager->setCameraRotQuat(res);
		}

	}
	else if (Mouse::getIns()->getPushingCountLeft() > 0)
	{

		//���N���b�N���Ă�����J�����̃r���[���_�̒��S������]���Ƃ�����]

		dl_vec::SQuaternion move_quat{ 0, 0, 0, 0 };

		int mouse_move = (abs(Mouse::getIns()->getDiffPosX()) > abs(Mouse::getIns()->getDiffPosY())) ? Mouse::getIns()->getDiffPosX() : Mouse::getIns()->getDiffPosY();

		move_quat.setRotAngleAndAxis(mouse_move * kCameraMoveSpeed * -1.0f, { 1,0,0 });

		dl_vec::SQuaternion res = mp_camera_manager->getCameraRotQuat() * move_quat;

		res = res.normalize();

		mp_camera_manager->setCameraRotQuat(res);

	}
	else if (Mouse::getIns()->getPushingCountRight() > 0 && Mouse::getIns()->getDiffPos() > kMouseMoveMargin)
	{

		//�E�N���b�N���Ă�����J�����̕��s�ړ�

		if (mp_camera_manager->getCameraViewMode() != ECameraMode::FREE_CONTROLLED_TARGET)
		{
			//�\�����[�h�𒍎��_�����R�ɓ������郂�[�h�ɕύX
			mp_camera_manager->setCameraViewMode(ECameraMode::FREE_CONTROLLED_TARGET);
		}


		dl_vec::SVector move_vec;

		if (abs(Mouse::getIns()->getDiffPosX()) > abs(Mouse::getIns()->getDiffPosY()))
		{
			//X�̈ړ��ʂ��傫���ꍇ�͉��ړ��ʂ��ړ��ʂƂ���

			move_vec = { 0, Mouse::getIns()->getDiffPosX() * kCameraTargetMoveSpeed * -1, 0 };

			move_vec = dl_vec::rotVecByQuat(move_vec, mp_camera_manager->getCameraRotQuat());
		}
		else
		{
			//Y�̈ړ��ʂ��傫���ꍇ�͏c�ړ��ʂ��ړ��ʂƂ���

			move_vec = { 0, 0, Mouse::getIns()->getDiffPosY() * kCameraTargetMoveSpeed };

			move_vec = dl_vec::rotVecByQuat(move_vec, mp_camera_manager->getCameraRotQuat());
		}


		VECTOR now_target_pos = mp_camera_manager->getFreeTargetPos();				//���݂̃^�[�Q�b�g���W���擾

		now_target_pos = VAdd(now_target_pos, dl_dxlib::convertToDxVec(move_vec));	//�ړ��ʂ����Z

		mp_camera_manager->setFreeTargetPos(now_target_pos);						//�^�[�Q�b�g���W���X�V	

	}
}
