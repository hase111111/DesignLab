#pragma once
#include "DxLib.h"
#include "vectorFunc.h"

//Dxlib��3D�̃J�����̏������s���N���X�D
class CameraController final
{
private:
	VECTOR m_target_pos;	//�J��������������ڕW�̍��W�D
	VECTOR m_camera_up;		//�J�����̏������ݒ肷��D
	float m_length_camera_to_target;	//�J�����ƖڕW���Ƃ̋����D
	float m_angle;			//�J�����̉�]�p�x

public:
	CameraController();
	~CameraController() = default;

	//�J�����̈ʒu�Ȃǂ̍X�V���s���D
	void update();

	//�J�����̒�������ڕW�̍��W�D
	void setTargetPos(const VECTOR _pos);

};
