#pragma once
#include "DxLib.h"
#include "vectorFunc.h"
#include "Define.h"

//�J�����̎��_��\������񋓎q
enum class ECameraMode : int
{
	TopView,
	OverheadView,
	SideView
};


//Dxlib��3D�̃J�����̏������s���N���X�D
class CameraController final
{
private:
	ECameraMode m_mode;			//�J�����̎��_�����肷��D

	VECTOR m_goal_upvec;	//�J�����̏�����̃x�N�g���̖ڕW�l
	VECTOR m_goal_pos;		//�J�����̈ʒu�̖ڕW�l
	VECTOR m_goal_target_pos;		//�J��������������ڕW�̍��W�D

	VECTOR m_now_target_pos;		//�J��������������ڕW�̍��W�D
	VECTOR m_now_camera_upvec;	//�J�����̌��݂̏�����D
	VECTOR m_now_camera_pos;	//�J�����̌��݂̍��W�D
	float m_length_camera_to_target;	//�J�����ƖڕW���Ƃ̋����D

public:

	CameraController();
	~CameraController() = default;

	//�J�����̈ʒu�Ȃǂ̍X�V���s���D
	void update();

	//�J�����̒�������ڕW�̍��W�D
	inline void setTargetPos(const VECTOR _pos) { m_goal_target_pos = _pos; }

	//�J�����̃��[�h���Z�b�g����D
	inline void setCameraMode(const ECameraMode _mode) { m_mode = _mode; }

private:
	const float CAMERA_SLOW_SPEED = 0.02f;
	const float CAMERA_SPEED = 3.0f;
	const float CAMERA_HIGH_SPEED = 25.0f;
	const float CAMERA_HIGH_DIF = 150.0f;
	const float CAMERA_ANGLE = Define::MY_PI / 4;	//�΂߂���̃J�����̊p�x

	void setGoalCameraPos();

	//�J������ڕW�l�ɏ��X�ɋ߂Â���̂Ɏg�p����D�߂�l�͈ړ����VECTOR
	VECTOR approachTargetVECTOR(const VECTOR _current, const VECTOR _target) const;

	//�J������ڕW�l�ɏ��X�ɋ߂Â���̂Ɏg�p����D��̊֐����X�ɂ������߂Â��D
	VECTOR approachSlowlyTargetVECTOR(const VECTOR _current, const VECTOR _target) const;

	//�����̒l�����X�ɖڕW�l�ɋ߂Â���D
	float approachTargetValue(const float _current, const float _target) const;

	//�����̒l�����X�ɖڕW�l�ɋ߂Â���D��̊֐����X�ɂ������߂Â��D
	float approachSlowlyTargetValue(const float _current, const float _target) const;
};
