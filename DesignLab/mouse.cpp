#include "mouse.h"

#include <cmath>

#include "DxLib.h"

#include "designlab_math.h"


Mouse::Mouse()
{
	//�ϐ�������������
	cursor_pos_x_ = 0;
	cursor_pos_y_ = 0;
	cursor_past_pos_x_ = 0;
	cursor_past_pos_y_ = 0;
	left_pushing_counter_ = 0;
	middle_pushing_counter_ = 0;
	right_pushing_counter_ = 0;
	left_releasing_counter_ = 0;
	middle_releasing_counter_ = 0;
	right_releasing_counter_ = 0;
	wheel_rot_ = 0;
}


void Mouse::Update()
{
	//�}�E�X�̈ʒu�擾
	cursor_past_pos_x_ = cursor_pos_x_;
	cursor_past_pos_y_ = cursor_pos_y_;
	GetMousePoint(&cursor_pos_x_, &cursor_pos_y_);

	//���N���b�N
	if ((GetMouseInput() & MOUSE_INPUT_LEFT) != 0)
	{
		//������Ă���Ȃ�
		left_pushing_counter_++;
		left_releasing_counter_ = 0;
	}
	else
	{
		//������Ă���Ȃ�
		left_pushing_counter_ = 0;
		left_releasing_counter_++;
	}

	//�E�N���b�N
	if ((GetMouseInput() & MOUSE_INPUT_RIGHT) != 0)
	{
		//������Ă���Ȃ�
		right_pushing_counter_++;
		right_releasing_counter_ = 0;
	}
	else
	{
		//������Ă���Ȃ�
		right_pushing_counter_ = 0;
		right_releasing_counter_++;
	}

	//�z�[���h�{�^��
	if ((GetMouseInput() & MOUSE_INPUT_MIDDLE) != 0)
	{
		//������Ă���Ȃ�
		middle_pushing_counter_++;
		middle_releasing_counter_ = 0;
	}
	else
	{
		//������Ă���Ȃ�
		middle_pushing_counter_ = 0;
		middle_releasing_counter_++;
	}

	//�z�C�[����]
	wheel_rot_ = GetMouseWheelRotVol();
}


int Mouse::GetDiffPosX() const
{
	return cursor_pos_x_ - cursor_past_pos_x_;
}


int Mouse::GetDiffPosY() const
{
	return cursor_pos_y_ - cursor_past_pos_y_;
}


double Mouse::getDiffPos() const
{
	return sqrt(static_cast<double>(dl_math::squared(GetDiffPosY()) + dl_math::squared(GetDiffPosX())));
}