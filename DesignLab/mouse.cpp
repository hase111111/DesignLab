#include "mouse.h"

#include <cmath>

#include <Dxlib.h>

#include "designlab_math_util.h"

namespace dlm = designlab::math_util;


Mouse::Mouse() : 
	kMouseKeyCodes
	{
		MOUSE_INPUT_RIGHT,
		MOUSE_INPUT_LEFT,
		MOUSE_INPUT_MIDDLE,
		MOUSE_INPUT_4,
		MOUSE_INPUT_5,
		MOUSE_INPUT_6,
		MOUSE_INPUT_7,
		MOUSE_INPUT_8
	},
	cursor_pos_x_(0),
	cursor_pos_y_(0),
	cursor_past_pos_x_(0),
	cursor_past_pos_y_(0),
	pushing_counter_({}),
	releasing_counter_({}),
	wheel_rot_(0)
{
}

void Mouse::Update()
{
	// �}�E�X�̈ʒu�擾
	cursor_past_pos_x_ = cursor_pos_x_;
	cursor_past_pos_y_ = cursor_pos_y_;
	GetMousePoint(&cursor_pos_x_, &cursor_pos_y_);

	// �}�E�X�̃N���b�N�擾
	const int mouse_input = GetMouseInput();

	for (const auto& i : kMouseKeyCodes)
	{
		if (mouse_input & i)
		{
			// ������Ă���Ȃ�
			pushing_counter_[i]++;
			releasing_counter_[i] = 0;
		}
		else
		{
			// ������Ă���Ȃ�
			pushing_counter_[i] = 0;
			releasing_counter_[i]++;
		}
	}

	// �z�C�[����]���擾�CGetMouseWheelRotVol()�͑O��̌Ăяo���ȍ~�̉�]�ʂ�Ԃ��D
	wheel_rot_ = GetMouseWheelRotVol();
}

int Mouse::GetPressingCount(const int mouse_code) const
{
	// std::map�ł�find��count���������x�͓������炢�Cmultimap��multiset��find�𐄏�
	if (releasing_counter_.count(mouse_code) == 0)
	{
		return -1;
	}

	return pushing_counter_.at(mouse_code);
}

int Mouse::GetReleasingCount(const int mouse_code) const
{
	if (releasing_counter_.count(mouse_code) == 0)
	{
		return -1;
	}

	return releasing_counter_.at(mouse_code);
}

int Mouse::GetDiffPosX() const
{
	return cursor_pos_x_ - cursor_past_pos_x_;
}

int Mouse::GetDiffPosY() const
{
	return cursor_pos_y_ - cursor_past_pos_y_;
}

double Mouse::GetDiffPos() const
{
	return sqrt
	(
		static_cast<double>
		(
			dlm::Squared(GetDiffPosY()) + dlm::Squared(GetDiffPosX())
		)
	);
}