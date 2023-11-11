#include "camera_gui.h"

#include <string>

#include "dxlib_util.h"
#include "mouse.h"


namespace dldu = designlab::dxlib_util;


CameraGui::CameraGui() : CameraGui::CameraGui{ 0, 0, kOptionLeftTop }
{
}

CameraGui::CameraGui(const int left_x_pos, const int top_y_pos, const unsigned int option) :
	kGuiSizeX(235),
	kGuiSizeY(410),
	kGuiSizeYClosed(40),
	kButtonDistance(10),
	kButtonSize(60),
	kGuiLeftPosX(option & 0b01 ? left_x_pos - kGuiSizeX : left_x_pos),
	kGuiTopPosY(option & 0b10 ? top_y_pos - kGuiSizeY : top_y_pos),
	is_closed_{ false }
{
	const int kCloseButtonSizeX = 100;
	const int kCloseButtonSizeY = 30;

	button_list_[ButtonType::kClosed] = std::make_unique<ButtomController>(kGuiLeftPosX + kGuiSizeX - kCloseButtonSizeX / 2 - 10, kGuiTopPosY + kCloseButtonSizeY - 10,
		kCloseButtonSizeX, kCloseButtonSizeY, "�ő�/����");

	const int kButtonRange = kButtonSize + kButtonDistance;
	const int kLeftPosX = kGuiLeftPosX + kButtonRange / 2 + 15;
	const int kTopPosY = kGuiTopPosY + kButtonRange / 2 + kCloseButtonSizeY + 10;

	button_list_[ButtonType::kLenghReset] = std::make_unique<ButtomController>(kLeftPosX, kTopPosY, kButtonSize, kButtonSize, "Reset\nZoom");
	button_list_[ButtonType::kFront] = std::make_unique<ButtomController>(kLeftPosX + kButtonRange, kTopPosY, kButtonSize, kButtonSize, "Front");
	button_list_[ButtonType::kLeft] = std::make_unique<ButtomController>(kLeftPosX, kTopPosY + kButtonRange, kButtonSize, kButtonSize, "Left");
	button_list_[ButtonType::kTop] = std::make_unique<ButtomController>(kLeftPosX + kButtonRange, kTopPosY + kButtonRange, kButtonSize, kButtonSize, "Top");
	button_list_[ButtonType::kRight] = std::make_unique<ButtomController>(kLeftPosX + kButtonRange * 2, kTopPosY + kButtonRange, kButtonSize, kButtonSize, "Right");
	button_list_[ButtonType::kBack] = std::make_unique<ButtomController>(kLeftPosX + kButtonRange, kTopPosY + kButtonRange * 2, kButtonSize, kButtonSize, "Back");
	button_list_[ButtonType::kTargetReset] = std::make_unique<ButtomController>(kLeftPosX + kButtonRange * 2, kTopPosY, kButtonSize, kButtonSize, "Reset\nTarget");
}


void CameraGui::SetHexapodPos(const designlab::Vector3& pos)
{
	camera_manager_.SetTargetPos(pos);
}


void CameraGui::Update()
{
	//�e�{�^���̏���
	for (auto& button : button_list_)
	{
		button.second->Update();

		//�{�^���������ꂽ��(�ŏ����{�^���ȊO)
		if (button.second->IsPushedNow() && !is_closed_)
		{
			//�{�^���̎�ނɂ���ď�����ς���
			switch (button.first)
			{
			case ButtonType::kLenghReset:
				camera_manager_.InitCaneraTargetLength();
				break;

			case ButtonType::kFront:
				camera_manager_.SetCameraViewMode(CameraViewMode::kFrontView);
				break;

			case ButtonType::kLeft:
				camera_manager_.SetCameraViewMode(CameraViewMode::kLeftSideView);
				break;

			case ButtonType::kTop:
				camera_manager_.SetCameraViewMode(CameraViewMode::kTopView);
				break;

			case ButtonType::kRight:
				camera_manager_.SetCameraViewMode(CameraViewMode::kRightSideView);
				break;

			case ButtonType::kBack:
				camera_manager_.SetCameraViewMode(CameraViewMode::kBackView);
				break;

			case ButtonType::kTargetReset:
				camera_manager_.SetCameraViewMode(CameraViewMode::kFreeControlled);
				break;

			}	//switch (button.first)
		}

		//�ŏ����{�^���������ꂽ��
		if (button.second->IsPushedNow() && button.first == ButtonType::kClosed)
		{
			is_closed_ = !is_closed_;
		}
	}


	//�L�[�{�[�h�ɂ��J�����̑���
	camera_controller_.ChangeCameraState(&camera_manager_);

	//�J�����̍X�V
	camera_manager_.Update();
}


void CameraGui::Draw() const
{
	if (is_closed_)
	{
		DrawClosedBackground();
	}
	else
	{
		DrawBackground();
	}


	//�S�Ẵ{�^���̕`��
	for (auto& button : button_list_)
	{
		if (!(is_closed_ && button.first != ButtonType::kClosed))
		{
			button.second->Draw();
		}
	}

	if (!is_closed_)
	{
		DrawString();
	}
}


void CameraGui::DrawBackground() const
{
	const unsigned int kBackColor = GetColor(255, 255, 255);

	SetDrawBlendMode(DX_BLENDMODE_ALPHA, 200);
	DrawBox(kGuiLeftPosX, kGuiTopPosY, kGuiLeftPosX + kGuiSizeX, kGuiTopPosY + kGuiSizeY, kBackColor, TRUE);
	SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);
}


void CameraGui::DrawClosedBackground() const
{
	const unsigned int kBackColor = GetColor(255, 255, 255);

	SetDrawBlendMode(DX_BLENDMODE_ALPHA, 200);
	DrawBox(kGuiLeftPosX, kGuiTopPosY, kGuiLeftPosX + kGuiSizeX, kGuiTopPosY + kGuiSizeYClosed, kBackColor, TRUE);
	SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);
}


void CameraGui::DrawString() const
{
	const unsigned int kStrColor = GetColor(54, 54, 54);
	const unsigned int kStrRedColor = GetColor(255, 128, 128);

	const int kTextYInterval = 20;
	const int kTextYTop = kGuiTopPosY + 250;

	int text_line = 0;

	DrawFormatString(kGuiLeftPosX + 10, kTextYTop + kTextYInterval * (text_line++),
		Mouse::GetIns()->GetWheelRot() == 0 ? kStrColor : kStrRedColor, "�}�E�X�z�C�[����]");

	DrawFormatString(kGuiLeftPosX + 10, kTextYTop + kTextYInterval * (text_line++),
		Mouse::GetIns()->GetWheelRot() == 0 ? kStrColor : kStrRedColor, " �E�Y�[��");

	DrawFormatString(kGuiLeftPosX + 10, kTextYTop + kTextYInterval * (text_line++),
		Mouse::GetIns()->GetPressingCount(MOUSE_INPUT_MIDDLE) == 0 ? kStrColor : kStrRedColor, "�z�C�[���N���b�N���h���b�O");

	DrawFormatString(kGuiLeftPosX + 10, kTextYTop + kTextYInterval * (text_line++),
		Mouse::GetIns()->GetPressingCount(MOUSE_INPUT_MIDDLE) == 0 ? kStrColor : kStrRedColor, " �E�r���[����]");

	DrawFormatString(kGuiLeftPosX + 10, kTextYTop + kTextYInterval * (text_line++),
		Mouse::GetIns()->GetPressingCount(MOUSE_INPUT_LEFT) == 0 ? kStrColor : kStrRedColor, "���N���b�N���h���b�O");

	DrawFormatString(kGuiLeftPosX + 10, kTextYTop + kTextYInterval * (text_line++),
		Mouse::GetIns()->GetPressingCount(MOUSE_INPUT_LEFT) == 0 ? kStrColor : kStrRedColor, " �E��ʂ̒��S�����]");

	DrawFormatString(kGuiLeftPosX + 10, kTextYTop + kTextYInterval * (text_line++),
		Mouse::GetIns()->GetPressingCount(MOUSE_INPUT_RIGHT) == 0 ? kStrColor : kStrRedColor, "�E�N���b�N���h���b�O");

	DrawFormatString(kGuiLeftPosX + 10, kTextYTop + kTextYInterval * (text_line++),
		Mouse::GetIns()->GetPressingCount(MOUSE_INPUT_RIGHT) == 0 ? kStrColor : kStrRedColor, " �E��ʂ̕��s�ړ�");
}
