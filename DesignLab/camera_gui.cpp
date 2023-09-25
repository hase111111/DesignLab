#include "camera_gui.h"

#include <string>

#include "dxlib_util.h"
#include "mouse.h"


namespace dldu = designlab::dxlib_util;


CameraGui::CameraGui(const int left_x, const int top_y) :
	kGuiLeftPosX(left_x),
	kGuiTopPosY(top_y),
	m_camera_controller(m_camera_manager)
{
	const int kCloseButtonSizeX = 100;
	const int kCloseButtonSizeY = 30;

	m_buttons[EButtonType::CLOSED] = std::make_unique<ButtomController>(kGuiLeftPosX + GUI_SIZE_X - kCloseButtonSizeX / 2 - 10, kGuiTopPosY + kCloseButtonSizeY - 10,
		kCloseButtonSizeX, kCloseButtonSizeY, "�ő�/����");

	const int kButtonRange = kButtonSize + kButtonDistance;
	const int kLeftPosX = kGuiLeftPosX + kButtonRange / 2 + 15;
	const int kTopPosY = kGuiTopPosY + kButtonRange / 2 + kCloseButtonSizeY + 10;

	m_buttons[EButtonType::LENGH_RESET] = std::make_unique<ButtomController>(kLeftPosX, kTopPosY, kButtonSize, kButtonSize, "Reset\nZoom");
	m_buttons[EButtonType::FRONT] = std::make_unique<ButtomController>(kLeftPosX + kButtonRange, kTopPosY, kButtonSize, kButtonSize, "Front");
	m_buttons[EButtonType::LEFT] = std::make_unique<ButtomController>(kLeftPosX, kTopPosY + kButtonRange, kButtonSize, kButtonSize, "Left");
	m_buttons[EButtonType::TOP] = std::make_unique<ButtomController>(kLeftPosX + kButtonRange, kTopPosY + kButtonRange, kButtonSize, kButtonSize, "Top");
	m_buttons[EButtonType::RIGHT] = std::make_unique<ButtomController>(kLeftPosX + kButtonRange * 2, kTopPosY + kButtonRange, kButtonSize, kButtonSize, "Right");
	m_buttons[EButtonType::BACK] = std::make_unique<ButtomController>(kLeftPosX + kButtonRange, kTopPosY + kButtonRange * 2, kButtonSize, kButtonSize, "Back");
	m_buttons[EButtonType::TARGET_RESET] = std::make_unique<ButtomController>(kLeftPosX + kButtonRange * 2, kTopPosY, kButtonSize, kButtonSize, "Reset\nTarget");

}


CameraGui::CameraGui() : CameraGui::CameraGui(10, 10)
{
}


void CameraGui::setHexapodPos(const designlab::Vector3 pos)
{
	m_camera_manager.SetTargetPos(pos);
}


void CameraGui::Update()
{

	//�e�{�^���̏���
	for (auto& button : m_buttons)
	{
		button.second->Update();

		//�{�^���������ꂽ��(�ŏ����{�^���ȊO)
		if (button.second->isPushedNow() && !m_is_closed)
		{
			//�{�^���̎�ނɂ���ď�����ς���
			switch (button.first)
			{
			case EButtonType::LENGH_RESET:
				m_camera_manager.InitCaneraTargetLength();
				break;

			case EButtonType::FRONT:
				m_camera_manager.SetCameraViewMode(CameraViewMode::kFrontView);
				break;

			case EButtonType::LEFT:
				m_camera_manager.SetCameraViewMode(CameraViewMode::kLeftSideView);
				break;

			case EButtonType::TOP:
				m_camera_manager.SetCameraViewMode(CameraViewMode::kTopView);
				break;

			case EButtonType::RIGHT:
				m_camera_manager.SetCameraViewMode(CameraViewMode::kRightSideView);
				break;

			case EButtonType::BACK:
				m_camera_manager.SetCameraViewMode(CameraViewMode::kBackView);
				break;

			case EButtonType::TARGET_RESET:
				m_camera_manager.SetCameraViewMode(CameraViewMode::kFreeControlled);
				break;

			}	//switch (button.first)
		}

		//�ŏ����{�^���������ꂽ��
		if (button.second->isPushedNow() && button.first == EButtonType::CLOSED)
		{
			m_is_closed = !m_is_closed;
		}
	}


	//�L�[�{�[�h�ɂ��J�����̑���
	m_camera_controller.Update();

	//�J�����̍X�V
	m_camera_manager.Update();
}


void CameraGui::Draw() const
{
	if (m_is_closed)
	{
		drawClosedBackground();
	}
	else
	{
		DrawBackground();
	}


	//�S�Ẵ{�^���̕`��
	for (auto& button : m_buttons)
	{
		if (!(m_is_closed && button.first != EButtonType::CLOSED))
		{
			button.second->Draw();
		}
	}

	if (!m_is_closed)
	{
		drawString();
	}
}


void CameraGui::DrawBackground() const
{
	const unsigned int kBackColor = GetColor(255, 255, 255);

	SetDrawBlendMode(DX_BLENDMODE_ALPHA, 200);
	DrawBox(kGuiLeftPosX, kGuiTopPosY, kGuiLeftPosX + GUI_SIZE_X, kGuiTopPosY + GUI_SIZE_Y, kBackColor, TRUE);
	SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);
}


void CameraGui::drawClosedBackground() const
{
	const unsigned int kBackColor = GetColor(255, 255, 255);

	SetDrawBlendMode(DX_BLENDMODE_ALPHA, 200);
	DrawBox(kGuiLeftPosX, kGuiTopPosY, kGuiLeftPosX + GUI_SIZE_X, kGuiTopPosY + CLOSED_GUI_SIZE_Y, kBackColor, TRUE);
	SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);
}


void CameraGui::drawString() const
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
