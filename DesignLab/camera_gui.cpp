#include "camera_gui.h"

#include <string>

#include "designlab_dxlib.h"
#include "mouse.h"


CameraGUI::CameraGUI(const int left_x, const int top_y) :
	kGUILeftPosX(left_x),
	kGUITopPosY(top_y),
	m_camera_controller(&m_camera_manager)
{
	const int kCloseButtonSizeX = 100;
	const int kCloseButtonSizeY = 30;

	m_buttons[EButtonType::CLOSED] = std::make_unique<ButtomController>(kGUILeftPosX + GUI_SIZE_X - kCloseButtonSizeX / 2 - 10, kGUITopPosY + kCloseButtonSizeY - 10,
		kCloseButtonSizeX, kCloseButtonSizeY, "�ő�/����");

	const int kButtonRange = kButtonSize + kButtonDistance;
	const int kLeftPosX = kGUILeftPosX + kButtonRange / 2 + 15;
	const int kTopPosY = kGUITopPosY + kButtonRange / 2 + kCloseButtonSizeY + 10;

	m_buttons[EButtonType::LENGH_RESET] = std::make_unique<ButtomController>(kLeftPosX, kTopPosY, kButtonSize, kButtonSize, "Reset\nZoom");
	m_buttons[EButtonType::FRONT] = std::make_unique<ButtomController>(kLeftPosX + kButtonRange, kTopPosY, kButtonSize, kButtonSize, "Front");
	m_buttons[EButtonType::LEFT] = std::make_unique<ButtomController>(kLeftPosX, kTopPosY + kButtonRange, kButtonSize, kButtonSize, "Left");
	m_buttons[EButtonType::TOP] = std::make_unique<ButtomController>(kLeftPosX + kButtonRange, kTopPosY + kButtonRange, kButtonSize, kButtonSize, "Top");
	m_buttons[EButtonType::RIGHT] = std::make_unique<ButtomController>(kLeftPosX + kButtonRange * 2, kTopPosY + kButtonRange, kButtonSize, kButtonSize, "Right");
	m_buttons[EButtonType::BACK] = std::make_unique<ButtomController>(kLeftPosX + kButtonRange, kTopPosY + kButtonRange * 2, kButtonSize, kButtonSize, "Back");
	m_buttons[EButtonType::TARGET_RESET] = std::make_unique<ButtomController>(kLeftPosX + kButtonRange * 2, kTopPosY, kButtonSize, kButtonSize, "Reset\nTarget");

}


CameraGUI::CameraGUI() : CameraGUI::CameraGUI(10, 10)
{
}


void CameraGUI::setHexapodPos(const dl_vec::SVector pos)
{
	m_camera_manager.setTargetPos(dl_dxlib::convertToDxVec(pos));
}


void CameraGUI::update()
{

	//�e�{�^���̏���
	for (auto& button : m_buttons)
	{
		button.second->update();

		//�{�^���������ꂽ��(�ŏ����{�^���ȊO)
		if (button.second->isPushedNow() && !m_is_closed)
		{
			//�{�^���̎�ނɂ���ď�����ς���
			switch (button.first)
			{
			case EButtonType::LENGH_RESET:
				m_camera_manager.initCaneraTargetLength();
				break;

			case EButtonType::FRONT:
				m_camera_manager.setCameraViewMode(ECameraMode::FRONT_VIEW);
				break;

			case EButtonType::LEFT:
				m_camera_manager.setCameraViewMode(ECameraMode::LEFT_SIDE_VIEW);
				break;

			case EButtonType::TOP:
				m_camera_manager.setCameraViewMode(ECameraMode::TOP_VIEW);
				break;

			case EButtonType::RIGHT:
				m_camera_manager.setCameraViewMode(ECameraMode::RIGHT_SIDE_VIEW);
				break;

			case EButtonType::BACK:
				m_camera_manager.setCameraViewMode(ECameraMode::BACK_VIEW);
				break;

			case EButtonType::TARGET_RESET:
				m_camera_manager.setCameraViewMode(ECameraMode::FREE_CONTROLLED);
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
	m_camera_controller.update();

	//�J�����̍X�V
	m_camera_manager.update();
}


void CameraGUI::draw() const
{
	if (m_is_closed)
	{
		drawClosedBackground();
	}
	else
	{
		drawBackground();
	}


	//�S�Ẵ{�^���̕`��
	for (auto& button : m_buttons)
	{
		if (!(m_is_closed && button.first != EButtonType::CLOSED))
		{
			button.second->draw();
		}
	}

	if (!m_is_closed)
	{
		drawString();
	}
}


void CameraGUI::drawBackground() const
{
	const unsigned int kBackColor = GetColor(255, 255, 255);

	SetDrawBlendMode(DX_BLENDMODE_ALPHA, 200);
	DrawBox(kGUILeftPosX, kGUITopPosY, kGUILeftPosX + GUI_SIZE_X, kGUITopPosY + GUI_SIZE_Y, kBackColor, TRUE);
	SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);
}


void CameraGUI::drawClosedBackground() const
{
	const unsigned int kBackColor = GetColor(255, 255, 255);

	SetDrawBlendMode(DX_BLENDMODE_ALPHA, 200);
	DrawBox(kGUILeftPosX, kGUITopPosY, kGUILeftPosX + GUI_SIZE_X, kGUITopPosY + CLOSED_GUI_SIZE_Y, kBackColor, TRUE);
	SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);
}


void CameraGUI::drawString() const
{
	const unsigned int kStrColor = GetColor(54, 54, 54);
	const unsigned int kStrRedColor = GetColor(255, 128, 128);

	const int kTextYInterval = 20;
	const int kTextYTop = kGUITopPosY + 250;

	int text_line = 0;

	DrawFormatString(kGUILeftPosX + 10, kTextYTop + kTextYInterval * (text_line++),
		Mouse::getIns()->getWheelRot() == 0 ? kStrColor : kStrRedColor, "�}�E�X�z�C�[����]");

	DrawFormatString(kGUILeftPosX + 10, kTextYTop + kTextYInterval * (text_line++),
		Mouse::getIns()->getWheelRot() == 0 ? kStrColor : kStrRedColor, " �E�Y�[��");

	DrawFormatString(kGUILeftPosX + 10, kTextYTop + kTextYInterval * (text_line++),
		Mouse::getIns()->getPushingCountMiddle() == 0 ? kStrColor : kStrRedColor, "�z�C�[���N���b�N���h���b�O");

	DrawFormatString(kGUILeftPosX + 10, kTextYTop + kTextYInterval * (text_line++),
		Mouse::getIns()->getPushingCountMiddle() == 0 ? kStrColor : kStrRedColor, " �E�r���[����]");

	DrawFormatString(kGUILeftPosX + 10, kTextYTop + kTextYInterval * (text_line++),
		Mouse::getIns()->getPushingCountLeft() == 0 ? kStrColor : kStrRedColor, "���N���b�N���h���b�O");

	DrawFormatString(kGUILeftPosX + 10, kTextYTop + kTextYInterval * (text_line++),
		Mouse::getIns()->getPushingCountLeft() == 0 ? kStrColor : kStrRedColor, " �E��ʂ̒��S�����]");

	DrawFormatString(kGUILeftPosX + 10, kTextYTop + kTextYInterval * (text_line++),
		Mouse::getIns()->getPushingCountRight() == 0 ? kStrColor : kStrRedColor, "�E�N���b�N���h���b�O");

	DrawFormatString(kGUILeftPosX + 10, kTextYTop + kTextYInterval * (text_line++),
		Mouse::getIns()->getPushingCountRight() == 0 ? kStrColor : kStrRedColor, " �E��ʂ̕��s�ړ�");
}
