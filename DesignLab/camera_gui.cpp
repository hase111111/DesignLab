#include "camera_gui.h"

#include <string>

#include "designlab_dxlib.h"
#include "mouse.h"


CameraGUI::CameraGUI() : m_camera_controller(&m_camera_manager)
{
	const int kButtonRange = kButtonSize + kButtonDistance;
	const int kLeftPosX = kGUILeftPosX + kButtonRange / 2;
	const int kTopPosY = kGUITopPosY + kButtonRange / 2;

	m_buttons[EButtonType::LENGH_RESET] = std::make_unique<ButtomController>(kLeftPosX, kTopPosY, kButtonSize, kButtonSize, "Reset\nZoom");
	m_buttons[EButtonType::FRONT] = std::make_unique<ButtomController>(kLeftPosX + kButtonRange, kTopPosY, kButtonSize, kButtonSize, "Front");
	m_buttons[EButtonType::LEFT] = std::make_unique<ButtomController>(kLeftPosX, kTopPosY + kButtonRange, kButtonSize, kButtonSize, "Left");
	m_buttons[EButtonType::TOP] = std::make_unique<ButtomController>(kLeftPosX + kButtonRange, kTopPosY + kButtonRange, kButtonSize, kButtonSize, "Top");
	m_buttons[EButtonType::RIGHT] = std::make_unique<ButtomController>(kLeftPosX + kButtonRange * 2, kTopPosY + kButtonRange, kButtonSize, kButtonSize, "Right");
	m_buttons[EButtonType::BACK] = std::make_unique<ButtomController>(kLeftPosX + kButtonRange, kTopPosY + kButtonRange * 2, kButtonSize, kButtonSize, "Back");
	m_buttons[EButtonType::TARGET_RESET] = std::make_unique<ButtomController>(kLeftPosX + kButtonRange * 2, kTopPosY, kButtonSize, kButtonSize, "Reset\nTarget");
}


void CameraGUI::setHexapodPos(const dl_vec::SVector pos)
{
	m_camera_manager.setTargetPos(dl_dxlib::convertToDxVec(pos));
}


void CameraGUI::update()
{

	//各ボタンの処理
	for (auto& button : m_buttons)
	{
		button.second->update();

		//ボタンが押されたら
		if (button.second->isPushedNow())
		{
			//ボタンの種類によって処理を変える
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
			}
		}
	}


	//キーボードによるカメラの操作
	m_camera_controller.update();

	//カメラの更新
	m_camera_manager.update();
}


void CameraGUI::draw() const
{
	drawBackground();

	//全てのボタンの描画
	for (auto& button : m_buttons)
	{
		button.second->draw();
	}

	drawString();
}


void CameraGUI::drawBackground() const
{
	const unsigned int kBackColor = GetColor(255, 255, 255);
	const int kGUIRightSize = 25;
	const int kGUILowerSize = 170;

	SetDrawBlendMode(DX_BLENDMODE_ALPHA, 200);
	DrawBox(kGUILeftPosX, kGUITopPosY, kGUILeftPosX + (kButtonSize + kButtonDistance) * 3 + kGUIRightSize, kGUITopPosY + (kButtonSize + kButtonDistance) * 3 + kGUILowerSize, kBackColor, TRUE);
	SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);
}


void CameraGUI::drawString() const
{
	const unsigned int kStrColor = GetColor(54, 54, 54);
	const unsigned int kStrRedColor = GetColor(255, 128, 128);
	const int kTextYInterval = 20;

	int text_line = 0;

	DrawFormatString(kGUILeftPosX + 10, kGUITopPosY + (kButtonSize + kButtonDistance) * 3 + kTextYInterval * (text_line++),
		Mouse::getIns()->getWheelRot() == 0 ? kStrColor : kStrRedColor, "マウスホイール回転");

	DrawFormatString(kGUILeftPosX + 10, kGUITopPosY + (kButtonSize + kButtonDistance) * 3 + kTextYInterval * (text_line++),
		Mouse::getIns()->getWheelRot() == 0 ? kStrColor : kStrRedColor, " ・ズーム");

	DrawFormatString(kGUILeftPosX + 10, kGUITopPosY + (kButtonSize + kButtonDistance) * 3 + kTextYInterval * (text_line++),
		Mouse::getIns()->getPushingCountMiddle() == 0 ? kStrColor : kStrRedColor, "ホイールクリック＆ドラッグ");

	DrawFormatString(kGUILeftPosX + 10, kGUITopPosY + (kButtonSize + kButtonDistance) * 3 + kTextYInterval * (text_line++),
		Mouse::getIns()->getPushingCountMiddle() == 0 ? kStrColor : kStrRedColor, " ・ビューを回転");

	DrawFormatString(kGUILeftPosX + 10, kGUITopPosY + (kButtonSize + kButtonDistance) * 3 + kTextYInterval * (text_line++),
		Mouse::getIns()->getPushingCountLeft() == 0 ? kStrColor : kStrRedColor, "左クリック＆ドラッグ");

	DrawFormatString(kGUILeftPosX + 10, kGUITopPosY + (kButtonSize + kButtonDistance) * 3 + kTextYInterval * (text_line++),
		Mouse::getIns()->getPushingCountLeft() == 0 ? kStrColor : kStrRedColor, " ・画面の中心から回転");

	DrawFormatString(kGUILeftPosX + 10, kGUITopPosY + (kButtonSize + kButtonDistance) * 3 + kTextYInterval * (text_line++),
		Mouse::getIns()->getPushingCountRight() == 0 ? kStrColor : kStrRedColor, "右クリック＆ドラッグ");

	DrawFormatString(kGUILeftPosX + 10, kGUITopPosY + (kButtonSize + kButtonDistance) * 3 + kTextYInterval * (text_line++),
		Mouse::getIns()->getPushingCountRight() == 0 ? kStrColor : kStrRedColor, " ・画面の平行移動");
}
