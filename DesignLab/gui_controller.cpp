#include "gui_controller.h"

#include <string>

#include "DxLib.h"

#include "leg_state.h"
#include "keyboard.h"


GUIController::GUIController()
{
	const int RIGHTX = GraphicConst::WIN_X - (CENTER_X + BOX_X / 2);
	const int RIGHTY = CENTER_Y - BOX_Y / 2;

	const int kCameraButtomX = 100;
	const int kCameraButtomY = 50;
	const int kCameraY = RIGHTY + 150;

	m_buttom.push_back(std::make_unique<ButtomController>(RIGHTX + kCameraButtomX * 6 / 4, kCameraY, kCameraButtomX, kCameraButtomY, "�^��J����"));
	m_buttom.push_back(std::make_unique<ButtomController>(RIGHTX + kCameraButtomX * 9 / 4, kCameraY + kCameraButtomY * 5 / 4, kCameraButtomX, kCameraButtomY, "���ՃJ����"));
	m_buttom.push_back(std::make_unique<ButtomController>(RIGHTX + kCameraButtomX * 9 / 4, kCameraY + kCameraButtomY * 10 / 4, kCameraButtomX, kCameraButtomY, "�^���J����"));
	m_buttom.push_back(std::make_unique<ButtomController>(RIGHTX + kCameraButtomX * 3 / 4, kCameraY + kCameraButtomY * 5 / 4, kCameraButtomX, kCameraButtomY, "����(���])"));
	m_buttom.push_back(std::make_unique<ButtomController>(RIGHTX + kCameraButtomX * 3 / 4, kCameraY + kCameraButtomY * 10 / 4, kCameraButtomX, kCameraButtomY, "�^��(���])"));

	m_buttom.push_back(std::make_unique<ButtomController>(RIGHTX + kCameraButtomX * 6 / 4, kCameraY + kCameraButtomY * 20 / 4, kCameraButtomX * 2, kCameraButtomY, "�m�[�h�\���؂�ւ�"));
	m_buttom.push_back(std::make_unique<ButtomController>(RIGHTX + kCameraButtomX * 3 / 4, kCameraY + kCameraButtomY * 25 / 4, kCameraButtomX, kCameraButtomY, "��"));
	m_buttom.push_back(std::make_unique<ButtomController>(RIGHTX + kCameraButtomX * 9 / 4, kCameraY + kCameraButtomY * 25 / 4, kCameraButtomX, kCameraButtomY, "��"));
}


void GUIController::update(CameraController& camera_controller, const int max_node, int& display_node, const int counter)
{
	if (m_mode == ENodeDisplayNode::AUTO_UPDATE)
	{
		if (counter % CHANGE_NEXT_NODE == 0)
		{
			display_node++;
		}
	}
	else if (m_mode == ENodeDisplayNode::ALWAYS_NEW)
	{
		display_node = max_node - 1;
	}


	//�\���ؑփL�[�������ꂽ�Ȃ�΁C�\���؂�Ԃ��t���O�̒l���������܂ɂ���D
	if (Keyboard::getIns()->getPressingCount(KEY_INPUT_Z) == 1) { m_is_displayed = !m_is_displayed; }

	if (m_is_displayed)
	{
		//�\�����Ă��Ȃ����Ƀ{�^���̏������s��Ȃ�

		//�{�^�����X�V����D
		for (int i = 0; i < (int)m_buttom.size(); i++)
		{
			m_buttom.at(i)->update();

			//�{�^���̒ǉ����ꂽ���Ԃ��Ƃɏ���������D�N�\�݂����Ȏ����ŃX�}�\
			switch (i)
			{
			case 0:
				if (m_buttom.at(i)->isPushedNow()) { camera_controller.setCameraMode(ECameraMode::TOP_VIEW); }
				break;

			case 1:
				if (m_buttom.at(i)->isPushedNow()) { camera_controller.setCameraMode(ECameraMode::OVERHEAD_VIEW); }
				break;

			case 2:
				if (m_buttom.at(i)->isPushedNow()) { camera_controller.setCameraMode(ECameraMode::SIDE_VIEW); }
				break;

			case 3:
				if (m_buttom.at(i)->isPushedNow()) { camera_controller.setCameraMode(ECameraMode::OVERHEAD_VIEW_FLIP); }
				break;

			case 4:
				if (m_buttom.at(i)->isPushedNow()) { camera_controller.setCameraMode(ECameraMode::SIDE_VIEW_FLIP); }
				break;

			case 5:
				if (m_buttom.at(i)->isPushedNow())
				{
					if (m_mode == ENodeDisplayNode::SELECTABLE)m_mode = ENodeDisplayNode::ALWAYS_NEW;
					else if (m_mode == ENodeDisplayNode::ALWAYS_NEW)m_mode = ENodeDisplayNode::AUTO_UPDATE;
					else if (m_mode == ENodeDisplayNode::AUTO_UPDATE)m_mode = ENodeDisplayNode::SELECTABLE;
				}
				break;

			case 6:
				if (m_buttom.at(i)->isPushedNow()) { display_node--; }
				break;

			case 7:
				if (m_buttom.at(i)->isPushedNow()) { display_node++; }
				break;

			default:
				break;
			}
		}
	}


	if (display_node < 0) { display_node = 0; }
	if (display_node >= max_node) { display_node = max_node - 1; }
}


void GUIController::draw(const SNode& node) const
{
	//�\�����Ȃ��Ȃ瑦�I��
	if (m_is_displayed == false) { return; }

	const int kBaseColor = GetColor(255, 255, 255);

	//�������ɕ`�悷��D
	SetDrawBlendMode(DX_BLENDMODE_ALPHA, 200);

	//���n��`�悷��
	DrawBox(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2, CENTER_X + BOX_X / 2, CENTER_Y + BOX_Y / 2, kBaseColor, TRUE);

	//�E��
	DrawBox(GraphicConst::WIN_X - (CENTER_X - BOX_X / 2), CENTER_Y - BOX_Y / 2, GraphicConst::WIN_X - (CENTER_X + BOX_X / 2), CENTER_Y + BOX_Y / 2, kBaseColor, TRUE);

	//���������猳�ɖ߂��D����Y���ƕ`�悪���󂷂�̂Œ���
	SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);

	//�m�[�h�̏�Ԃ𕶎��ŏo�͂���D
	drawNodeByStr(node);

	drawExplanationByStr();

	//�{�^����`�悷��D
	for (const auto& i : m_buttom)
	{
		i->draw();
	}
}


void GUIController::drawNodeByStr(const SNode node) const
{
	const int kStrColor = GetColor(54, 54, 54);
	const int kLineHeight = 28;

	//���݂̍s������\������n�_������o���Ă����֐��I�u�W�F�N�g�D
	int now_line = 0;
	auto line = [kLineHeight, &now_line]() -> int { return kLineHeight * (now_line++); };

	//�����Ȃ����̂ŃX�R�[�v�̒��ɂ���Ă����C�ʂɓ���Ȃ��Ƃ������͕ς��Ȃ��̂����ǁCvisual stdio�̋@�\�Ő܂肽���߂�̂Ō����ڂ𐮗��ł���D

	//�r���
	{
		std::string leg_state_bit_str = "";

		//int �^��32bit�Ȃ̂� 32�񃋁[�v����D
		const int kIntegerBitSize = 32;

		for (int i = 4; i < kIntegerBitSize; i++)
		{
			//�\������s����C���bit���璲�ׂ�D
			if (node.leg_state & 1 << (kIntegerBitSize - i - 1))
			{
				//bit�������Ă���Ȃ�
				leg_state_bit_str += "1";
			}
			else
			{
				//bit�������Ă��Ȃ��Ȃ�
				leg_state_bit_str += "0";
			}

			// 4�Ŋ������]�肪3�Ȃ�C4bit���ƂɃX�y�[�X�����ꂽ���D
			if (i % 4 == 3)
			{
				leg_state_bit_str += " ";
			}
		}

		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), kStrColor, "�r���");
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), kStrColor, leg_state_bit_str.c_str());
		line();		//���s
	}

	//�r�ׂ̍������D
	{
		std::string temp_str = "";
		auto ground_or_lift = [](int _state, int _num) -> std::string
		{
			if (dl_leg::isGrounded(_state, _num) == true) { return "�ڒn"; }
			else { return "�V�r"; }
		};

		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), kStrColor, "�r�T�@�@�@�@�@�@�r�O");
		temp_str = "�@" + ground_or_lift(node.leg_state, 5) + "�@�@�@�@�@�@" + ground_or_lift(node.leg_state, 0);
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), kStrColor, temp_str.c_str());
		temp_str = "�@���U���ʒu�F" + std::to_string(dl_leg::getLegState(node.leg_state, 5)) + " ���U���ʒu�F" + std::to_string(dl_leg::getLegState(node.leg_state, 0));
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), kStrColor, temp_str.c_str());

		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), kStrColor, "�r�S�@�@�@�@�@�@�r�P");
		temp_str = "�@" + ground_or_lift(node.leg_state, 4) + "�@�@�@�@�@�@" + ground_or_lift(node.leg_state, 1);
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), kStrColor, temp_str.c_str());
		temp_str = "�@���U���ʒu�F" + std::to_string(dl_leg::getLegState(node.leg_state, 4)) + " ���U���ʒu�F" + std::to_string(dl_leg::getLegState(node.leg_state, 1));
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), kStrColor, temp_str.c_str());

		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), kStrColor, "�r�R�@�@�@�@�@�@�r�Q");
		temp_str = "�@" + ground_or_lift(node.leg_state, 3) + "�@�@�@�@�@�@" + ground_or_lift(node.leg_state, 2);
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), kStrColor, temp_str.c_str());
		temp_str = "�@���U���ʒu�F" + std::to_string(dl_leg::getLegState(node.leg_state, 3)) + " ���U���ʒu�F" + std::to_string(dl_leg::getLegState(node.leg_state, 2));
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), kStrColor, temp_str.c_str());
	}

	//�d�S�ɂ���
	{
		std::string str = "�d�S�p�^�[��" + std::to_string(dl_leg::getComPatternState(node.leg_state));

		line();	//���s
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), kStrColor, str.c_str());

		str = "�d�S���W";
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), kStrColor, str.c_str());
		str = "�@x = " + std::to_string(node.global_center_of_mass.x);
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), kStrColor, str.c_str());
		str = "�@y = " + std::to_string(node.global_center_of_mass.y);
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), kStrColor, str.c_str());
		str = "�@z = " + std::to_string(node.global_center_of_mass.z);
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), kStrColor, str.c_str());
	}

	//��]�ɂ���
	{
		std::string str = "��]";

		line();	//���s
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), kStrColor, str.c_str());

		str = "�@pitch = " + std::to_string(node.rot.pitch);
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), kStrColor, str.c_str());
		str = "�@roll  = " + std::to_string(node.rot.roll);
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), kStrColor, str.c_str());
		str = "�@yaw   = " + std::to_string(node.rot.yaw);
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), kStrColor, str.c_str());
	}
}


void GUIController::drawExplanationByStr() const
{
	const int kRightX = GraphicConst::WIN_X - (CENTER_X + BOX_X / 2);
	const int kStrColor = GetColor(54, 54, 54);
	const int kLineHeight = 28;

	//���݂̍s������\������n�_������o���Ă����֐��I�u�W�F�N�g�D
	int now_line = 0;
	auto line = [kLineHeight, &now_line]() -> int { return kLineHeight * (now_line++); };

	DrawFormatString(kRightX, CENTER_Y - BOX_Y / 2 + line(), kStrColor, "Z�L�[���������Ƃ�UI�̕\���������܂��D");
	DrawFormatString(kRightX, CENTER_Y - BOX_Y / 2 + line(), kStrColor, "������xZ�L�[���������ƂŖ߂�܂�.");
	DrawFormatString(kRightX, CENTER_Y - BOX_Y / 2 + line(), kStrColor, "��{�I�ɂ̓N���b�N�ő�������܂��D");
	DrawFormatString(kRightX, CENTER_Y - BOX_Y / 2 + line(), kStrColor, "�ȉ��̃{�^���ŃJ�����̎��_��ύX�D");

	//�{�^�����u���Ă��镪�������s����
	for (int i = 0; i < 7; i++) { line(); }

	{
		std::string str = "�\�����@�F";

		if (m_mode == ENodeDisplayNode::ALWAYS_NEW) { str += "��ɍŐV�D"; }
		else if (m_mode == ENodeDisplayNode::AUTO_UPDATE) { str += "�����ōX�V"; }
		else if (m_mode == ENodeDisplayNode::SELECTABLE) { str += "�{�^���őI��"; }
		DrawFormatString(kRightX, CENTER_Y - BOX_Y / 2 + line(), kStrColor, str.c_str());
	}

}
