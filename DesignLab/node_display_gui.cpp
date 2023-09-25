#include "node_display_gui.h"

#include <Dxlib.h>

#include "designlab_math_util.h"
#include "leg_state.h"

namespace dlm = designlab::math_util;


const int NodeDisplayGui::kWidth = 450;
const int NodeDisplayGui::kHeight = 550;
const int NodeDisplayGui::kClosedHeight = 50;


NodeDisplayGui::NodeDisplayGui(const int x_pos, const int y_pos, const std::shared_ptr<const AbstractHexapodStateCalculator>& calc) :
	kGuiLeftPosX(x_pos), kGuiTopPosY(y_pos), calculator_ptr_(calc), is_closed_(false), display_type_(DisplayMode::kDefualt)
{
	//�{�^�����쐬����
	const int kButtonSizeX = 100;
	const int kButtonSizeY = 30;

	buttons_[ButtonType::kOpenClose] = std::make_unique<ButtomController>(kGuiLeftPosX + kWidth - kButtonSizeX / 2 - 10, kGuiTopPosY + 10 + kButtonSizeY / 2,
		kButtonSizeX, kButtonSizeY, "�ő�/����");
	buttons_[ButtonType::kModeSwitching] = std::make_unique<ButtomController>(kGuiLeftPosX + kWidth - kButtonSizeX / 2 - 10, kGuiTopPosY + kHeight - kButtonSizeY / 2 - 10,
		kButtonSizeX, kButtonSizeY, "�؂�ւ�");

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		joint_state_[i].global_joint_position.resize(4);
		joint_state_[i].local_joint_position.resize(4);
		joint_state_[i].joint_angle.resize(3);
	}
}


void NodeDisplayGui::SetDisplayNode(const SNode& node)
{
	//�m�[�h���Z�b�g����
	display_node_ = node;

	if (!calculator_ptr_) { return; }

	// �֐߂̊p�x���Z�b�g����
	calculator_ptr_->CalculateAllJointState(display_node_, &joint_state_);
}


void NodeDisplayGui::Update()
{
	//�{�^���̍X�V���s��
	for (auto& button : buttons_)
	{
		button.second->Update();

		if (button.second->IsPushedNow() && button.first == ButtonType::kOpenClose)
		{
			is_closed_ = !is_closed_;
		}
		else if (button.second->IsPushedNow() && button.first == ButtonType::kModeSwitching && !is_closed_)
		{
			if (display_type_ == DisplayMode::kDefualt)
			{
				display_type_ = DisplayMode::kJointState;
			}
			else
			{
				display_type_ = DisplayMode::kDefualt;
			}
		}
	}
}


void NodeDisplayGui::Draw() const
{
	// �g
	DrawBackground();

	// �e�L�X�g
	if (!is_closed_)
	{
		if (display_type_ == DisplayMode::kDefualt)
		{
			DrawNodeInfo();
		}
		else
		{
			DrawJointInfo();
		}
	}


	//�{�^����`�悷��
	for (auto& button : buttons_)
	{
		if (!(button.first == ButtonType::kModeSwitching && is_closed_))
		{
			button.second->Draw();
		}
	}
}


void NodeDisplayGui::DrawBackground() const
{
	const unsigned int kBoxColor = GetColor(255, 255, 255);
	const unsigned int kBoxAlpha = 200;

	if (is_closed_)
	{
		SetDrawBlendMode(DX_BLENDMODE_ALPHA, kBoxAlpha);
		DrawBox(kGuiLeftPosX, kGuiTopPosY, kGuiLeftPosX + kWidth, kGuiTopPosY + kClosedHeight, kBoxColor, TRUE);
		SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);
	}
	else
	{
		SetDrawBlendMode(DX_BLENDMODE_ALPHA, kBoxAlpha);
		DrawBox(kGuiLeftPosX, kGuiTopPosY, kGuiLeftPosX + kWidth, kGuiTopPosY + kHeight, kBoxColor, TRUE);
		SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);
	}

}

void NodeDisplayGui::DrawNodeInfo() const
{
	const unsigned int kTextColor = GetColor(10, 10, 10);
	const unsigned int kBaseTextColor = GetColor(80, 80, 80);
	const int kTextXPos = kGuiLeftPosX + 10;
	const int kTextYMinPos = kGuiTopPosY + 10;
	const int kTextYInterval = 30;

	int text_line = 0;

	DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor, "�d�S�F%d�C�r�ʒu�F%d,%d,%d,%d,%d,%d", dl_leg::getComPatternState(display_node_.leg_state),
		dl_leg::getLegState(display_node_.leg_state, 0), dl_leg::getLegState(display_node_.leg_state, 1), dl_leg::getLegState(display_node_.leg_state, 2),
		dl_leg::getLegState(display_node_.leg_state, 3), dl_leg::getLegState(display_node_.leg_state, 4), dl_leg::getLegState(display_node_.leg_state, 5));

	// �d�S��\������
	DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor,
		"�d�S�ʒu(x:%5.3f,y:%5.3f,z:%5.3f)", display_node_.global_center_of_mass.x, display_node_.global_center_of_mass.y, display_node_.global_center_of_mass.z);

	// ��]��\������
	DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor,
		"��](x_angle:%5.3f,y_angle:%5.3f,z_angle:%5.3f)", display_node_.rot.x_angle, display_node_.rot.y_angle, display_node_.rot.z_angle);

	//�V�r���ڒn�r��
	std::string str = "";
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (dl_leg::IsGrounded(display_node_.leg_state, i)) { str += "�ڒn,"; }
		else { str += "�V�r,"; }
	}
	DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor, "�r�̏�ԁF%s", str.c_str());

	// �r�̈ʒu��\������
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor,
			"%d�ԋr�̈ʒu(x:%5.3f,y:%5.3f,z:%5.3f)", i, display_node_.leg_pos[i].x, display_node_.leg_pos[i].y, display_node_.leg_pos[i].z);
	}

	// �r�̊���W��\������
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kBaseTextColor,
			" %d�ԋr�̊���W(x:%5.3f,y:%5.3f,z:%5.3f)", i, display_node_.leg_base_pos[i].x, display_node_.leg_base_pos[i].y, display_node_.leg_base_pos[i].z);
	}

	// �[���Ǝ��̓����\������
	DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor,
		"�[���F%d, ���̓��� : %s", display_node_.depth, std::to_string(display_node_.next_move).c_str());

	DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor, "�P�ʂ͒�����[mm]�C�p�x��[rad]");
}


void NodeDisplayGui::DrawJointInfo() const
{
	if (!calculator_ptr_) { return; }


	const unsigned int kTextColor = GetColor(10, 10, 10);
	const unsigned int kErrorTextColor = GetColor(128, 10, 10);
	const int kTextXPos = kGuiLeftPosX + 10;
	const int kTextYMinPos = kGuiTopPosY + 50;
	const int kTextYInterval = 30;


	int text_line = 0;


	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor, "[%d] c %3.3f[deg],f %3.3f[deg],t %3.3f[deg]", i,
			dlm::ConvertRadToDeg(joint_state_[i].joint_angle[0]), dlm::ConvertRadToDeg(joint_state_[i].joint_angle[1]), dlm::ConvertRadToDeg(joint_state_[i].joint_angle[2]));

		DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor, "    c %3.3f[mm],f %3.3f[mm],t %3.3f[mm]",
			(joint_state_[i].local_joint_position[0] - joint_state_[i].local_joint_position[1]).Length(),
			(joint_state_[i].local_joint_position[1] - joint_state_[i].local_joint_position[2]).Length(),
			(joint_state_[i].local_joint_position[2] - joint_state_[i].local_joint_position[3]).Length()
		);


		if (calculator_ptr_->IsLegInRange(i, joint_state_[i].local_joint_position[3]))
		{
			DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor, "    �ߎ��l true");
		}
		else
		{
			DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kErrorTextColor, "    �ߎ��l false");
		}


		std::string str = "";
		if (joint_state_[i].joint_angle[0] < HexapodConst::PHANTOMX_COXA_DEFAULT_ANGLE[i] + HexapodConst::PHANTOMX_COXA_ANGLE_MIN) { str += "coxa_min "; }
		if (joint_state_[i].joint_angle[0] > HexapodConst::PHANTOMX_COXA_DEFAULT_ANGLE[i] + HexapodConst::PHANTOMX_COXA_ANGLE_MAX) { str += "coxa_max "; }
		if (joint_state_[i].joint_angle[1] < HexapodConst::PHANTOMX_FEMUR_ANGLE_MIN) { str += "femur_min "; }
		if (joint_state_[i].joint_angle[1] > HexapodConst::PHANTOMX_FEMUR_ANGLE_MAX) { str += "femur_max "; }
		if (joint_state_[i].joint_angle[2] < HexapodConst::PHANTOMX_TIBIA_ANGLE_MIN) { str += "tibia_min "; }
		if (joint_state_[i].joint_angle[2] > HexapodConst::PHANTOMX_TIBIA_ANGLE_MAX) { str += "tibia_max "; }

		if (!str.empty())
		{
			DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kErrorTextColor, "error %s", str.c_str());
		}
	}
}
