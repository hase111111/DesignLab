#include "graph_viewer_gui_controller.h"

#include <string>

#include <Dxlib.h>
#include <magic_enum.hpp>

#include "graphic_const.h"
#include "graph_search_const.h"
#include "keyboard.h"
#include "leg_state.h"


namespace dllf = designlab::leg_func;


GraphViewerGUIController::GraphViewerGUIController(const std::vector<RobotStateNode>* const p_graph, size_t* const p_display_node_index,
	const std::shared_ptr<const ApplicationSettingRecorder>& setting_ptr) :
	graph_ptr_(p_graph),
	display_node_index_ptr_(p_display_node_index),
	setting_ptr_(setting_ptr)
{
}


void GraphViewerGUIController::Update()
{
	InputNumber();
	UpdateChildrenList();
	ChangeDisplayNodeIndex();
}


void GraphViewerGUIController::Draw() const
{
	DrawGraphData();
	DrawNodeControllPanel();

	if (graph_ptr_->size() != 0 && *display_node_index_ptr_ < graph_ptr_->size())
	{
		//DrawNodeData(graph_ptr_->at(*display_node_index_ptr_));
	}
}


void GraphViewerGUIController::DrawGraphData() const
{
	const int kBoxSizeX = 250;
	const int kBoxSizeY = 200;
	const int kBoxMinX = setting_ptr_->window_size_x - kBoxSizeX - 10;
	const int kBoxMinY = setting_ptr_->window_size_y - kBoxSizeY - 10;
	const unsigned int kBaseColor = GetColor(255, 255, 255);

	// �g
	SetDrawBlendMode(DX_BLENDMODE_ALPHA, 128);
	DrawBox(kBoxMinX, kBoxMinY, kBoxMinX + kBoxSizeX, kBoxMinY + kBoxSizeY, kBaseColor, TRUE);
	SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);

	const unsigned int kTextColor = GetColor(10, 10, 10);

	// �e�L�X�g
	if (graph_ptr_->size() == 0)
	{
		DrawString(kBoxMinX + 10, kBoxMinY + 10, "�m�[�h�� : 0", kTextColor);
		DrawString(kBoxMinX + 10, kBoxMinY + 30, "�O���t�𐶐����Ă�������", kTextColor);
	}
	else
	{
		DrawFormatString(kBoxMinX + 10, kBoxMinY + 10, kTextColor, "���m�[�h��:%d", graph_ptr_->size());
		DrawFormatString(kBoxMinX + 10, kBoxMinY + 30, kTextColor, "�\���m�[�h:%d��", *display_node_index_ptr_);

		//�[�����Ƃ̃m�[�h�̐�
		for (size_t i = 0; i < graph_node_depth_data_.size(); i++)
		{
			DrawFormatString(kBoxMinX + 10, kBoxMinY + 50 + 20 * (int)i, kTextColor, "�@(�[��%d�̃m�[�h:%d)", (int)i, (int)graph_node_depth_data_.at(i));
		}
	}
}


void GraphViewerGUIController::DrawNodeControllPanel() const
{
	const int kBoxSizeX = 350;
	const int kBoxSizeY = 250;
	const int kBoxMinX = 10;
	const int kBoxMinY = 10;
	const unsigned int kBaseColor = GetColor(255, 255, 255);

	// �g
	SetDrawBlendMode(DX_BLENDMODE_ALPHA, 128);
	DrawBox(kBoxMinX, kBoxMinY, kBoxMinX + kBoxSizeX, kBoxMinY + kBoxSizeY, kBaseColor, TRUE);
	SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);

	const unsigned int kTextColor = GetColor(10, 10, 10);

	// �e�L�X�g
	DrawFormatString(kBoxMinX + 10, kBoxMinY + 10, kTextColor, "input ( C �ŃN���A)");
	if (input_number_ < 0) DrawFormatString(kBoxMinX + 10, kBoxMinY + 30, kTextColor, "�@��������͂��Ă�������");
	else DrawFormatString(kBoxMinX + 10, kBoxMinY + 30, kTextColor, "�@%d", input_number_);

	if (graph_ptr_->size() > *display_node_index_ptr_)
	{
		DrawFormatString(kBoxMinX + 10, kBoxMinY + 60, kTextColor, "%d�ԃm�[�h�̐e�m�[�h:%d��", *display_node_index_ptr_, graph_ptr_->at(*display_node_index_ptr_).parent_num);
		DrawFormatString(kBoxMinX + 10, kBoxMinY + 90, kTextColor, "%d�ԃm�[�h�̎q�m�[�h��:%d��", childen_list_.first, childen_list_.second.size());

		std::string str = childen_list_.second.size() == 0 ? "None" : "�@";

		for (size_t i = 0; i < childen_list_.second.size(); i++)
		{
			if (i > (size_t)6 * 5 - 1) { str += "..."; break; }

			str += std::to_string(childen_list_.second.at(i)) + ",";

			if (i % 6 == 0 && i != 0) { str += "\n�@"; }
		}

		DrawFormatString(kBoxMinX + 10, kBoxMinY + 110, kTextColor, "%d�ԃm�[�h�̎q�m�[�h���X�g", childen_list_.first);
		DrawFormatString(kBoxMinX + 10, kBoxMinY + 130, kTextColor, str.c_str());
		//DrawFormatString(kBoxMinX + 10, kBoxMinY + 230, kTextColor, "�@(�q�m�[�h���X�g�̍X�V�� U )");
		//DrawFormatString(kBoxMinX + 10, kBoxMinY + 250, kTextColor, "�@(�㉺�L�[�Ńm�[�h�ړ�)");
		//DrawFormatString(kBoxMinX + 10, kBoxMinY + 270, kTextColor, "�@(���E�L�[�ŕ\������q�m�[�h�؂�ւ�)");
		//DrawFormatString(kBoxMinX + 10, kBoxMinY + 290, kTextColor, "�@(Z�L�[�ŃJ�����\����؂�ւ�)");
	}
}


void GraphViewerGUIController::DrawNodeData(const RobotStateNode& node) const
{
	const int kBoxSizeX = 400;
	const int KBoxSizeY = 300;
	const int kBoxMinX = setting_ptr_->window_size_x - 25 - kBoxSizeX;
	const int kBoxMinY = 25;
	const unsigned int kBoxColor = GetColor(255, 255, 255);
	const unsigned int kBoxAlpha = 128;

	// �g
	SetDrawBlendMode(DX_BLENDMODE_ALPHA, kBoxAlpha);
	DrawBox(kBoxMinX, kBoxMinY, kBoxMinX + kBoxSizeX, kBoxMinY + KBoxSizeY, kBoxColor, TRUE);
	SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);

	// �e�L�X�g
	const unsigned int kTextColor = GetColor(10, 10, 10);
	const int kTextXPos = kBoxMinX + 10;
	const int kTextYMinPos = kBoxMinY + 10;
	const int kTextYInterval = 30;

	int text_line = 0;

	DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor, "�d�S�F%d�C�r�ʒu�F%d,%d,%d,%d,%d,%d", dllf::GetDiscreteComPos(node.leg_state),
		dllf::GetDiscreteLegPos(node.leg_state, 0), dllf::GetDiscreteLegPos(node.leg_state, 1), dllf::GetDiscreteLegPos(node.leg_state, 2),
		dllf::GetDiscreteLegPos(node.leg_state, 3), dllf::GetDiscreteLegPos(node.leg_state, 4), dllf::GetDiscreteLegPos(node.leg_state, 5));

	// �d�S��\������
	DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor,
		"�d�S�ʒu(x:%5.3f,y:%5.3f,z:%5.3f)", node.global_center_of_mass.x, node.global_center_of_mass.y, node.global_center_of_mass.z);

	//�V�r���ڒn�r��
	std::string str = "";
	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		if (dllf::IsGrounded(node.leg_state, i)) { str += "�ڒn,"; }
		else { str += "�V�r,"; }
	}
	DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor, "�r�̏�ԁF%s", str.c_str());

	// �r�̈ʒu��\������
	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor,
			"%d�ԋr�̈ʒu(x:%5.3f,y:%5.3f,z:%5.3f)", i, node.leg_pos[i].x, node.leg_pos[i].y, node.leg_pos[i].z);
	}

	// �[���Ǝ��̓����\������
	DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor,
		"�[���F%d, ���̓��� : %s", node.depth, static_cast<std::string>(magic_enum::enum_name(node.next_move)).c_str());
}


void GraphViewerGUIController::InputNumber()
{
	// C�L�[�Ń��Z�b�g
	if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_C) == 1)
	{
		input_number_ = -1;
		return;
	}

	// ��������
	int input_number = -1;

	if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_0) == 1) { input_number = 0; }
	else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_1) == 1) { input_number = 1; }
	else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_2) == 1) { input_number = 2; }
	else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_3) == 1) { input_number = 3; }
	else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_4) == 1) { input_number = 4; }
	else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_5) == 1) { input_number = 5; }
	else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_6) == 1) { input_number = 6; }
	else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_7) == 1) { input_number = 7; }
	else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_8) == 1) { input_number = 8; }
	else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_9) == 1) { input_number = 9; }

	if (input_number >= 0)
	{
		if (input_number_ < 0)
		{
			input_number_ = input_number;
		}
		else
		{
			input_number_ *= 10;
			input_number_ += input_number;
		}
	}
}


void GraphViewerGUIController::ChangeDisplayNodeIndex()
{
	if (graph_ptr_->size() == 0) return;

	if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_DOWN) == 1)
	{
		(*display_node_index_ptr_)--;
	}
	else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_UP) == 1)
	{
		(*display_node_index_ptr_)++;
	}
	else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_RETURN) == 1)
	{
		(*display_node_index_ptr_) = input_number_;
		input_number_ = -1;
	}
	else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_P) == 1)
	{
		if (graph_ptr_->size() > *display_node_index_ptr_)
		{
			(*display_node_index_ptr_) = graph_ptr_->at(*display_node_index_ptr_).parent_num;
		}
	}
	else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_LEFT) == 1 && !childen_list_.second.empty())
	{
		display_children_list_index_--;

		if (display_children_list_index_ < 0) display_children_list_index_ = (int)childen_list_.second.size() - 1;
		display_children_list_index_ = display_children_list_index_ < 0 ? 0 : display_children_list_index_;

		(*display_node_index_ptr_) = childen_list_.second.at(display_children_list_index_);
	}
	else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_RIGHT) == 1 && !childen_list_.second.empty())
	{
		display_children_list_index_++;

		if (display_children_list_index_ >= childen_list_.second.size()) display_children_list_index_ = 0;
		(*display_node_index_ptr_) = childen_list_.second.at(display_children_list_index_);
	}


	if (*display_node_index_ptr_ < 0)*display_node_index_ptr_ = graph_ptr_->size() - 1;
	else if (*display_node_index_ptr_ >= graph_ptr_->size()) *display_node_index_ptr_ = 0;
}


void GraphViewerGUIController::UpdateChildrenList()
{
	if (graph_ptr_->size() == 0) return;

	if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_U) == 1)
	{
		childen_list_.first = (int)(*display_node_index_ptr_);
		childen_list_.second.clear();

		const size_t kGraphSize = graph_ptr_->size();

		for (size_t i = 0; i < kGraphSize; i++)
		{
			if (graph_ptr_->at(i).parent_num == childen_list_.first)
			{
				childen_list_.second.push_back((int)i);
			}
		}
	}
}


void GraphViewerGUIController::UpdateGraphNodeDepthData()
{
	graph_node_depth_data_.clear();

	if (graph_ptr_->size() > 0)
	{
		graph_node_depth_data_.resize((size_t)GraphSearchConst::kMaxDepth + 1);

		for (size_t i = 0; i < graph_ptr_->size(); ++i)
		{
			graph_node_depth_data_.at(static_cast<size_t>(graph_ptr_->at(i).depth))++;
		}
	}
}