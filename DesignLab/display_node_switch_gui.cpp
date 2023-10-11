#include "display_node_switch_gui.h"

#include <algorithm>

#include <Dxlib.h>



DisplayNodeSwitchGui::DisplayNodeSwitchGui(const int x, const int y) : 
	kGuiLeftPosX(x), 
	kGuiTopPosY(y),
	kAnimeSpeedMax(120),
	kAnimeSpeedMin(1),
	display_node_num_(0), 
	all_node_num_(0), 
	simulation_num_(0),
	counter_(0), 
	do_auto_animation_(false), 
	animation_speed_(kAnimeSpeedMin)
{
	const int kButtonDif = 10;
	const int kButtonWidth = 40;
	const int kButtonLeftX = kButtonWidth / 2 + kGuiLeftPosX + 10;
	const int kButtonTopY = kButtonWidth / 2 + kGuiTopPosY + 100;


	//�e��{�^�����쐬����
	button_[ButtonType::kMostPrevNode] = std::make_unique<ButtomController>(kButtonLeftX, kButtonTopY, kButtonWidth, kButtonWidth, "<<");
	button_[ButtonType::kPrevNode] = std::make_unique<ButtomController>(kButtonLeftX + (kButtonDif + kButtonWidth) * 1, kButtonTopY, kButtonWidth, kButtonWidth, "<");
	button_[ButtonType::kPlayStop] = std::make_unique<ButtomController>(kButtonLeftX + (kButtonDif + kButtonWidth) * 2, kButtonTopY, kButtonWidth, kButtonWidth, "�Đ�\n��~");
	button_[ButtonType::kNextNode] = std::make_unique<ButtomController>(kButtonLeftX + (kButtonDif + kButtonWidth) * 3, kButtonTopY, kButtonWidth, kButtonWidth, ">");
	button_[ButtonType::kMostNextNode] = std::make_unique<ButtomController>(kButtonLeftX + (kButtonDif + kButtonWidth) * 4, kButtonTopY, kButtonWidth, kButtonWidth, ">>");
	button_[ButtonType::kSpeedDown] = std::make_unique<ButtomController>(kButtonLeftX + (kButtonDif + kButtonWidth) * 3, kButtonTopY + (kButtonDif + kButtonWidth), kButtonWidth, kButtonWidth, "��");
	button_[ButtonType::kSpeedUp] = std::make_unique<ButtomController>(kButtonLeftX + (kButtonDif + kButtonWidth) * 4, kButtonTopY + (kButtonDif + kButtonWidth), kButtonWidth, kButtonWidth, "��");

	button_[ButtonType::kPrevSimu] = std::make_unique<ButtomController>(kButtonLeftX + (kButtonDif + kButtonWidth) * 1, kButtonTopY + (kButtonDif + kButtonWidth) * 2,
		kButtonWidth * 2, kButtonWidth, "Prev Simu");
	button_[ButtonType::kNextSimu] = std::make_unique<ButtomController>(kButtonLeftX + (kButtonDif + kButtonWidth) * 3, kButtonTopY + (kButtonDif + kButtonWidth) * 2,
		kButtonWidth * 2, kButtonWidth, "Next Simu");
}


DisplayNodeSwitchGui::DisplayNodeSwitchGui() : DisplayNodeSwitchGui::DisplayNodeSwitchGui(0, 0)
{
}


void DisplayNodeSwitchGui::SetGraphData(const size_t node_num, const std::vector<size_t>& simu_end_index)
{
	all_node_num_ = node_num;

	simu_end_index_.clear();

	simu_end_index_ = simu_end_index;
}


size_t DisplayNodeSwitchGui::GetDisplayNodeNum() const
{
	// �͈͊O�̒l��Ԃ��Ȃ��悤�ɂ���D
	if (display_node_num_ > all_node_num_ && all_node_num_ != 0) { return all_node_num_ - 1; }


	// �͈͓��̒l�Ȃ�΁C���̂܂ܕԂ�
	return display_node_num_;
}


int DisplayNodeSwitchGui::GetSimulationNum() const
{
	return simulation_num_;
}


void DisplayNodeSwitchGui::Update()
{
	++counter_;


	// �����Đ����s��
	if (do_auto_animation_ && counter_ % (180 / animation_speed_) == 0)
	{
		MoveNextNode();
	}


	// �{�^�����X�V����
	for (auto& i : button_)
	{
		i.second->Update();

		if (i.second->IsPushedNow())
		{
			switch (i.first)
			{
			case ButtonType::kMostPrevNode:
				MoveMostPrevNode();
				break;

			case ButtonType::kMostNextNode:
				MoveMostNextNode();
				break;

			case ButtonType::kPrevNode:
				MovePrevNode();
				break;

			case ButtonType::kNextNode:
				MoveNextNode();
				break;

			case ButtonType::kPrevSimu:
				MovePrevSimulation();
				break;

			case ButtonType::kNextSimu:
				MoveNextSimulation();
				break;

			case ButtonType::kPlayStop:
				do_auto_animation_ = !do_auto_animation_;
				break;
			}
		}

		if (i.second->IsPushedNow())
		{
			if (i.first == ButtonType::kSpeedDown)
			{
				if (animation_speed_ > kAnimeSpeedMin)
				{
					--animation_speed_;
				}
			}
			else if (i.first == ButtonType::kSpeedUp)
			{
				if (animation_speed_ < kAnimeSpeedMax)
				{
					++animation_speed_;
				}
			}
		}

	}
}


void DisplayNodeSwitchGui::Draw() const
{
	const unsigned int kAlpha = 200;

	const unsigned int kColor = GetColor(255, 255, 255);

	// �{�b�N�X��`�悷��
	SetDrawBlendMode(DX_BLENDMODE_ALPHA, kAlpha);

	DrawBox(kGuiLeftPosX, kGuiTopPosY, kGuiLeftPosX + kGuiWidth, kGuiTopPosY + kGuiHeight, kColor, TRUE);

	SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);


	// �{�^����`�悷��
	for (const auto& i : button_)
	{
		i.second->Draw();
	}


	//������`�悷��

	const int kTextLeftX = kGuiLeftPosX + 10;

	const unsigned int kTextColor = GetColor(0, 0, 0);


	DrawFormatString(kTextLeftX, kGuiTopPosY + 10, kTextColor, "[�V�~�����[�V����%d���(�S%d��)]", simulation_num_ + 1, GetAllSimulationNum(), display_node_num_, all_node_num_);

	int start_node_num = 0;
	int end_node_num = 0;

	if (simulation_num_ == 0)
	{
		start_node_num = 0;
		if (!simu_end_index_.empty()) { end_node_num = (int)simu_end_index_[0]; }
		else { end_node_num = (int)all_node_num_ - 1; }
	}
	else if (simulation_num_ == (int)simu_end_index_.size())
	{
		start_node_num = (int)simu_end_index_[simulation_num_ - 1] + 1;
		end_node_num = (int)all_node_num_ - 1;
	}
	else
	{
		start_node_num = (int)simu_end_index_[simulation_num_ - 1] + 1;
		end_node_num = (int)simu_end_index_[simulation_num_];
	}

	DrawFormatString(kTextLeftX, kGuiTopPosY + 30, kTextColor, "�\���m�[�h : %d (%d�`%d)", display_node_num_, start_node_num, end_node_num, all_node_num_ - 1);

	DrawFormatString(kTextLeftX, kGuiTopPosY + 50, kTextColor, "�S�m�[�h : %d ", all_node_num_ - 1);

	DrawFormatString(kTextLeftX, kGuiTopPosY + 70, kTextColor, do_auto_animation_ == true ? "�����Đ� : �Đ�/���x%d" : "�����Đ� : ��~", animation_speed_);

	DrawFormatString(kTextLeftX, kGuiTopPosY + 150, kTextColor, "�A�j���[�V������\n     ���x�ύX");
}


void DisplayNodeSwitchGui::MoveMostPrevNode()
{
	//���
	size_t candidate = 0;

	for (size_t i = 0; i < simu_end_index_.size(); i++)
	{
		if (simu_end_index_[i] < display_node_num_)
		{
			candidate = simu_end_index_[i] + 1;
		}
		else
		{
			break;
		}
	}

	display_node_num_ = candidate;
}


void DisplayNodeSwitchGui::MovePrevNode()
{
	for (size_t i = 0; i < simu_end_index_.size(); i++)
	{
		if (simu_end_index_[i] + 1 == display_node_num_)
		{
			return;
		}
	}

	--display_node_num_;

	display_node_num_ = static_cast<size_t>((std::max)(static_cast<int>(display_node_num_), 0));
}


void DisplayNodeSwitchGui::MoveMostNextNode()
{
	//���
	size_t candidate = all_node_num_ - 1;

	for (size_t i = 0; i < simu_end_index_.size(); i++)
	{
		if (simu_end_index_[i] >= display_node_num_)
		{
			candidate = simu_end_index_[i];
			break;
		}
	}

	display_node_num_ = candidate;
}


void DisplayNodeSwitchGui::MoveNextNode()
{
	for (size_t i = 0; i < simu_end_index_.size(); i++)
	{
		if (simu_end_index_[i] == display_node_num_)
		{
			return;
		}
	}

	++display_node_num_;

	display_node_num_ = static_cast<size_t>((std::min)(static_cast<int>(display_node_num_), static_cast<int>(all_node_num_ - 1)));
}


void DisplayNodeSwitchGui::MovePrevSimulation()
{
	//�O�̃V�~�����[�V�����ֈړ�����
	--simulation_num_;

	simulation_num_ = static_cast<size_t>((std::max)(static_cast<int>(simulation_num_), 0));

	//�m�[�h�����̃V�~�����[�V�����̍ŏ��̃m�[�h�Ɉړ�����
	if (simulation_num_ == 0)
	{
		display_node_num_ = 0;
	}
	else
	{
		display_node_num_ = simu_end_index_[simulation_num_ - 1] + 1;
	}
}


void DisplayNodeSwitchGui::MoveNextSimulation()
{
	//���̃V�~�����[�V�����ֈړ�����
	++simulation_num_;


	simulation_num_ = static_cast<size_t>((std::min)(static_cast<int>(simulation_num_), GetAllSimulationNum() - 1));


	//�m�[�h�����̃V�~�����[�V�����̍ŏ��̃m�[�h�Ɉړ�����
	if (simulation_num_ == 0)
	{
		display_node_num_ = 0;
	}
	else
	{
		display_node_num_ = simu_end_index_[simulation_num_ - 1] + 1;
	}
}


int DisplayNodeSwitchGui::GetAllSimulationNum() const
{
	int all_simu_num = 1;

	if (!simu_end_index_.empty())
	{
		if (simu_end_index_.back() == all_node_num_ - 1) { all_simu_num = (int)simu_end_index_.size(); }
		else { all_simu_num = (int)simu_end_index_.size() + 1; }
	}

	return all_simu_num;
}
