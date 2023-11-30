#include "dxlib_gui_terminal.h"

#include <DxLib.h>


DxlibGuiTerminal::DxlibGuiTerminal(std::vector<std::shared_ptr<IDxlibGui>> gui_list) : 
	kTerminalWidth(static_cast<int>(gui_list.size() + 2) * kButtonSize)
{
	gui_list_ = gui_list;

	//gui�̐������{�^�����쐬
	for (int i = 0; i < gui_list_.size(); ++i)
	{
		std::string class_name = typeid(*gui_list_[i].get()).name();

		//typeid�̕Ԃ�l��"class DxlibGui�`"�Ƃ���������ɂȂ��Ă���̂ŁC�ז��ȕ������폜����D
		const std::vector<std::string> unnecessary_words = { "class ", "DxlibGui", " ", "const", "*", "&" };

		for (const auto& word : unnecessary_words)
		{
			//���������������word���܂܂�Ă��Ȃ��ꍇ�͉������Ȃ�
			if (class_name.find(word) == std::string::npos) { continue; }

			//�������word���܂܂�Ă���ꍇ�͍폜
			class_name.erase(class_name.find(word), word.size());
		}

		//class_name��7�������Ƃɉ��s
		for (int j = 7; j < class_name.size(); j += 7)
		{
			class_name.insert(j, "\n");
			++j;
		}

		button_list_.push_back(
			std::make_shared<SimpleButton>(class_name,
				kLeftTopX + kButtonSize + i * 100, kLeftTopY + kTerminalHeight / 2, kButtonSize, kButtonSize)
		);

		button_list_.back()->SetActivateFunction([this, i]() { gui_list_[i]->SetVisible(true); });

		button_list_.back()->SetVisible(false);
	}
}

void DxlibGuiTerminal::Update()
{
	//�e�{�^���̍X�V
	for (auto& button : button_list_)
	{
		button->Update();
	}
}

void DxlibGuiTerminal::Draw() const
{
	if (is_closed_) 
	{
		DrawClosedTerminal();
	}
	else 
	{
		DrawTerminal();

		//�{�^���̕`��
		for (const auto& button : button_list_)
		{
			button->Draw();
		}

		DrawButtonGuard();
	}
}

void DxlibGuiTerminal::SetVisible([[maybe_unused]] bool visible)
{
	//�����Ȃ��D�Ȃ̂ŉ������Ȃ��D
}

bool DxlibGuiTerminal::IsVisible() const
{
	//�����Ȃ��D�Ȃ̂ŏ��true�D
	return true;
}

void DxlibGuiTerminal::ClickedAction(const int cursor_x, const int cursor_y, 
	const int left_pushing_count, const int middle_pushing_count, const int right_pushing_count)
{
	bool is_clicked = false;

	//�e�{�^���̏���
	for (auto& button : button_list_)
	{
		if (button->CursorOnGui(cursor_x, cursor_y))
		{
			button->ClickedAction(cursor_x, cursor_y, left_pushing_count, middle_pushing_count, right_pushing_count);
			is_clicked = true;
			break;
		}
	}

	//�{�^���������ꂽ�ꍇ�͏I��
	if (is_clicked) { return; }

	//�^�[�~�i���̏���
	if (left_pushing_count == 1 && is_closed_)
	{
		is_closed_ = false;

		//�{�^����\��
		for (auto& button : button_list_)
		{
			button->SetVisible(true);
		}
	}
	else if (left_pushing_count == 1 && !is_closed_)
	{
		is_closed_ = true;

		//�{�^�����\��
		for (auto& button : button_list_)
		{
			button->SetVisible(false);
		}
	}
}

bool DxlibGuiTerminal::CursorOnGui(int cursor_x, int cursor_y) const noexcept
{
	if (is_closed_) 
	{
		return cursor_x >= kLeftTopX && cursor_x <= kLeftTopX + kClosedTerminalWidth &&
			cursor_y >= kLeftTopY && cursor_y <= kLeftTopY + kTerminalHeight;
	}
	else 
	{
		return cursor_x >= kLeftTopX && cursor_x <= kLeftTopX + kTerminalWidth &&
			cursor_y >= kLeftTopY && cursor_y <= kLeftTopY + kTerminalHeight;
	}
}

void DxlibGuiTerminal::DrawClosedTerminal() const
{
	const int closed_box_width = kClosedTerminalWidth - 20;	//�l�p�`�����̕�

	const unsigned int base_color = GetColor(255, 255, 255);
	const unsigned int frame_color = GetColor(30, 30, 30);
	const unsigned int alpha = 200;

	const int frame_width = 1;

	SetDrawBlendMode(DX_BLENDMODE_ALPHA, alpha);

	DrawBox(kLeftTopX - frame_width, kLeftTopY - frame_width,
		kLeftTopX + closed_box_width, kLeftTopY + kTerminalHeight + frame_width, frame_color, TRUE);
	DrawTriangleAA(
		kLeftTopX + closed_box_width, kLeftTopY - frame_width,
		kLeftTopX + closed_box_width, kLeftTopY + kTerminalHeight + frame_width,
		kLeftTopX + kClosedTerminalWidth + frame_width, kLeftTopY + kTerminalHeight / 2,
		frame_color, TRUE
	);

	DrawBox(kLeftTopX, kLeftTopY, kLeftTopX + closed_box_width, kLeftTopY + kTerminalHeight, base_color, TRUE);
	DrawTriangleAA(kLeftTopX + closed_box_width, kLeftTopY, kLeftTopX + closed_box_width, kLeftTopY + kTerminalHeight,
		kLeftTopX + kClosedTerminalWidth, kLeftTopY + kTerminalHeight / 2, base_color, TRUE);

	SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);
}

void DxlibGuiTerminal::DrawTerminal() const
{
	const int closed_box_width = kTerminalWidth - 20;	//�l�p�`�����̕�

	const unsigned int base_color = GetColor(255, 255, 255);
	const unsigned int frame_color = GetColor(30, 30, 30);
	const unsigned int alpha = 200;

	const int frame_width = 1;

	SetDrawBlendMode(DX_BLENDMODE_ALPHA, alpha);

	DrawBox(kLeftTopX - frame_width, kLeftTopY - frame_width,
		kLeftTopX + closed_box_width, kLeftTopY + kTerminalHeight + frame_width, frame_color, TRUE);
	DrawTriangleAA(
		static_cast<float>(kLeftTopX + closed_box_width), static_cast<float>(kLeftTopY - frame_width),
		static_cast<float>(kLeftTopX + closed_box_width), static_cast<float>(kLeftTopY + kTerminalHeight + frame_width),
		static_cast<float>(kLeftTopX + kTerminalWidth + frame_width), static_cast<float>(kLeftTopY + kTerminalHeight / 2),
		frame_color, TRUE
	);

	DrawBox(kLeftTopX, kLeftTopY, kLeftTopX + closed_box_width, kLeftTopY + kTerminalHeight, base_color, TRUE);
	DrawTriangleAA(
		static_cast<float>(kLeftTopX + closed_box_width), static_cast<float>(kLeftTopY),
		static_cast<float>(kLeftTopX + closed_box_width), static_cast<float>(kLeftTopY + kTerminalHeight),
		static_cast<float>(kLeftTopX + kTerminalWidth), static_cast<float>(kLeftTopY + kTerminalHeight / 2),
		base_color, TRUE
	);

	SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);
}

void DxlibGuiTerminal::DrawButtonGuard() const
{
	for (int i = 0; i < button_list_.size(); ++i)
	{
		if (gui_list_[i]->IsVisible())
		{
			const unsigned int color = GetColor(45, 45, 45);
			const int alpha = 200;

			SetDrawBlendMode(DX_BLENDMODE_ALPHA, alpha);

			DrawBox(button_list_[i]->GetPosMiddleX() - kButtonSize / 2, button_list_[i]->GetPosMiddleY() - kButtonSize / 2,
				button_list_[i]->GetPosMiddleX() + kButtonSize / 2, button_list_[i]->GetPosMiddleY() + kButtonSize / 2, color, TRUE);

			SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);
		}
	}

}