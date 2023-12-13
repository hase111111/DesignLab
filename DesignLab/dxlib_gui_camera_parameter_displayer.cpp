#include "dxlib_gui_camera_parameter_displayer.h"

#include <magic_enum.hpp>

#include "designlab_rot_converter.h"
#include "designlab_string_util.h"
#include "font_loader.h"


namespace dl = ::designlab;
namespace dlm = ::designlab::math_util;
namespace dlsu = ::designlab::string_util;


DxlibGuiCameraParameterDisplayer::DxlibGuiCameraParameterDisplayer(
	const int window_x,
	const int window_y,
	const std::shared_ptr<DxlibCamera> camera_ptr
) :
	window_x_(window_x),
	window_y_(window_y),
	camera_ptr_(camera_ptr)
{
	const int close_button_size = 28;
	const int close_button_x = gui_left_pos_x_ + kWidth - close_button_size / 2 - 2;
	const int close_button_y = gui_top_pos_y_ + close_button_size / 2 + 2;

	button_.push_back(std::make_unique<SimpleButton>("�~", close_button_x, close_button_y, close_button_size, close_button_size));
	button_.back()->SetActivateFunction([this]() { SetVisible(false); });
}

void DxlibGuiCameraParameterDisplayer::SetPos(const int pos_x, const int pos_y, unsigned int option, const bool this_is_first_time)
{
	const int past_x = gui_left_pos_x_;
	const int past_y = gui_top_pos_y_;

	if (option & dl::kDxlibGuiAnchorLeft) { gui_left_pos_x_ = pos_x; }
	else if (option & dl::kDxlibGuiAnchorMidleX) { gui_left_pos_x_ = pos_x - kWidth / 2; }
	else if (option & dl::kDxlibGuiAnchorRight) { gui_left_pos_x_ = pos_x - kWidth; }

	if (option & dl::kDxlibGuiAnchorTop) { gui_top_pos_y_ = pos_y; }
	else if (option & dl::kDxlibGuiAnchorMidleY) { gui_top_pos_y_ = pos_y - kHeight / 2; }
	else if (option & dl::kDxlibGuiAnchorBottom) { gui_top_pos_y_ = pos_y - kHeight; }

	const int diff_x = gui_left_pos_x_ - past_x;
	const int diff_y = gui_top_pos_y_ - past_y;

	for (auto& button : button_)
	{
		button->SetPos(button->GetPosMiddleX() + diff_x, button->GetPosMiddleY() + diff_y, dl::kDxlibGuiAnchorMidleXMidleY);
	}

	if (this_is_first_time)
	{
		set_pos_x_ = gui_left_pos_x_;
		set_pos_y_ = gui_top_pos_y_;
	}
}

void DxlibGuiCameraParameterDisplayer::Update()
{
	// �{�^�����X�V����
	for (auto& i : button_)
	{
		i->Update();
	}

	if (!IsInWindow())
	{
		SetVisible(false);
	}
}

void DxlibGuiCameraParameterDisplayer::Draw() const
{
	// �g
	DrawBackground();

	// �{�^����`�悷��
	for (const auto& i : button_)
	{
		i->Draw();
	}

	DrawCameraParameter();
}

void DxlibGuiCameraParameterDisplayer::SetVisible(const bool visible)
{
	visible_ = visible;

	for (auto& i : button_)
	{
		i->SetVisible(visible);
	}

	if (visible_)
	{
		SetPos(set_pos_x_, set_pos_y_, dl::kDxlibGuiAnchorLeftTop);
	}
}

void DxlibGuiCameraParameterDisplayer::ClickedAction(const int cursor_x, const int cursor_y,
	const int left_pushing_count, [[maybe_unused]] const int middle_pushing_count, [[maybe_unused]] const int right_pushing_count)
{
	// �{�^�����X�V����
	for (auto& i : button_)
	{
		if (i->CursorOnGui(cursor_x, cursor_y))
		{
			i->ClickedAction(cursor_x, cursor_y, left_pushing_count, middle_pushing_count, right_pushing_count);
			break;
		}
	}
}

bool DxlibGuiCameraParameterDisplayer::CursorOnGui(const int cursor_x, const int cursor_y) const noexcept
{
	if (!IsVisible()) { return false; }

	return (gui_left_pos_x_ < cursor_x && cursor_x < gui_left_pos_x_ + kWidth) &&
		(gui_top_pos_y_ < cursor_y && cursor_y < gui_top_pos_y_ + kHeight);
}

bool DxlibGuiCameraParameterDisplayer::IsDraggable(const int cursor_x, const int cursor_y) const
{
	if (!IsVisible()) { return false; }

	return (gui_left_pos_x_ < cursor_x && cursor_x < gui_left_pos_x_ + kWidth) &&
		(gui_top_pos_y_ < cursor_y && cursor_y < gui_top_pos_y_ + kHeight);
}

void DxlibGuiCameraParameterDisplayer::DraggedAction(const int cursor_dif_x, const int cursor_dif_y, [[maybe_unused]] const unsigned int mouse_key_bit)
{
	SetPos(gui_left_pos_x_ + cursor_dif_x, gui_top_pos_y_ + cursor_dif_y, dl::kDxlibGuiAnchorLeftTop);
}

void DxlibGuiCameraParameterDisplayer::DrawBackground() const
{
	const unsigned int base_color = GetColor(255, 255, 255);
	const unsigned int frame_color = GetColor(30, 30, 30);
	const unsigned int alpha = 200;

	const int frame_width = 1;

	SetDrawBlendMode(DX_BLENDMODE_ALPHA, alpha);

	DrawBox(gui_left_pos_x_ - frame_width, gui_top_pos_y_ - frame_width,
		gui_left_pos_x_ + kWidth + frame_width, gui_top_pos_y_ + kHeight + frame_width, frame_color, TRUE);
	DrawBox(gui_left_pos_x_, gui_top_pos_y_, gui_left_pos_x_ + kWidth, gui_top_pos_y_ + kHeight, base_color, TRUE);

	DrawBox(gui_left_pos_x_, gui_top_pos_y_, gui_left_pos_x_ + kWidth, gui_top_pos_y_ + kTitleBarHeight, base_color, TRUE);
	DrawBox(gui_left_pos_x_ - frame_width, gui_top_pos_y_ - frame_width,
		gui_left_pos_x_ + kWidth + frame_width, gui_top_pos_y_ + kTitleBarHeight + frame_width, frame_color, FALSE);


	const int text_pos_x = gui_left_pos_x_ + 10;
	const int text_pos_y = gui_top_pos_y_ + 10;
	const int font_handle = FontLoader::GetIns()->GetFontHandle("font/Yu_Gothic_UI.dft");
	const unsigned int text_color = GetColor(10, 10, 10);
	DrawFormatStringToHandle(text_pos_x, text_pos_y, text_color, font_handle, "CameraParameterDisplayer");

	SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);
}

void DxlibGuiCameraParameterDisplayer::DrawCameraParameter() const
{
	const unsigned int text_color = GetColor(10, 10, 10);

	const int font_handle = FontLoader::GetIns()->GetFontHandle("font/Yu_Gothic_UI.dft");
	const int text_pos_x = gui_left_pos_x_ + 10;
	const int text_pos_y_min = gui_top_pos_y_ + kTitleBarHeight + 10;
	const int text_interval_y = kFontSize + 4;

	int text_line = 0;

	DrawFormatStringToHandle(text_pos_x, text_pos_y_min + text_interval_y * (text_line++), text_color, font_handle, "�J�����̌���(���K���N�H�[�^�j�I��)");
	DrawFormatStringToHandle(text_pos_x, text_pos_y_min + text_interval_y * (text_line++), text_color, font_handle, "�@(w:%5.3f,x:%5.3f,y:%5.3f,z:%5.3f)",
		camera_ptr_->GetNowCameraQuat().w, camera_ptr_->GetNowCameraQuat().v.x, camera_ptr_->GetNowCameraQuat().v.y, camera_ptr_->GetNowCameraQuat().v.z);

	dl::EulerXYZ euler_xyz = dl::ToEulerXYZ(camera_ptr_->GetNowCameraQuat());
	DrawFormatStringToHandle(text_pos_x, text_pos_y_min + text_interval_y * (text_line++), text_color, font_handle, "�J�����̌���(�I�C���[�p)");
	DrawFormatStringToHandle(text_pos_x, text_pos_y_min + text_interval_y * (text_line++), text_color, font_handle, "�@(roll:%5.3f[deg],pitch:%5.3f[deg],yaw:%5.3f[deg])",
		dlm::ConvertRadToDeg(euler_xyz.x_angle), dlm::ConvertRadToDeg(euler_xyz.y_angle), dlm::ConvertRadToDeg(euler_xyz.z_angle));

	text_line++;
	DrawFormatStringToHandle(text_pos_x, text_pos_y_min + text_interval_y * (text_line++), text_color, font_handle, "�J�����̕\�����[�h");
	DrawFormatStringToHandle(text_pos_x, text_pos_y_min + text_interval_y * (text_line++), text_color, font_handle, "�@%s", 
		static_cast<std::string>(magic_enum::enum_name(camera_ptr_->GetCameraViewMode())).c_str());

	DrawFormatStringToHandle(text_pos_x, text_pos_y_min + text_interval_y * (text_line++), text_color, font_handle, "�J�����̒����_�̍��W");
	DrawFormatStringToHandle(text_pos_x, text_pos_y_min + text_interval_y * (text_line++), text_color, font_handle, "�@(x:%5.3f [mm],y:%5.3f [mm],z:%5.3f [mm])",
		camera_ptr_->GetNowTargetPos().x, camera_ptr_->GetNowTargetPos().y, camera_ptr_->GetNowTargetPos().z);

	DrawFormatStringToHandle(text_pos_x, text_pos_y_min + text_interval_y * (text_line++), text_color, font_handle, "�J�����ƑΏۂƂ̋���");
	DrawFormatStringToHandle(text_pos_x, text_pos_y_min + text_interval_y * (text_line++), text_color, font_handle, "�@%5.3f [mm]", camera_ptr_->GetNowCameraToTargetLength());

	text_line++;
	DrawFormatStringToHandle(text_pos_x, text_pos_y_min + text_interval_y * (text_line++), text_color, font_handle, "�J�����̈ʒu");
	DrawFormatStringToHandle(text_pos_x, text_pos_y_min + text_interval_y * (text_line++), text_color, font_handle, "�@(x:%5.3f [mm],y:%5.3f [mm],z:%5.3f [mm])",
		camera_ptr_->GetNowCameraPos().x, camera_ptr_->GetNowCameraPos().y, camera_ptr_->GetNowCameraPos().z);
}

bool DxlibGuiCameraParameterDisplayer::IsInWindow() const
{
	return gui_left_pos_x_ < window_x_ && gui_top_pos_y_ < window_y_ &&
		0 < gui_left_pos_x_ + kWidth && 0 < gui_top_pos_y_ + kHeight;
}