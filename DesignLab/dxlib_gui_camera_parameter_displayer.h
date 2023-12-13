//! @file dxlib_gui_camera_displayer.h
//! @brief �J�����̏���\������GUI�̏����E�`����s���N���X�D


#ifndef DESIGNLAB_DXLIB_GUI_CAMERA_DISPLAYER_H_
#define DESIGNLAB_DXLIB_GUI_CAMERA_DISPLAYER_H_

#include <array>
#include <memory>
#include <vector>

#include "dxlib_camera.h"
#include "interface_dxlib_clickable.h"
#include "interface_dxlib_draggable.h"
#include "interface_dxlib_gui.h"
#include "simple_button.h"


//! @class DxlibGuiCameraParameterDisplayer
//! @brief �J�����̏���\������GUI�̏����E�`����s���N���X�D
class DxlibGuiCameraParameterDisplayer final : public IDxlibGui, public IDxlibClickable, public IDxlibDraggable
{
public:

	DxlibGuiCameraParameterDisplayer() = delete;	//!< �f�t�H���g�R���X�g���N�^�͐����ł��Ȃ��D

	//! @brief �R���X�g���N�^��Window�̃T�C�Y�ƁC�J�����̊Ǘ����s���N���X���󂯎��D
	//! @param[in] window_x �E�B���h�E�̉����D
	//! @param[in] window_y �E�B���h�E�̏c���D
	//! @param[in] camera �J�����̊Ǘ����s���N���X�D
	DxlibGuiCameraParameterDisplayer(
		int window_x,
		int window_y,
		const std::shared_ptr<DxlibCamera> camera_ptr
	);

	//! @brief GUI�̈ʒu��ݒ肷��D
	//! @n Dxlib�̉�ʂ̍��W�͍�������_�Ƃ��C�E���ɍs���قǒl���傫���Ȃ�D
	//! @n ��������x���C�c������y�����Ƃ�D
	//! @param[in] pos_x GUI��x���W�D
	//! @param[in] pos_y GUI��y���W�D
	//! @param[in] option GUI�̂ǂ̒n�_���N�_�ɍ��W��ݒ肷�邩���w�肷��Ddefault�ł͍�����N�_�Ƃ���D
	//! @param[in] this_is_first_time ���̌Ăяo�������߂Ă��ǂ������w�肷��Ddefault�ł�false�D
	//! true���w�肷��ƁCGUI�̈ʒu��ݒ肷�邾���łȂ��CGUI�̏����ʒu���X�V����D
	void SetPos(int pos_x, int pos_y, unsigned int option = ::designlab::kDxlibGuiAnchorLeftTop, bool this_is_first_time = false);

	void Update() override;

	void Draw() const override;

	void SetVisible(bool visible) override;

	bool IsVisible() const override { return visible_; }

	void ClickedAction(int cursor_x, int cursor_y, int left_pushing_count, int middle_pushing_count, int right_pushing_count) override;

	bool CursorOnGui(int cursor_x, int cursor_y) const noexcept override;

	bool IsDraggable(int cursor_x, int cursor_y) const override;

	bool IsDragged() const override { return is_dragging_; };

	void SetDragged(const bool is_dragged) override { is_dragging_ = is_dragged; };

	void DraggedAction(int cursor_dif_x, int cursor_dif_y, unsigned int mouse_key_bit) override;

private:

	static constexpr int kWidth{ 470 };			//!< GUI�̕��D
	static constexpr int kHeight{ 340 };		//!< GUI�̍����D
	static constexpr int kTitleBarHeight{ 32 };	//!< �^�C�g���o�[�̍����D

	void DrawBackground() const;

	void DrawCameraParameter() const;

	bool IsInWindow() const;

	int gui_left_pos_x_{ 0 };	//!< GUI�̍����X���W�D
	int gui_top_pos_y_{ 0 };	//!< GUI�̍����Y���W�D

	int set_pos_x_{ 0 };	//!< GUI�̍����X���W(�����ʒu)
	int set_pos_y_{ 0 };	//!< GUI�̍����Y���W(�����ʒu)

	const int window_x_;	//!< �E�B���h�E��X���W�D
	const int window_y_;	//!< �E�B���h�E��Y���W�D

	bool is_dragging_{ false };	//!< �h���b�O�����ǂ����D
	bool visible_{ true };		//!< �{�^���̕\�����s�����ǂ����D

	const std::shared_ptr<const DxlibCamera> camera_ptr_;	//!< �J�����̃|�C���^�D

	std::vector<std::unique_ptr<SimpleButton>> button_;	//!< �{�^���D

	const int kFontSize{ 16 };		//!< �t�H���g�̃T�C�Y�D
	const std::string kFontPath{ "font/Yu_Gothic_UI.dft" };	//!< �t�H���g�ւ̃p�X�D
};

#endif	//DESIGNLAB_DXLIB_GUI_CAMERA_DISPLAYER_H_