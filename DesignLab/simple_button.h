﻿//! @file button_controller.h
//! @brief ボタンの処理，描画を管理するクラス．


#ifndef DESIGNLAB_BUTTON_CONTROLLER_H_
#define DESIGNLAB_BUTTON_CONTROLLER_H_

#include "interface_dxlib_clickable.h"
#include "interface_dxlib_gui.h"

#include <functional>
#include <string>
#include <vector>

//! @class SimpleButton
//! @brief ボタンの処理，描画を管理するクラス．
class SimpleButton final : public IDxlibGui, public IDxlibClickable
{
public:

	SimpleButton(const std::string& text, int x_pos, int y_pos, int x_size, int y_size, bool fit_size = false);
	~SimpleButton() = default;

	//! @brief ボタンがクリックされたときに実行される関数を設定する．
	//! @param[in] func ボタンがクリックされたときに実行される関数．
	inline void SetActivateFunction(const std::function<void()>& func) { click_function_ = func; }

	void SetPos(int pos_x, int pos_y, unsigned int option = designlab::kDxlibGuiAnchorLeftTop);

	constexpr int GetPosMiddleX() const noexcept { return pos_middle_x; }
	constexpr int GetPosMiddleY() const noexcept { return pos_middle_y; }

	void Update() override;

	void Draw() const override;

	void SetVisible(bool visible) override { visible_ = visible; }

	bool IsVisible() const override { return visible_; }

	void ClickedAction(int cursor_x, int cursor_y, int left_pushing_count, int middle_pushing_count, int right_pushing_count) override;

	bool CursorOnGui(int cursor_x, int cursor_y) const noexcept override;

private:

	int GetFitButtonSizeX(int now_size_x) const noexcept;
	int GetFitButtonSizeY(int now_size_y) const noexcept;

	const int kFontSize{ 16 };		//!< フォントのサイズ
	const std::string kFontPath{ "font/Yu_Gothic_UI.dft" };	//!< フォントへのパス

	std::vector<std::string> text_;		//!< ボタンに書かれているテキスト，複数行に対応．
	int font_handle_;	//!< フォントのハンドル

	int now_color_blue_{ 0 };	//!< クリック時，あるいはマウスが乗ったときにボタンの色を変える．
	int target_color_blue_{ 0 };	//!< そのための色の目標値と現在値．

	bool visible_{ true };	//!< ボタンの表示を行うかどうか．

	int pos_middle_x, pos_middle_y;		//!< ボタンの座標，中心座標．
	const int kSizeX, kSizeY;	//!< ボタンの横幅と縦幅

	std::function<void()> click_function_;	//!< ボタンがクリックされたときに実行される関数．
};


#endif // DESIGNLAB_BUTTON_CONTROLLER_H_