﻿//! @file camera_gui.h
//! @brief カメラの操作・管理を行うGuiの処理，表示を行うクラス


#ifndef DESIGNLAB_CAMERA_GUI_H_
#define DESIGNLAB_CAMERA_GUI_H_


#include <map>
#include <memory>

#include "dxlib_camera.h"
#include "designlab_vector3.h"
#include "interface_dxlib_clickable.h"
#include "interface_dxlib_draggable.h"
#include "interface_dxlib_gui.h"
#include "simple_button.h"


//! @class CameraGui
//! @brief カメラの操作・管理を行うGUIを行うクラス．
class CameraGui final : public IDxlibGui, public IDxlibClickable, public IDxlibDraggable
{
public:

	CameraGui() = delete;

	//! @brief コンストラクタでカメラの管理を行うクラスを受け取る．
	CameraGui(const std::shared_ptr<DxlibCamera> camera);

	//! @brief GUIの位置を設定する．
	//! @n Dxlibの画面の座標は左上を原点とし，右下に行くほど値が大きくなる．
	//! @n 横方向にx軸，縦方向にy軸をとる．
	//! @param[in] pos_x GUIのx座標．
	//! @param[in] pos_y GUIのy座標．
	//! @param[in] option GUIのどの地点を起点に座標を設定するかを指定する．defaultでは左上を起点とする．
	void SetPos(int pos_x, int pos_y, unsigned int option = ::designlab::kOptionLeftTop);

	//! @brief カメラが注視するロボットの座標を設定する．
	//! @param[in] pos ロボットの座標．
	void SetHexapodPos(const designlab::Vector3& pos);

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

	static constexpr int kWidth{ 235 };			//!< GUIの横幅．
	static constexpr int kHeight{ 410 };		//!< GUIの縦幅．
	static constexpr int kTitleBarHeight{ 32 };	//!< タイトルバーの高さ．

	//! @brief GUIの背景を描画する
	void DrawBackground() const;

	//! @brief GUIの文字を描画する
	void DrawString() const;


	int gui_left_pos_x_{ 0 };	//!< GUIの左端の位置
	int gui_top_pos_y_{ 0 };	//!< GUIの上端の位置
	bool visible_{ true };		//!< GUIが表示されているかどうかのフラグ
	bool is_dragging_{ false };	//!< ドラッグ中かどうかのフラグ

	const std::shared_ptr<DxlibCamera> camera_;			//!< カメラの管理を行うクラス
	std::vector<std::unique_ptr<SimpleButton>> button_;	//!< ボタンのリスト

	const int kFontSize{ 16 };		//!< フォントのサイズ
	const std::string kFontPath{ "font/Yu_Gothic_UI.dft" };	//!< フォントへのパス
	int font_handle_;	//!< フォントのハンドル
};


#endif	// DESIGNLAB_CAMERA_GUI_H_