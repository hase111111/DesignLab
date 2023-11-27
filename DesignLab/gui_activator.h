﻿//! @file gui_activator.h
//! @brief クリック判定を行うクラス．

#ifndef DESIGNLAB_GUI_ACTIVATOR_H_
#define DESIGNLAB_GUI_ACTIVATOR_H_

#include <map>
#include <memory>

#include "interface_dxlib_clickable.h"
#include "mouse.h"


//! @class GuiActivator
//! @brief クリック判定を行うクラス．
//! @n 一度のクリックで複数のGUIが反応することを防ぐために，優先度を設定する．
//! @n 優先度が高いものから順にクリック判定を行い，クリックされたらそのGUIのActivate関数を実行する．
class GuiActivator final
{
public:

	//! @brief GUIを登録する．
	//! @param[in] clickable_ptr クリック可能なGUIのポインタ．
	//! @param[in] priority クリック入力の優先度．これが高いほど優先的にクリックの判定がされる．
	void Register(const std::shared_ptr<IDxlibClickable> clickable_ptr, int priority);

	//! @brief クリック判定を行う．
	//! @n 優先度の高いものから順にクリック判定を行う．
	//! @n クリックされたら，そのGUIのActivate関数を実行する．
	//! @n 一度Activateされたら，それ以降のGUIのActivate関数は実行されない．
	//! @param[in] mouse_ptr マウスの状態を管理するクラスのポインタ．
	void Activate(const std::shared_ptr<const Mouse> mouse_ptr);

private:
	
	//! @struct Priority
	struct Priority final
	{
		constexpr Priority(const int p, const int o) noexcept : priority(p), order(o) {}

		//! mapでの優先度の比較に使用する．
		//! @n 優先度を比較し，優先度が同じならば追加された順番で比較する．
		bool operator<(const Priority& other) const noexcept
		{
			if (priority == other.priority)
			{
				return order < other.order;
			}
			else
			{
				return priority < other.priority;
			}
		}

		int priority{ 0 };		//!< 優先度
		int order{ 0 };			//!< 追加された順番
	};


	//! @brief クリック可能なGUIのポインタを優先度順に格納する．
	std::map<Priority, std::shared_ptr<IDxlibClickable>> clickable_ptrs_;
};


#endif	// DESIGNLAB_GUI_ACTIVATOR_H_