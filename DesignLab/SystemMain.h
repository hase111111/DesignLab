#pragma once
#include "MapState.h"
#include "GraphicDataBroker.h"
#include "GraphicSystem.h"
#include "NodeValidityChecker.h"
#include "Target.h"


class SystemMain final
{
public:
	SystemMain();
	~SystemMain() = default;

	//! @brief いままでint mainで行われた処理をまとめたもの．目標地点へ着くか，歩容計画に失敗した場合に，シミュレーションを終える．規定の回数シミュレーションしたら終了する．
	void main();

private:
	MapState m_Map;
	STarget m_target;
	GraphicDataBroker m_Broker;
	GraphicSystem m_Graphic;
	NodeValidityChecker m_Checker;
};


//! @file SystemMain.h
//! @brief このプログラムの処理をまとめたもの．処理の内容を大きく変えたい場合はint main()から，全く別のクラスを呼べばよい．
//! @author 長谷川

//! @class SystemMain
//! @brief 中〜大規模な設計において，int mainになんでも詰め込むわけにはいかないため，このクラスにまとめる．
//! @details 処理の内容を書き換えるときには，int mainから呼ぶクラスを変えるだけでいい．
//! @author 長谷川

