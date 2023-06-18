//! @file Define.h
//! @brief プロジェクト全体の定数
//! @author 長谷川

#pragma once

//! @class Define
//! @brief
//! @details Effective C++ という本 (私が持っているのはかなり古い版のものなので正直正しいのかはよくわからないけど) によると，<br>
//! C++においてはあまり #defineを使用するべきではないようである．結構いろんなサイトでも同じことが言われている．<br> 
//! https://qiita.com/jonichonpa/items/595ed7914751787ebaee <br>
//! https://myon.info/blog/2015/12/18/avoid-defining-macros/ <br> <br>
//! const statisな定数．inline 関数．constexpr定数を使うべき．<br> <br>
//! 以下参考資料．<br> クラスメンバに constexpr static 変数はおすすめしない ― 現象と対策<br>
//! https://qiita.com/Nabetani/items/d8a3ebccaef03cd18d81
//! @author 長谷川
class Define final
{
public:

	const static int SIMURATE_NUM;	//!< 連続でシミュレーションを行う回数

	const static char GRAPH_SEARCH_DEPTH;	//!< グラフ探索の探索深さ

	const static int GATE_PATTERN_GENERATE_NUM;	//!< 1シミュレーション当たりの最大歩容生成回数

	const static int GOAL_TAPE;	//!< 直進のときに、Y方向にこの値だけ進めたら1シミュレーション終了

	const static bool FLAG_GRAPHIC_AVAILABLE;	//!< グラフィックを表示するときはtrue

	constexpr static float ALLOWABLE_ERROR = 0.0001f;	//!< これ以上小さい値は0とみなす．allowable error，許容誤差のこと

	const static float MY_PI;	//!< 円周率

	const static bool FLAG_DO_PRUNING;	//!< グラフ探索において枝刈りを行うときはtrue

private:

	Define() = delete;
	Define(const Define& _other) = delete;
};
