//! @file discrete_leg_pos.h
//! @brief 離散化された脚位置を表す列挙体

#ifndef DESIGNLAB_DISCRETE_LEG_POS_H_
#define DESIGNLAB_DISCRETE_LEG_POS_H_


//! @enum DiscreteLegPos
//! @brief 離散化された脚位置を表すenum
//! @n 先行研究では 1〜7のint型の数値で表現されているが，可読性を上げるためにenumにした
//! @n 処理の速度は変わらんはず
//! @n 離散化された脚位置は 3bit (0 〜 7)の範囲で表現されるため，これを拡張する場合，
//! @n leg stateを表す変数の型を変更する必要がある
enum class DiscreteLegPos : int
{
	kLowerBack = 1,		//!< 現在の位置より後方かつ下方にある
	kBack,				//!< 現在の位置より後方にある
	kUpperBack,			//!< 現在の位置より後方かつ上方にある
	kCenter,			//!< 現在の位置にある
	kLowerFront,		//!< 現在の位置より前方かつ下方にある
	kFront,				//!< 現在の位置より前方にある
	kUpperFront,		//!< 現在の位置より前方かつ上方にある
};


#endif // DESIGNLAB_DISCRETE_LEG_POS_H_