//! @file LegState.h
//! @brief このプログラムでは脚状態をint型(32bit)の情報で表す．そのデータを処理するための関数をまとめたもの
//! @details 脚状態って言ってるけど重心タイプも入ってる．C++だからint型は32bit．<br>
//! 1脚の脚状態を4bitで表す 最上位0:遊脚,1:接地．　残り3bitで離散化した脚位置．<br>
//! 離散化した脚位置は基準位置を4として，それより前にあるなら4より大きい数字，後ろにあるならば4より小さい数字．<br> <br> 
//! [離散化した脚位置] <br>
//!	7 .. 3    (0は使わない)<br> 
//! 6 4 2<br> 
//! 5 .. 1<br><br> 
//! [bitのデータ] <br> 	
//! 余り　重心パターン　脚５　脚４　脚３　脚２　脚１　脚０<br>
//! 1111 1111　　　　　1111　1111　1111　1111　1111　1111<br> <br>
//! 脚は右前脚を0として時計回りに 0 〜 5 ．
//! @author 長谷川

#pragma once
#include <vector>
#include "HexapodConst.h"

/**
* @brief 脚状態を編集するための関数をまとめた名前空間．
* @details 定数は基本的には編集しないようにしてください．脚の離散化方法を変更する時以外編集する必要はないと思います．<br>
* DISCRETE_NUMは脚の離散化数を表しています．この値は他のファイルから参照することが多いと思いますが，それ以外の値は他のファイルから参照しないように，<br>
* どうしても参照したい場合，この名前空間に新しい関数を追加するべきです．<br>
*/
namespace leg_state
{
	constexpr int DISCRETE_NUM = 7;				//!< 脚状態の離散化数．LegState.hの通り7通り．

	constexpr int LEG_POS_MASKBIT = 0b0111;		//!< 脚位置は4bitの下位三桁で管理されるので，そこをマスクする

	constexpr int LEG_STATE_MASKBIT = 0b1111;	//!< 脚状態は4bitで管理されるので，そこをマスクする

	constexpr int SHIFT_TO_COM_NUM = HexapodConst::LEG_NUM * 4;			//!< 重心パターンを保存するビットまで行くために，どれだけビットをシフトするか．

	constexpr int COM_STATE_MASKBIT = (0b1111 << SHIFT_TO_COM_NUM);		//!< 重心パターンを保存するビットをマスクするビット．

	 
	/**
	* 脚状態を作成して返す関数．脚状態は重心パターン，脚の接地・遊脚，離散化した脚位置のデータが含まれる．
	* @param [in] _com_pattern どの重心パターンか．0 〜 9 の整数の範囲．詳しくはComType.hに記述．範囲外ならば自動的に 0 になる．
	* @param [in] _ground 脚が接地しているかを表すbool型の配列．接地しているならばtrue．遊脚しているならばfalse
	* @param [in] _leg_pos 離散化した脚位置を表す変数． 1 〜 7 の範囲で与える．範囲外ならば自動的に 4 になる．
	* @return int 作成した脚状態を返す．
	*/
	int makeLegState(const int _com_pattern, const bool _ground[HexapodConst::LEG_NUM], const int _leg_pos[HexapodConst::LEG_NUM]);

	/**
	* 脚番号_leg_num 0 〜 5 に応じて，その脚が接地しているかを調べる．<br>脚は右前脚を0番として，時計回りに0,1,2,3,4,5となる．左前足が5番．
	* @param [in] _leg_state 現在の脚状態
	* @param [in] _leg_num どの脚を調べるか． 0 〜 5 の整数で入力する．
	* @return bool 脚が接地しているならばtrueを返す．
	*/
	bool isGrounded(const int _leg_state, const int _leg_num);

	/**
	* 接地している脚の本数を返す関数．
	* @param [in] _leg_state 現在の脚状態
	* @return int 接地している脚の本数
	*/
	int getGroundedLegNum(const int _leg_state);

	//接地している脚の脚番号0〜5を，引数_res_numberで参照渡しする関数
	void getGroundedLegNumWithVector(const int _leg_state, std::vector<int>& _res_number);

	//遊脚している脚の脚番号0〜5を，引数_res_numberで参照渡しする関数
	void getLiftedLegNumWithVector(const int _leg_state, std::vector<int>& _res_number);

	// 脚状態を取得する．
	int getLegState(const int _leg_state, const int _leg_num);

	//重心パターンを取得する．0b0000 〜 0b1000 まで
	int getComPatternState(const int _leg_state);

	// 脚の情報を 0〜15 の状態に変更する．引数の値がおかしいならばfalseを出力する．
	// _leg_state 変更する脚状態．_leg_num どの脚を変更するか 0〜5．_new_state 新しい脚状態 1〜15
	bool changeLegState(int &_leg_state, const int _leg_num, const int _new_state);

	// 脚の情報を 0〜7 の状態に変更する．引数の値がおかしいならばfalseを出力する．遊脚を表すbitはそのまま
	// _leg_state 変更する脚状態．_leg_num どの脚を変更するか 0〜5．_new_state 新しい脚状態 1〜7
	bool changeLegStateKeepTopBit(int& _leg_state, const int _leg_num, const int _new_state);

	//脚の接地・遊脚情報を変更する．第1引数の_leg_stateを参照渡しして変更する
	void changeGround(int& _leg_state, const int _leg_num, const bool _ground);

	/**
	* 脚を表す番号は，右前脚を 0 として， 0 〜 5 の範囲．その範囲外ならばfalseを返す．<br> 高速で処理するため inline 関数にしてある．
	* @param [in] _num 脚の本数
	* @return bool 0 〜 5 の範囲内ならばtrue
	*/
	inline bool isAbleLegNum(const int _num) 
	{
		if (0 <= _num && _num < HexapodConst::LEG_NUM) { return true; }	// 0 〜 5なら true

		return false;
	}

	/**
	* 
	*/
	inline bool isAbleLegState(const int _state) 
	{
		if (_state == 8) { return false; }		// 8 (0b1000) なら false

		if (0 < _state && _state < 15) { return true; }		// 1 〜 15なら true

		return false;
	}
}
