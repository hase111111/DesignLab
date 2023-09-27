#include "com_type.h"

#include <iostream>

#include <magic_enum.hpp>

#include "cassert_define.h"


namespace dlcf = designlab::com_func;


namespace
{
	// このように無名名前空間の中に変数を宣言することで，このファイル内でのみ使用可能になる．
	// アクセスするには，先頭に::をつける．
	// ここまでやるなら，classにすりゃよかったかも


	//! 脚の接地パターンに数値を割り振ったマップ．接地を1，遊脚を0として，
	//! { 111111 , 0 } のような形式で代入されている 
	const dlcf::LegGroundedMap kLegGrouededPatternMap = dlcf::MakeLegGroundedMap();		

	//!< 脚の接地パターンの数．
	const int kLegGroundedPatternNum = static_cast<int>(kLegGrouededPatternMap.size());	

	//!< 重心位置から使用不可能な接地パターンをmapで管理する．
	const std::unordered_map<DiscreteComPos, std::vector<int>> kLegGroundedPatternBanList = dlcf::MakeLegGroundedPatternBanList();	

	//!< その脚が遊脚のとき，取り得ない脚の接地パターンを管理する．
	const std::vector<std::vector<int>> kLegGroundedPatternBanListFromLeg = dlcf::MakeLegGroundedPatternBanListFromLeg();
}

namespace designlab::com_func
{
	LegGroundedMap MakeLegGroundedMap()
	{
		LegGroundedMap res;
		int counter = 0;


		// 脚が接地している場合1，遊脚の場合0として，6bitの数値を作成する．0番脚が遊脚，残り接地の場合 111 110 ．
		// そしてそれに 0 から始まる番号を割り振る．(管理しやすくするため．)
		// 全パターンを総当りで書いてあるけど，本当はこのリストを作成する関数を作りたい．


		// 6脚全て接地している場合
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("111111"), counter++));


		// 5脚接地している場合

		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("011111"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("101111"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("110111"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("111011"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("111101"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("111110"), counter++));


		// 4脚接地している場合

		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("001111"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("010111"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("011011"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("011101"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("011110"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("100111"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("101011"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("101101"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("101110"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("110011"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("110101"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("110110"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("111001"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("111010"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("111100"), counter++));


		// 3脚接地している場合．隣り合う3脚が遊脚している場合は除く(転倒してしまうため)．

		//res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("000111"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("001011"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("001101"), counter++));
		//res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("001110"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("010011"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("010101"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("010110"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("011001"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("011010"), counter++));
		//res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("011100"), counter++));
		//res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("100011"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("100101"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("100110"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("101001"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("101010"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("101100"), counter++));
		//res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("110001"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("110010"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("110100"), counter++));
		//res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("111000"), counter++));

		return std::move(res);
	}


	bool IsLegPairFree(int leg_index, int leg_ground_pattern_index)
	{
		dl_leg::LegGroundedBit leg_ground_pattern;

		// indexから遊脚のパターンを取得する．
		try
		{
			leg_ground_pattern = ::kLegGrouededPatternMap.right.at(leg_ground_pattern_index);
		}
		catch (...)
		{
			return false;
		}

		// 両隣が遊脚の場合はtrueを返す．
		if (!leg_ground_pattern[leg_index % HexapodConst::LEG_NUM] && !leg_ground_pattern[(leg_index + 1) % HexapodConst::LEG_NUM])
		{
			return true;
		}
		else
		{
			return false;
		}
	}


	std::unordered_map<DiscreteComPos, std::vector<int>> MakeLegGroundedPatternBanList()
	{
		std::unordered_map<DiscreteComPos, std::vector<int>> res;


		// ロボットの体が前に寄っている時に前足が両方とも遊脚だと転倒してしまう．
		// そのため，離散化された重心から，連続する脚が両方とも遊脚になるパターンを禁止するのがこの関数の目的である．
		std::unordered_map<DiscreteComPos, std::vector<int>> ban_leg_index_list;
		ban_leg_index_list[DiscreteComPos::kFront] = { 0,4,5 };
		ban_leg_index_list[DiscreteComPos::kFrontRight] = { 0,1,5 };
		ban_leg_index_list[DiscreteComPos::kFrontLeft] = { 3,4,5 };
		ban_leg_index_list[DiscreteComPos::kBack] = { 1,2,3 };
		ban_leg_index_list[DiscreteComPos::kBackRight] = { 0,1,2 };
		ban_leg_index_list[DiscreteComPos::kBackLeft] = { 2,3,4 };
		ban_leg_index_list[DiscreteComPos::kCenterBack] = { 0,2,4 };
		ban_leg_index_list[DiscreteComPos::kCenterFront] = { 1,3,5 };


		// DiscreteComPosの要素数だけループする
		for (const auto &i : magic_enum::enum_values<DiscreteComPos>())
		{
			if (ban_leg_index_list.count(i) == 0) { continue; }

			for (auto& j : ban_leg_index_list[i])
			{
				for (int k = 0; k < ::kLegGroundedPatternNum; ++k)
				{
					if (IsLegPairFree(j, k))
					{
						res[i].push_back(k);
					}
				}
			}
		}

		return std::move(res);
	}


	std::vector<std::vector<int>> MakeLegGroundedPatternBanListFromLeg()
	{
		std::vector<std::vector<int>> res;

		res.resize(HexapodConst::LEG_NUM);	// 脚の数だけvectorを確保する．

		// i 番脚を接地しなければ，取ることができないものを保存する．
		for (int i = 0; i < HexapodConst::LEG_NUM; i++)
		{
			for (int j = 0; j < ::kLegGroundedPatternNum; ++j)
			{
				// i番目のbitを確認し，立っているならば(つまり，その脚を接地しなければいけないなら)，そのパターンを禁止する．
				if (::kLegGrouededPatternMap.right.at(j)[i])
				{
					res[i].push_back(j);
				}
			}
		}

		return std::move(res);
	}


	int GetLegGroundPatternNum()
	{
		return kLegGroundedPatternNum;
	}

	dl_leg::LegGroundedBit GetLegGroundedBitFromLegGroundPatternIndex(int leg_ground_pattern_index)
	{
		dl_leg::LegGroundedBit res;

		// indexから遊脚のパターンを取得する．
		res = ::kLegGrouededPatternMap.right.at(leg_ground_pattern_index);

		return std::move(res);
	}


	void RemoveLegGroundPatternFromCom(DiscreteComPos discrete_com_pos, boost::dynamic_bitset<>* output)
	{
		//異常な値ならばreturn

		if (output == nullptr) { return; }

		if ((*output).size() != GetLegGroundPatternNum()) { return; }


		// LEG_GROUNDE_PATTERN_BAN_LISTにキーが存在していないことや，値がgetLegGroundPatternNumを超えてないことを確認していない．エラーが出たらそこが原因かもしれない．

		for (auto& i : kLegGroundedPatternBanList.at(discrete_com_pos))
		{
			(*output)[i] = false;
		}
	}

	void RemoveLegGroundPatternFromNotGroundableLeg(int not_groundble_leg_index, boost::dynamic_bitset<>* output)
	{
		//異常な値ならばreturn

		if (output == nullptr) { return; }

		if ((*output).size() != GetLegGroundPatternNum()) { return; }


		// LEG_GROUNDED_PATTERN_BAN_LIST_FROM_LEGにキーが存在していないことや，値がgetLegGroundPatternNumを超えてないことを確認していない．エラーが出たらそこが原因かもしれない．

		for (auto& i : kLegGroundedPatternBanListFromLeg[not_groundble_leg_index])
		{
			(*output)[i] = false;
		}
	}

	void RemoveLegGroundPatternFromNotFreeLeg(int not_lift_leg_index, boost::dynamic_bitset<>* output)
	{
		//異常な値ならばreturn

		if (output == nullptr) { return; }

		if ((*output).size() != GetLegGroundPatternNum()) { return; }


		// LEG_GROUNDED_PATTERN_BAN_LIST_FROM_LEGにキーが存在していないことや，値がgetLegGroundPatternNumを超えてないことを確認していない．エラーが出たらそこが原因かもしれない．
		boost::dynamic_bitset<> inverse_output(GetLegGroundPatternNum());

		for (auto& i : kLegGroundedPatternBanListFromLeg[not_lift_leg_index])
		{
			inverse_output[i] = true;
		}

		(*output) &= inverse_output;
	}

}	// namespace designlab::com_func