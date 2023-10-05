#include "leg_state.h"

#include "cassert_define.h"


namespace designlab::leg_func
{
	LegStateBit MakeLegStateBit(const DiscreteComPos discrete_com_pos, const std::array<bool, HexapodConst::LEG_NUM>& is_ground,
		const std::array<DiscreteLegPos, HexapodConst::LEG_NUM>& discretized_leg_pos)
	{
		LegStateBit res = 0;

		res |= static_cast<int>(discrete_com_pos) << kShiftToComNum;	//重心パターンの数値だけbitを立てる


		for (int i = 0; i < HexapodConst::LEG_NUM; i++)
		{
			//接地しているならば上位bitを立てる
			if (is_ground[i]) { res[(i + 1) * 4 - 1] = true; }

			// 脚のbitを立てる
			res |= static_cast<int>(discretized_leg_pos[i]) << (i * 4);
		}

		return res;
	}


	bool IsGrounded(const LegStateBit& leg_state, const int leg_index)
	{
		// leg_indexは0〜5の範囲にある必要がある．
		assert(0 <= leg_index );
		assert(leg_index < HexapodConst::LEG_NUM);

		//指定された脚の接地脚のbitが立っているか調べる
		if (leg_state[(leg_index + 1) * 4 - 1])
		{
			return true;
		}
		else
		{
			return false;
		}
	}


	LegGroundedBit GetLegGroundedBit(const LegStateBit& leg_state)
	{
		LegGroundedBit res;

		for (int i = 0; i < HexapodConst::LEG_NUM; i++)
		{
			if (IsGrounded(leg_state, i))
			{
				res[i] = true;
			}
			else
			{
				res[i] = false;
			}
		}

		return std::move(res);
	}


	int GetGroundedLegNum(const LegStateBit& leg_state)
	{
		int res = 0;

		//脚の本数分ループする
		for (int i = 0; i < HexapodConst::LEG_NUM; i++)
		{
			if (IsGrounded(leg_state, i))
			{
				//接地している脚があればカウントアップする
				res++;
			}
		}

		return res;
	}


	int GetLiftedLegNum(const LegStateBit& leg_state)
	{
		return HexapodConst::LEG_NUM - GetGroundedLegNum(leg_state);
	}


	void GetGroundedLegIndexByVector(const LegStateBit& leg_state, std::vector<int>* res_index)
	{
		// res_indexはnullptrでないこと，かつ空である必要がある
		assert(res_index != nullptr);
		assert((*res_index).size() == 0);

		//脚は6本あるので6回ループする
		for (int i = 0; i < HexapodConst::LEG_NUM; i++)
		{
			if (IsGrounded(leg_state, i))
			{
				//接地している脚の脚番号をvectorに代入
				(*res_index).push_back(i);
			}
		}
	}


	void GetLiftedLegIndexByVector(const LegStateBit& leg_state, std::vector<int>* res_index)
	{
		// res_indexはnullptrでないこと，かつ空である必要がある
		assert(res_index != nullptr);
		assert((*res_index).size() == 0);

		//脚は6本あるので6回ループする
		for (int i = 0; i < HexapodConst::LEG_NUM; i++)
		{
			if (!IsGrounded(leg_state, i))
			{
				//浮いている脚の脚番号をvectorに代入
				(*res_index).push_back(i);
			}
		}
	}


	DiscreteLegPos GetDiscreteLegPos(const LegStateBit& leg_state, const int leg_index)
	{
		// leg_indexは0〜5の範囲にある必要がある．
		assert(0 <= leg_index);
		assert(leg_index < HexapodConst::LEG_NUM);

		const int shift_num = 4 * leg_index;	//4bitずつずらす

		const int res = static_cast<int>(((leg_state & (kLegPosMaskbit << shift_num)) >> shift_num).to_ulong());

		return static_cast<DiscreteLegPos>(res);
	}


	DiscreteComPos GetDiscreteComPos(const LegStateBit& leg_state)
	{
		//重心パターンを保存するビットをマスクし，その値だけ取得できるように右へシフトする．
		const int res = static_cast<int>(((leg_state & kComStateMaskbit) >> kShiftToComNum).to_ulong());

		return static_cast<DiscreteComPos>(res);
	}


	bool ChangeLegState(const int leg_index, const DiscreteLegPos new_discretized_leg_pos, const bool is_ground, LegStateBit* leg_state)
	{
		// leg_indexは0〜5の範囲にある必要がある．
		assert(0 <= leg_index);
		assert(leg_index < HexapodConst::LEG_NUM);

		// leg_state は nullptrではない
		assert(leg_state != nullptr);


		//新しい脚状態を生成する
		LegStateBit mask = kLegStateMaskbit << (leg_index * 4);								//4bitのデータを変更する地点までマスクをずらす
		LegStateBit  state = static_cast<int>(new_discretized_leg_pos) << (leg_index * 4);	//脚位置のデータは4bitづつ配置されているのでその位置まで移動する

		//浮いている脚の脚位置のみを変更（排他的論理和による特定ビットの交換 https://qiita.com/vivisuke/items/bc707190e008551ca07f）
		LegStateBit res = ((*leg_state) ^ state) & mask;
		(*leg_state) ^= res;

		ChangeGround(leg_index, is_ground, leg_state);

		return true;
	}


	bool ChangeDiscreteLegPos(const int leg_index, const DiscreteLegPos new_discretized_leg_pos, LegStateBit* leg_state)
	{
		// leg_indexは0〜5の範囲にある必要がある．
		assert(0 <= leg_index);
		assert(leg_index < HexapodConst::LEG_NUM);

		// leg_state は nullptrではない
		assert(leg_state != nullptr);


		//新しい脚状態を生成する
		LegStateBit mask = kLegPosMaskbit << (leg_index * 4);								//4bitのデータを変更する地点までマスクをずらす
		LegStateBit state = static_cast<int>(new_discretized_leg_pos) << (leg_index * 4);	//脚位置のデータは4bitづつ配置されているのでその位置まで移動する

		//浮いている脚の脚位置のみを変更（排他的論理和による特定ビットの交換 https://qiita.com/vivisuke/items/bc707190e008551ca07f）
		LegStateBit res = ((*leg_state) ^ state) & mask;
		(*leg_state) ^= res;

		return true;
	}


	void ChangeGround(const int leg_index, const bool is_ground, LegStateBit* leg_state)
	{
		// leg_indexは0〜5の範囲にある必要がある．
		assert(0 <= leg_index);
		assert(leg_index < HexapodConst::LEG_NUM);

		// leg_state は nullptrではない
		assert(leg_state != nullptr);


		//指定された脚の接地脚のbitを立てるか消すかする
		if (is_ground)
		{
			(*leg_state)[(leg_index + 1) * 4 - 1] = true;
		}
		else
		{
			(*leg_state)[(leg_index + 1) * 4 - 1] = false;
		}
	}


	void ChangeAllLegGround(const LegGroundedBit& is_ground_list, LegStateBit* leg_state)
	{
		// leg_state は nullptrではない
		assert(leg_state != nullptr);

		for (int i = 0; i < HexapodConst::LEG_NUM; i++)
		{
			ChangeGround(i, is_ground_list[i], leg_state);
		}
	}


	void ChangeDiscreteComPos(const DiscreteComPos new_com_pattern, LegStateBit* leg_state)
	{
		// leg_state は nullptrではない
		assert(leg_state != nullptr);

		const LegStateBit state = static_cast<int>(new_com_pattern) << kShiftToComNum;
		LegStateBit sub = ((*leg_state) ^ state) & kComStateMaskbit;
		(*leg_state) ^= sub;
	}

}	// namespace designlab::leg_func