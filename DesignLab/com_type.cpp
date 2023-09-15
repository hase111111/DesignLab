#include "com_type.h"

#include <iostream>


namespace
{
	// ���̂悤�ɖ������O��Ԃ̒��ɕϐ���錾���邱�ƂŁC���̃t�@�C�����ł̂ݎg�p�\�ɂȂ�D
	// �A�N�Z�X����ɂ́C�擪��::������D

	const dl_com::LegGroundedMap LEG_GROUNDED_PATTERN_MAP = dl_com::makeLegGroundedMap();		//!< �r�̐ڒn�p�^�[���ɐ��l������U�����}�b�v�D

	const int LEG_GROUNDED_PATTERN_NUM = static_cast<int>(LEG_GROUNDED_PATTERN_MAP.size());	//!< �r�̐ڒn�p�^�[���̐��D

	const std::unordered_map<EDiscreteComPos, std::vector<int>> LEG_GROUNDE_PATTERN_BAN_LIST = dl_com::makeLegGroundedPatternBanList();	//!< �d�S�ʒu����g�p�s�\�Ȑڒn�p�^�[����map�ŊǗ�����D

	const std::vector<std::vector<int>> LEG_GROUNDED_PATTERN_BAN_LIST_FROM_LEG = dl_com::makeLegGroundedPatternBanListFromLeg();		//!< ���̋r���V�r�̂Ƃ��C��蓾�Ȃ��r�̐ڒn�p�^�[�����Ǘ�����D
}



namespace dl_com
{
	LegGroundedMap dl_com::makeLegGroundedMap()
	{
		LegGroundedMap res;
		int counter = 0;


		// �r���ڒn���Ă���ꍇ1�C�V�r�̏ꍇ0�Ƃ��āC6bit�̐��l���쐬����D0�ԋr���V�r�C�c��ڒn�̏ꍇ 111 110 �D
		// �����Ă���� 0 ����n�܂�ԍ�������U��D(�Ǘ����₷�����邽�߁D)
		// �S�p�^�[���𑍓���ŏ����Ă��邯�ǁC�{���͂��̃��X�g���쐬����֐�����肽���D


		// 6�r�S�Đڒn���Ă���ꍇ
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("111111"), counter++));


		// 5�r�ڒn���Ă���ꍇ

		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("011111"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("101111"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("110111"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("111011"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("111101"), counter++));
		res.insert(LegGroundedMapValue(dl_leg::LegGroundedBit("111110"), counter++));


		// 4�r�ڒn���Ă���ꍇ

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


		// 3�r�ڒn���Ă���ꍇ�D�ׂ荇��3�r���V�r���Ă���ꍇ�͏���(�]�|���Ă��܂�����)�D

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


	bool isLegPairFree(int leg_index, int leg_ground_pattern_index)
	{
		dl_leg::LegGroundedBit leg_ground_pattern;

		// index����V�r�̃p�^�[�����擾����D
		try
		{
			leg_ground_pattern = ::LEG_GROUNDED_PATTERN_MAP.right.at(leg_ground_pattern_index);
		}
		catch (...)
		{
			return false;
		}

		// ���ׂ��V�r�̏ꍇ��true��Ԃ��D
		if (!leg_ground_pattern[leg_index % HexapodConst::LEG_NUM] && !leg_ground_pattern[(leg_index + 1) % HexapodConst::LEG_NUM])
		{
			return true;
		}
		else
		{
			return false;
		}
	}


	std::unordered_map<EDiscreteComPos, std::vector<int>> makeLegGroundedPatternBanList()
	{
		std::unordered_map<EDiscreteComPos, std::vector<int>> res;


		// ���{�b�g�̑̂��O�Ɋ���Ă��鎞�ɑO���������Ƃ��V�r���Ɠ]�|���Ă��܂��D
		// ���̂��߁C���U�����ꂽ�d�S����C�A������r�������Ƃ��V�r�ɂȂ�p�^�[�����֎~����̂����̊֐��̖ړI�ł���D
		std::unordered_map<EDiscreteComPos, std::vector<int>> ban_leg_index_list;
		ban_leg_index_list[EDiscreteComPos::FRONT] = { 0,4,5 };
		ban_leg_index_list[EDiscreteComPos::FRONT_RIGHT] = { 0,1,5 };
		ban_leg_index_list[EDiscreteComPos::FRONT_LEFT] = { 3,4,5 };
		ban_leg_index_list[EDiscreteComPos::BACK] = { 1,2,3 };
		ban_leg_index_list[EDiscreteComPos::BACK_RIGHT] = { 0,1,2 };
		ban_leg_index_list[EDiscreteComPos::BACK_LEFT] = { 2,3,4 };
		ban_leg_index_list[EDiscreteComPos::CENTER_BACK] = { 0,2,4 };
		ban_leg_index_list[EDiscreteComPos::CENTER_FRONT] = { 1,3,5 };


		for (auto i : EDiscreteComPos())
		{
			if (ban_leg_index_list.count(i) == 0) { continue; }

			for (auto& j : ban_leg_index_list[i])
			{
				for (int k = 0; k < ::LEG_GROUNDED_PATTERN_NUM; ++k)
				{
					if (isLegPairFree(j, k))
					{
						res[i].push_back(k);
					}
				}
			}
		}

		return std::move(res);
	}


	std::vector<std::vector<int>> makeLegGroundedPatternBanListFromLeg()
	{
		std::vector<std::vector<int>> res;

		res.resize(HexapodConst::LEG_NUM);	// �r�̐�����vector���m�ۂ���D

		// i �ԋr��ڒn���Ȃ���΁C��邱�Ƃ��ł��Ȃ����̂�ۑ�����D
		for (int i = 0; i < HexapodConst::LEG_NUM; i++)
		{
			for (int j = 0; j < ::LEG_GROUNDED_PATTERN_NUM; ++j)
			{
				// i�Ԗڂ�bit���m�F���C�����Ă���Ȃ��(�܂�C���̋r��ڒn���Ȃ���΂����Ȃ��Ȃ�)�C���̃p�^�[�����֎~����D
				if (LEG_GROUNDED_PATTERN_MAP.right.at(j)[i])
				{
					res[i].push_back(j);
				}
			}
		}

		return std::move(res);
	}


	int getLegGroundPatternNum()
	{
		return LEG_GROUNDED_PATTERN_NUM;
	}


	dl_leg::LegGroundedBit getLegGroundedBitFromLegGroundPatternIndex(int leg_ground_pattern_index)
	{
		dl_leg::LegGroundedBit res;

		// index����V�r�̃p�^�[�����擾����D
		res = ::LEG_GROUNDED_PATTERN_MAP.right.at(leg_ground_pattern_index);

		return std::move(res);
	}


	void banLegGroundPatternFromCom(EDiscreteComPos discrete_com_pos, boost::dynamic_bitset<>* output)
	{
		//�ُ�Ȓl�Ȃ��return

		if (output == nullptr) { return; }

		if ((*output).size() != getLegGroundPatternNum()) { return; }


		// LEG_GROUNDE_PATTERN_BAN_LIST�ɃL�[�����݂��Ă��Ȃ����Ƃ�C�l��getLegGroundPatternNum�𒴂��ĂȂ����Ƃ��m�F���Ă��Ȃ��D�G���[���o���炻����������������Ȃ��D

		for (auto& i : LEG_GROUNDE_PATTERN_BAN_LIST.at(discrete_com_pos))
		{
			(*output)[i] = false;
		}
	}


	void banLegGroundPatternFromNotGroundableLeg(int not_groundble_leg_index, boost::dynamic_bitset<>* output)
	{
		//�ُ�Ȓl�Ȃ��return

		if (output == nullptr) { return; }

		if ((*output).size() != getLegGroundPatternNum()) { return; }


		// LEG_GROUNDED_PATTERN_BAN_LIST_FROM_LEG�ɃL�[�����݂��Ă��Ȃ����Ƃ�C�l��getLegGroundPatternNum�𒴂��ĂȂ����Ƃ��m�F���Ă��Ȃ��D�G���[���o���炻����������������Ȃ��D

		for (auto& i : LEG_GROUNDED_PATTERN_BAN_LIST_FROM_LEG[not_groundble_leg_index])
		{
			(*output)[i] = false;
		}
	}


	void banLegGroundPatternFromNotFreeLeg(int not_lift_leg_index, boost::dynamic_bitset<>* output)
	{
		//�ُ�Ȓl�Ȃ��return

		if (output == nullptr) { return; }

		if ((*output).size() != getLegGroundPatternNum()) { return; }


		// LEG_GROUNDED_PATTERN_BAN_LIST_FROM_LEG�ɃL�[�����݂��Ă��Ȃ����Ƃ�C�l��getLegGroundPatternNum�𒴂��ĂȂ����Ƃ��m�F���Ă��Ȃ��D�G���[���o���炻����������������Ȃ��D
		boost::dynamic_bitset<> inverse_output(getLegGroundPatternNum());

		for (auto& i : LEG_GROUNDED_PATTERN_BAN_LIST_FROM_LEG[not_lift_leg_index])
		{
			inverse_output[i] = true;
		}

		(*output) &= inverse_output;
	}



}