#include "com_type.h"

#include <iostream>

#include <magic_enum.hpp>

#include "cassert_define.h"


namespace dlcf = designlab::com_func;
namespace dllf = designlab::leg_func;


// com_func���̊֐��͎��ۂɏ������s���ۂɌv�Z���s���ƒx���Ȃ邽�߁C���������Ɉ�x�����Ăяo���āC���ʂ�ۑ����Ă����D
// ���̒l���Ăяo�����Ƃő��x���グ�Ă���D
// ���̂悤�ɖ������O��Ԃ̒��ɕϐ���錾���邱�ƂŁC���̃t�@�C�����ł̂ݎg�p�\�ɂȂ�D�������Ăł����ϐ��Ɍ��ʂ�ۑ�����D
// �A�N�Z�X����ɂ́C�擪��::������D
// �����܂ł��Ȃ�Cclass�ɂ����悩��������
namespace
{
	//! @brief �r�̐ڒn�p�^�[����\���}�b�v���쐬����֐��D���������Ɉ�x�����Ăяo���D
	dlcf::LegGroundedMap MakeLegGroundedMap() 
	{
		dlcf::LegGroundedMap res;
		int counter = 0;


		// �r���ڒn���Ă���ꍇ1�C�V�r�̏ꍇ0�Ƃ��āC6bit�̐��l���쐬����D0�ԋr���V�r�C�c��ڒn�̏ꍇ 111 110 �D
		// �����Ă���� 0 ����n�܂�ԍ�������U��D(�Ǘ����₷�����邽�߁D)
		// �S�p�^�[���𑍓���ŏ����Ă��邯�ǁC�{���͂��̃��X�g���쐬����֐�����肽���D

		// �g���C�|�b�g���e�Ɏg�p����p�^�[���D
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("010101"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("101010"), counter++));

		// 6�r�S�Đڒn���Ă���ꍇ
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("111111"), counter++));


		// 5�r�ڒn���Ă���ꍇ

		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("011111"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("101111"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("110111"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("111011"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("111101"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("111110"), counter++));


		// 4�r�ڒn���Ă���ꍇ

		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("001111"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("010111"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("011011"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("011101"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("011110"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("100111"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("101011"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("101101"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("101110"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("110011"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("110101"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("110110"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("111001"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("111010"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("111100"), counter++));


		// 3�r�ڒn���Ă���ꍇ�D�ׂ荇��3�r���V�r���Ă���ꍇ�͏���(�]�|���Ă��܂�����)�D

		//res.insert(LegGroundedMapValue(dllf::LegGroundedBit("000111"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("001011"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("001101"), counter++));
		//res.insert(LegGroundedMapValue(dllf::LegGroundedBit("001110"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("010011"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("010110"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("011001"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("011010"), counter++));
		//res.insert(LegGroundedMapValue(dllf::LegGroundedBit("011100"), counter++));
		//res.insert(LegGroundedMapValue(dllf::LegGroundedBit("100011"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("100101"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("100110"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("101001"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("101100"), counter++));
		//res.insert(LegGroundedMapValue(dllf::LegGroundedBit("110001"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("110010"), counter++));
		res.insert(dlcf::LegGroundedMapValue(dllf::LegGroundedBit("110100"), counter++));
		//res.insert(LegGroundedMapValue(dllf::LegGroundedBit("111000"), counter++));

		return std::move(res);
	}

	//! �r�̐ڒn�p�^�[���ɐ��l������U�����}�b�v�D�ڒn��1�C�V�r��0�Ƃ��āC
	//! { 111111 , 0 } �̂悤�Ȍ`���ő������Ă��� 
	const dlcf::LegGroundedMap kLegGrouededPatternMap = MakeLegGroundedMap();

	//!< �r�̐ڒn�p�^�[���̐��D
	const int kLegGroundedPatternNum = static_cast<int>(kLegGrouededPatternMap.size());



	//! @brief leg_index�� leg_index + 1 �Ԃ̋r���Ƃ��ɗV�r�ɂȂ鎞��true��Ԃ��֐��D�������p�Ɏg�p���Ă���
	//! @param leg_index �r�̔ԍ��D
	//! @param leg_ground_pattern_index �r�̐ڒn�p�^�[���̔ԍ��D
	//! @return leg_index�� leg_index + 1 �Ԃ̋r���Ƃ��ɗV�r�ɂȂ鎞��true�D
	bool IsLegPairFree(int leg_index, int leg_ground_pattern_index)
	{
		dllf::LegGroundedBit leg_ground_pattern;

		// index����V�r�̃p�^�[�����擾����D
		try
		{
			leg_ground_pattern = ::kLegGrouededPatternMap.right.at(leg_ground_pattern_index);
		}
		catch (...)
		{
			return false;
		}

		// ���ׂ��V�r�̏ꍇ��true��Ԃ��D
		if (!leg_ground_pattern[leg_index % HexapodConst::kLegNum] && !leg_ground_pattern[(leg_index + 1) % HexapodConst::kLegNum])
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	//! @brief �d�S�ʒu����g�p�s�\�Ȑڒn�p�^�[�����쐬����֐��D���������Ɉ�x�����Ăяo���D
	std::unordered_map<DiscreteComPos, std::vector<int>> MakeLegGroundedPatternBanList()
	{
		std::unordered_map<DiscreteComPos, std::vector<int>> res;


		// ���{�b�g�̑̂��O�Ɋ���Ă��鎞�ɑO���������Ƃ��V�r���Ɠ]�|���Ă��܂��D
		// ���̂��߁C���U�����ꂽ�d�S����C�Ƃ邱�Ƃ��ł��Ȃ��C�A������r�������Ƃ��V�r�ɂȂ�p�^�[�����֎~����̂����̊֐��̖ړI�ł���D
		std::unordered_map<DiscreteComPos, std::vector<int>> ban_leg_index_list;
		ban_leg_index_list[DiscreteComPos::kFront] = { 0,4,5 };
		ban_leg_index_list[DiscreteComPos::kFrontRight] = { 0,1,5 };
		ban_leg_index_list[DiscreteComPos::kFrontLeft] = { 3,4,5 };
		ban_leg_index_list[DiscreteComPos::kBack] = { 1,2,3 };
		ban_leg_index_list[DiscreteComPos::kBackRight] = { 0,1,2 };
		ban_leg_index_list[DiscreteComPos::kBackLeft] = { 2,3,4 };
		ban_leg_index_list[DiscreteComPos::kCenterBack] = { 0,2,4 };
		ban_leg_index_list[DiscreteComPos::kCenterFront] = { 1,3,5 };


		// DiscreteComPos�̗v�f���������[�v����D
		// magic_enum::enum_values<DiscreteComPos>()�́CDiscreteComPos�̗v�f��񋓂���array��Ԃ��D
		for (const auto& i : magic_enum::enum_values<DiscreteComPos>())
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

	//! @brief ����̋r���ڒn�ł��Ȃ��ꍇ�Ɏ�蓾�Ȃ��ڒn�p�^�[�����쐬����֐��́D���������Ɉ�x�����Ăяo���D 
	std::vector<std::vector<int>> MakeLegGroundedPatternBanListFromLeg()
	{
		std::vector<std::vector<int>> res;

		res.resize(HexapodConst::kLegNum);	// �r�̐�����vector���m�ۂ���D

		// i �ԋr��ڒn���Ȃ���΁C��邱�Ƃ��ł��Ȃ����̂�ۑ�����D
		for (int i = 0; i < HexapodConst::kLegNum; i++)
		{
			for (int j = 0; j < ::kLegGroundedPatternNum; ++j)
			{
				// i�Ԗڂ�bit���m�F���C�����Ă���Ȃ��(�܂�C���̋r��ڒn���Ȃ���΂����Ȃ��Ȃ�)�C���̃p�^�[�����֎~����D
				if (::kLegGrouededPatternMap.right.at(j)[i])
				{
					res[i].push_back(j);
				}
			}
		}

		return std::move(res);
	}


	//!< �d�S�ʒu����g�p�s�\�Ȑڒn�p�^�[����map�ŊǗ�����D
	const std::unordered_map<DiscreteComPos, std::vector<int>> kLegGroundedPatternBanList = MakeLegGroundedPatternBanList();	

	//!< ���̋r���V�r�̂Ƃ��C��蓾�Ȃ��r�̐ڒn�p�^�[�����Ǘ�����D
	const std::vector<std::vector<int>> kLegGroundedPatternBanListFromLeg = MakeLegGroundedPatternBanListFromLeg();
}


int designlab::com_func::GetLegGroundPatternNum()
{
	return kLegGroundedPatternNum;
}

dllf::LegGroundedBit designlab::com_func::GetLegGroundedBitFromLegGroundPatternIndex(const int leg_ground_pattern_index)
{
	dllf::LegGroundedBit res;

	// index����V�r�̃p�^�[�����擾����D
	res = ::kLegGrouededPatternMap.right.at(leg_ground_pattern_index);

	return res;
}


void designlab::com_func::RemoveLegGroundPatternFromCom(DiscreteComPos discrete_com_pos, boost::dynamic_bitset<>* output)
{
	assert(output != nullptr);
	assert((*output).size() == GetLegGroundPatternNum());

	// kLegGroundedPatternBanList �ɃL�[�����݂��Ă��Ȃ����Ƃ�C
	// �l��getLegGroundPatternNum�𒴂��ĂȂ����Ƃ��m�F���Ă��Ȃ��D�G���[���o���炻����������������Ȃ��D
	for (auto& i : kLegGroundedPatternBanList.at(discrete_com_pos))
	{
		(*output)[i] = false;
	}
}

void designlab::com_func::RemoveLegGroundPatternFromNotGroundableLeg(int not_groundble_leg_index, boost::dynamic_bitset<>* output)
{
	assert(output != nullptr);
	assert((*output).size() == GetLegGroundPatternNum());

	// LEG_GROUNDED_PATTERN_BAN_LIST_FROM_LEG�ɃL�[�����݂��Ă��Ȃ����Ƃ�C�l��getLegGroundPatternNum�𒴂��ĂȂ����Ƃ��m�F���Ă��Ȃ��D�G���[���o���炻����������������Ȃ��D

	for (auto& i : kLegGroundedPatternBanListFromLeg[not_groundble_leg_index])
	{
		(*output)[i] = false;
	}
}

void designlab::com_func::RemoveLegGroundPatternFromNotFreeLeg(int not_lift_leg_index, boost::dynamic_bitset<>* output)
{
	assert(output != nullptr);
	assert((*output).size() == GetLegGroundPatternNum());

	// LEG_GROUNDED_PATTERN_BAN_LIST_FROM_LEG�ɃL�[�����݂��Ă��Ȃ����Ƃ�C�l��getLegGroundPatternNum�𒴂��ĂȂ����Ƃ��m�F���Ă��Ȃ��D�G���[���o���炻����������������Ȃ��D
	boost::dynamic_bitset<> inverse_output(GetLegGroundPatternNum());

	for (auto& i : kLegGroundedPatternBanListFromLeg[not_lift_leg_index])
	{
		inverse_output[i] = true;
	}

	(*output) &= inverse_output;
}