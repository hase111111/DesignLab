#include "com_type.h"

#include <iostream>


namespace
{
	// ���̂悤�ɖ������O��Ԃ̒��ɕϐ���錾���邱�ƂŁC���̃t�@�C�����ł̂ݎg�p�\�ɂȂ�D
	// �A�N�Z�X����ɂ́C�擪��::������D

	const dl_com::LegGroundedMap LEG_GROUNDED_PATTERN_MAP = dl_com::makeLegGroundedMap();		//!< �r�̐ڒn�p�^�[���ɐ��l������U�����}�b�v�D

	const size_t LEG_GROUNDED_PATTERN_NUM = LEG_GROUNDED_PATTERN_MAP.size();	//!< �r�̐ڒn�p�^�[���̐��D

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



}


void dl_com::getGroundLegFromComType(const int _com_type, bool _output_ground_leg[HexapodConst::LEG_NUM])
{
	switch (_com_type)
	{
	case 0:
		_output_ground_leg[0] = true; _output_ground_leg[1] = true; _output_ground_leg[2] = true;
		_output_ground_leg[3] = true; _output_ground_leg[4] = true; _output_ground_leg[5] = true;
		break;

	case 1:
		_output_ground_leg[0] = true;	_output_ground_leg[1] = false;	_output_ground_leg[2] = true;
		_output_ground_leg[3] = true;	_output_ground_leg[4] = true;	_output_ground_leg[5] = true;
		break;

	case 2:
		_output_ground_leg[0] = true;	_output_ground_leg[1] = true;	_output_ground_leg[2] = false;
		_output_ground_leg[3] = true;	_output_ground_leg[4] = true;	_output_ground_leg[5] = true;
		break;

	case 3:
		_output_ground_leg[0] = true;	_output_ground_leg[1] = true;	_output_ground_leg[2] = true;
		_output_ground_leg[3] = false;	_output_ground_leg[4] = true;	_output_ground_leg[5] = true;
		break;

	case 4:
		_output_ground_leg[0] = true;	_output_ground_leg[1] = true;	_output_ground_leg[2] = true;
		_output_ground_leg[3] = true;	_output_ground_leg[4] = false;	_output_ground_leg[5] = true;
		break;

	case 5:
		_output_ground_leg[0] = true;	_output_ground_leg[1] = true;	_output_ground_leg[2] = true;
		_output_ground_leg[3] = true;	_output_ground_leg[4] = true;	_output_ground_leg[5] = false;
		break;

	case 6:
		_output_ground_leg[0] = false;	_output_ground_leg[1] = true;	_output_ground_leg[2] = true;
		_output_ground_leg[3] = true;	_output_ground_leg[4] = true;	_output_ground_leg[5] = true;
		break;

	case 7:
		_output_ground_leg[0] = false;	_output_ground_leg[1] = true;	_output_ground_leg[2] = true;
		_output_ground_leg[3] = false;	_output_ground_leg[4] = true;	_output_ground_leg[5] = true;
		break;

	case 8:
		_output_ground_leg[0] = true;	_output_ground_leg[1] = true;	_output_ground_leg[2] = false;
		_output_ground_leg[3] = true;	_output_ground_leg[4] = true;	_output_ground_leg[5] = false;
		break;

	case 9:
		_output_ground_leg[0] = true;	_output_ground_leg[1] = false;	_output_ground_leg[2] = true;
		_output_ground_leg[3] = true;	_output_ground_leg[4] = false;	_output_ground_leg[5] = true;
		break;

	case 10:
		_output_ground_leg[0] = true;	_output_ground_leg[1] = false;	_output_ground_leg[2] = true;
		_output_ground_leg[3] = false;	_output_ground_leg[4] = true;	_output_ground_leg[5] = true;
		break;

	case 11:
		_output_ground_leg[0] = true;	_output_ground_leg[1] = true;	_output_ground_leg[2] = false;
		_output_ground_leg[3] = true;	_output_ground_leg[4] = false;	_output_ground_leg[5] = true;
		break;

	case 12:
		_output_ground_leg[0] = true;	_output_ground_leg[1] = true;	_output_ground_leg[2] = true;
		_output_ground_leg[3] = false;	_output_ground_leg[4] = true;	_output_ground_leg[5] = false;
		break;

	case 13:
		_output_ground_leg[0] = false;	_output_ground_leg[1] = true;	_output_ground_leg[2] = true;
		_output_ground_leg[3] = true;	_output_ground_leg[4] = false;	_output_ground_leg[5] = true;
		break;

	case 14:
		_output_ground_leg[0] = true;	_output_ground_leg[1] = false;	_output_ground_leg[2] = true;
		_output_ground_leg[3] = true;	_output_ground_leg[4] = true;	_output_ground_leg[5] = false;
		break;

	case 15:
		_output_ground_leg[0] = false;	_output_ground_leg[1] = true;	_output_ground_leg[2] = false;
		_output_ground_leg[3] = true;	_output_ground_leg[4] = true;	_output_ground_leg[5] = true;
		break;

	case 16:
		_output_ground_leg[0] = true;	_output_ground_leg[1] = false;	_output_ground_leg[2] = true;
		_output_ground_leg[3] = false;	_output_ground_leg[4] = true;	_output_ground_leg[5] = false;
		break;

	case 17:
		_output_ground_leg[0] = false;	_output_ground_leg[1] = true;	_output_ground_leg[2] = false;
		_output_ground_leg[3] = true;	_output_ground_leg[4] = false;	_output_ground_leg[5] = true;
		break;

	case 18:
		_output_ground_leg[0] = false;	_output_ground_leg[1] = false;	_output_ground_leg[2] = true;
		_output_ground_leg[3] = true;	_output_ground_leg[4] = true;	_output_ground_leg[5] = true;
		break;

	case 19:
		_output_ground_leg[0] = true;	_output_ground_leg[1] = false;	_output_ground_leg[2] = false;
		_output_ground_leg[3] = true;	_output_ground_leg[4] = true;	_output_ground_leg[5] = true;
		break;

	case 20:
		_output_ground_leg[0] = true;	_output_ground_leg[1] = true;	_output_ground_leg[2] = false;
		_output_ground_leg[3] = false;	_output_ground_leg[4] = true;	_output_ground_leg[5] = true;
		break;

	case 21:
		_output_ground_leg[0] = true;	_output_ground_leg[1] = true;	_output_ground_leg[2] = true;
		_output_ground_leg[3] = false;	_output_ground_leg[4] = false;	_output_ground_leg[5] = true;
		break;

	case 22:
		_output_ground_leg[0] = true;	_output_ground_leg[1] = true;	_output_ground_leg[2] = true;
		_output_ground_leg[3] = true;	_output_ground_leg[4] = false;	_output_ground_leg[5] = false;
		break;

	case 23:
		_output_ground_leg[0] = false;	_output_ground_leg[1] = true;	_output_ground_leg[2] = true;
		_output_ground_leg[3] = true;	_output_ground_leg[4] = true;	_output_ground_leg[5] = false;
		break;

	case 24:
		_output_ground_leg[0] = false;	_output_ground_leg[1] = false;	_output_ground_leg[2] = true;
		_output_ground_leg[3] = true;	_output_ground_leg[4] = false;	_output_ground_leg[5] = true;
		break;

	case 25:
		_output_ground_leg[0] = true;	_output_ground_leg[1] = false;	_output_ground_leg[2] = false;
		_output_ground_leg[3] = true;	_output_ground_leg[4] = true;	_output_ground_leg[5] = false;
		break;

	case 26:
		_output_ground_leg[0] = false;	_output_ground_leg[1] = true;	_output_ground_leg[2] = false;
		_output_ground_leg[3] = false;	_output_ground_leg[4] = true;	_output_ground_leg[5] = true;
		break;

	case 27:
		_output_ground_leg[0] = true;	_output_ground_leg[1] = false;	_output_ground_leg[2] = true;
		_output_ground_leg[3] = false;	_output_ground_leg[4] = false;	_output_ground_leg[5] = true;
		break;

	case 28:
		_output_ground_leg[0] = true;	_output_ground_leg[1] = true;	_output_ground_leg[2] = false;
		_output_ground_leg[3] = true;	_output_ground_leg[4] = false;	_output_ground_leg[5] = false;
		break;

	case 29:
		_output_ground_leg[0] = false;	_output_ground_leg[1] = true;	_output_ground_leg[2] = true;
		_output_ground_leg[3] = false;	_output_ground_leg[4] = true;	_output_ground_leg[5] = false;
		break;

	case 30:
		_output_ground_leg[0] = false;	_output_ground_leg[1] = false;	_output_ground_leg[2] = true;
		_output_ground_leg[3] = false;	_output_ground_leg[4] = true;	_output_ground_leg[5] = true;
		break;

	case 31:
		_output_ground_leg[0] = true;	_output_ground_leg[1] = false;	_output_ground_leg[2] = false;
		_output_ground_leg[3] = true;	_output_ground_leg[4] = false;	_output_ground_leg[5] = true;
		break;

	case 32:
		_output_ground_leg[0] = true;	_output_ground_leg[1] = true;	_output_ground_leg[2] = false;
		_output_ground_leg[3] = false;	_output_ground_leg[4] = true;	_output_ground_leg[5] = false;
		break;

	case 33:
		_output_ground_leg[0] = false;	_output_ground_leg[1] = true;	_output_ground_leg[2] = true;
		_output_ground_leg[3] = false;	_output_ground_leg[4] = false;	_output_ground_leg[5] = true;
		break;

	case 34:
		_output_ground_leg[0] = true;	_output_ground_leg[1] = false;	_output_ground_leg[2] = true;
		_output_ground_leg[3] = true;	_output_ground_leg[4] = false;	_output_ground_leg[5] = false;
		break;

	case 35:
		_output_ground_leg[0] = false;	_output_ground_leg[1] = true;	_output_ground_leg[2] = false;
		_output_ground_leg[3] = true;	_output_ground_leg[4] = true;	_output_ground_leg[5] = false;
		break;

	default:
		_output_ground_leg[0] = false;	_output_ground_leg[1] = false;	_output_ground_leg[2] = false;
		_output_ground_leg[3] = false;	_output_ground_leg[4] = false;	_output_ground_leg[5] = false;
		break;
		break;
	}
}

void dl_com::getDonotUseComTypeFromComPattern(const int _com_pattern, std::vector<int>& _output)
{
	//com pattern����g�p�s�\��com type���擾���ďo�͂���D
	for (const auto& i : COMTYPE_BAN_LIST[_com_pattern])
	{
		_output.push_back(i);
	}
}

void dl_com::checkAbleComTypeFromComPattern(const int _com_pattern, bool _com_type_able_array[COM_TYPE_NUM])
{
	std::vector<int> _cannot_use_type;
	getDonotUseComTypeFromComPattern(_com_pattern, _cannot_use_type);

	for (const auto& i : _cannot_use_type)
	{
		_com_type_able_array[i] = false;
	}
}

void dl_com::checkAbleComTypeFromNotGroundableLeg(const int _not_groundble_leg, bool _com_type_able_array[COM_TYPE_NUM])
{
	//�Ƃ邱�Ƃ̂ł��Ȃ�com type��S��false�ɕύX����D
	for (int i = 0; i < BAN_LIST_ARRAY_SIZE; i++)
	{
		_com_type_able_array[BAN_LIST[_not_groundble_leg][i]] = false;
	}
}

void dl_com::checkAbleComTypeFromNotFreeLeg(const int _not_free_leg, bool _com_type_able_array[COM_TYPE_NUM])
{
	bool _reverse[COM_TYPE_NUM];

	for (int i = 0; i < COM_TYPE_NUM; i++)
	{
		_reverse[i] = true;
	}


	//�Ƃ邱�Ƃ̂ł��Ȃ�com type��S��false�ɕύX����D
	for (int i = 0; i < BAN_LIST_ARRAY_SIZE; i++)
	{
		_reverse[BAN_LIST[_not_free_leg][i]] = false;
	}

	for (int i = 0; i < COM_TYPE_NUM; i++)
	{
		if (_reverse[i] == true)
		{
			_com_type_able_array[i] = false;
		}
	}
}