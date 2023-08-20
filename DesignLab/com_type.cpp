#include "com_type.h"

#include <iostream>

#include "leg_state.h"


int ComType::convertComPatternToBit(const EComPattern _com_pattern)
{
	switch (_com_pattern)
	{
	case EComPattern::FRONT:
		return 0b0110;
		break;
	case EComPattern::FRONT_LEFT:
		return 0b0001;
		break;
	case EComPattern::BACK_LEFT:
		return 0b0010;
		break;
	case EComPattern::BACK:
		return 0b0011;
		break;
	case EComPattern::BACK_RIGHT:
		return 0b0100;
		break;
	case EComPattern::FRONT_RIGHT:
		return 0b0101;
		break;
	case EComPattern::CENTER_FRONT:
		return 0b1000;
		break;
	case EComPattern::CENTER_BACK:
		return 0b0111;
		break;
	default:
		return 0;
		break;
	}
}

ComType::EComPattern ComType::convertBitToComPattern(const int _bit)
{
	switch (_bit)
	{
	case 0b0110:
		return EComPattern::FRONT;
		break;
	case 0b0001:
		return EComPattern::FRONT_LEFT;
		break;
	case 0b0010:
		return EComPattern::BACK_LEFT;
		break;
	case 0b0011:
		return EComPattern::BACK;
		break;
	case 0b0100:
		return EComPattern::BACK_RIGHT;
		break;
	case 0b0101:
		return EComPattern::FRONT_RIGHT;
		break;
	case 0b1000:
		return EComPattern::CENTER_FRONT;
		break;
	case 0b0111:
		return EComPattern::CENTER_BACK;
		break;
	default:
		return EComPattern();
		break;
	}
}

bool ComType::isAbleCoM(const int _com_pattern, const bool _ground_leg[HexapodConst::LEG_NUM])
{
	//�d�S�ʒu�̃p�^�[����v�̊֌W�@�@v=18,19,20�D�D�D�@�̂Ƃ��p�^�[��1�̏d�S�ʒu�͎��Ȃ��D�����ɏ�����Ă���ԍ��̋r��Ԃ͎��Ȃ��B
	const char _comType1[9] = { 18, 19,             23, 24, 25,             29, 30, 31,             35 };	//�p�^�[��6
	const char _comType2[9] = { 18, 19, 20,             24, 25, 26,             30, 31, 32, };				//�p�^�[��1
	const char _comType3[9] = { 19, 20, 21,             25, 26, 27,             31, 32, 33, };			//�p�^�[��2
	const char _comType4[9] = { 20, 21, 22,             26, 27, 28,             32, 33, 34, };		//�p�^�[��3
	const char _comType5[9] = { 21, 22, 23,             27, 28, 29,             33, 34, 35 };	//�p�^�[��4
	const char _comType6[9] = { 18,             22, 23, 24,             28, 29, 30,             34, 35 };	//�p�^�[��5
	const char _comType7[9] = { 18,     20,     22,     24,     26,     28,     30,     32,     34, };		//�p�^�[��7
	const char _comType8[9] = { 19,     21,     23,     25,     27,     29,     31,     33,     35 };	//�p�^�[��8
	const char _comType0[18] = { 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35 };	//�p�^�[��0 �ǂׂ̗肠���������グ�邱�Ƃ��ł��Ȃ�
	const char* _comType[9] = { _comType0, _comType1, _comType2, _comType3, _comType4, _comType5, _comType6, _comType7, _comType8 };

	int n = 9;

	if (_com_pattern == 0)
	{
		n = 18;
	}

	char _com_type = getComTypeFromGroundLeg(_ground_leg);

	for (int i = 0; i < n; i++)
	{
		if (_comType[_com_pattern][i] == _com_type)
		{
			return false;
		}
	}

	return true;
}

char ComType::getComTypeFromGroundLeg(const bool _ground_leg[HexapodConst::LEG_NUM])
{
	// PassFinding �̕ϐ� iHX2�������Ă������́D�܂��CinitiHX2();�������Ă���.

	int _compare_bit = 0;	//�l���r���邽�߂�bool��bit�ɕϊ�����

	//�E�オ�ŏ�ʂ�bit�C�������玞�v���ɉ���bit�Ƀf�[�^������D
	for (int i = HexapodConst::LEG_NUM - 1; i >= 0; i -= 1)
	{
		//�ڒn���Ă���Ȃ��
		if (_ground_leg[i] == true)
		{
			// i�Ԗڂ�bit�𗧂Ă�
			_compare_bit |= (1 << i);
		}
	}

	return getComTypeFromBit(_compare_bit);
}

char ComType::getComTypeFromLegState(const int _leg_state)
{
	// PassFinding �̕ϐ� iHX2�������Ă������́D�܂��CinitiHX2();�������Ă���.

	int _compare_bit = 0;	//�l���r���邽�߂�bool��bit�ɕϊ�����

	//�E�オ�ŏ�ʂ�bit�C�������玞�v���ɉ���bit�Ƀf�[�^������D
	for (int i = HexapodConst::LEG_NUM - 1; i >= 0; i -= 1)
	{
		//�ڒn���Ă���Ȃ��
		if (dl_leg::isGrounded(_leg_state, i) == true)
		{
			// i�Ԗڂ�bit�𗧂Ă�
			_compare_bit |= (1 << i);
		}
	}

	return getComTypeFromBit(_compare_bit);
}

char ComType::getComTypeFromBit(const int _bit)
{
	//�E�オ�ŏ��bit�C���ꂩ�玞�v���ɉ��ʃr�b�g�Ƀf�[�^�������Ă���D

	switch (_bit)
	{
	case 0b111111:
		return  0;		//HX0
	case 0b101111:
		return  1;		//HX1
	case 0b110111:
		return  2;		//HX2
	case 0b111011:
		return  3;		//HX3
	case 0b111101:
		return  4;		//HX4
	case 0b111110:
		return  5;		//HX5
	case 0b011111:
		return  6;		//HX6
	case 0b011011:
		return  7;		//HX7
	case 0b110110:
		return  8;		//HX8
	case 0b101101:
		return  9;		//HX9
	case 0b101011:
		return  10;	//HX10
	case 0b110101:
		return  11;	//HX11
	case 0b111010:
		return  12;	//HX12
	case 0b011101:
		return  13;	//HX13
	case 0b101110:
		return  14;	//HX14
	case 0b010111:
		return  15;	//HX15
	case 0b101010:
		return  16;	//HX16
	case 0b010101:
		return  17;	//HX17
	case 0b001111:
		return  18;	//HX18
	case 0b100111:
		return  19;	//HX19
	case 0b110011:
		return  20;	//HX20
	case 0b111001:
		return  21;	//HX21
	case 0b111100:
		return  22;	//HX22
	case 0b011110:
		return  23;	//HX23
	case 0b001101:
		return  24;	//HX24
	case 0b100110:
		return  25;	//HX25
	case 0b010011:
		return  26;	//HX26
	case 0b101001:
		return  27;	//HX27
	case 0b110100:
		return  28;	//HX28
	case 0b011010:
		return  29;	//HX29
	case 0b001011:
		return  30;	//HX30
	case 0b100101:
		return  31;	//HX31
	case 0b110010:
		return  32;	//HX32
	case 0b011001:
		return  33;	//HX33
	case 0b101100:
		return  34;	//HX34
	case 0b010110:
		return  35;	//HX35
	default:
		break;
	}

	//�Y�����Ȃ��Ȃ�Ε��̒l��Ԃ�
	return -1;
}

void ComType::getGroundLegFromComType(const int _com_type, bool _output_ground_leg[HexapodConst::LEG_NUM])
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

void ComType::getDonotUseComTypeFromComPattern(const int _com_pattern, std::vector<int> _output)
{
	//com pattern����g�p�s�\��com type���擾���ďo�͂���D
	for (const auto& i : COMTYPE_BAN_LIST[_com_pattern])
	{
		_output.push_back(i);
	}
}

void ComType::checkAbleComTypeFromComPattern(const int _com_pattern, bool _com_type_able_array[COM_TYPE_NUM])
{
	std::vector<int> _cannot_use_type;
	getDonotUseComTypeFromComPattern(_com_pattern, _cannot_use_type);

	for (const auto& i : _cannot_use_type)
	{
		_com_type_able_array[i] = false;
	}
}

void ComType::checkAbleComTypeFromNotGroundableLeg(const int _not_groundble_leg, bool _com_type_able_array[COM_TYPE_NUM])
{
	//�Ƃ邱�Ƃ̂ł��Ȃ�com type��S��false�ɕύX����D
	for (int i = 0; i < BAN_LIST_ARRAY_SIZE; i++)
	{
		_com_type_able_array[BAN_LIST[_not_groundble_leg][i]] = false;
	}
}

void ComType::checkAbleComTypeFromNotFreeLeg(const int _not_free_leg, bool _com_type_able_array[COM_TYPE_NUM])
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