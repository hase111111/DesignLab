#include "LegUpDownNodeCreator.h"
#include "NodeEdit.h"
#include "ComType.h"
#include "LegState.h"

void LegUpDownNodeCreator::init(const MapState* const _p_Map)
{
	mp_Map = _p_Map;
}

void LegUpDownNodeCreator::create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph)
{
	//�r�̗V�r�E�ڒn�ɂ���Đ�����Ƃ肤��d�S��comtype�Ƃ��Ďd�����Ă���D(�ڂ�����Comtype.h���Q��)�D�܂��͑S��true�ɂ��Ă����D
	bool _is_able_type[ComType::COM_TYPE_NUM];

	for (int i = 0; i < ComType::COM_TYPE_NUM; i++)
	{
		_is_able_type[i] = true;
	}

	//�d�S�����݂ǂ��ɂ��邩(�O��肩�^�񒆂�...)�Ȃǂ̃p�����[�^�͂���com pattern�Ŏd�����Ă���D(�ڂ�����Comtype.h���Q��)�D������擾����D
	int _com_pattern = LegStateEdit::getComPatternState(_current_node.leg_state);

	//com pattern���Ƃ邱�Ƃ��ł��Ȃ�com type��S��false�ɂ���D
	ComType::checkAbleComTypeFromComPattern(_com_pattern, _is_able_type);



	//���ɋr���n�ʂɐڒn�\�����ׂ�D

	bool _is_groundable[HexapodConst::LEG_NUM];				//�r���ݒu�\�Ȃ��true�ɂȂ�D���ɐڒn���Ă���Ȃ��true�ɂȂ�D
	my_vec::SVector _ground_pos[HexapodConst::LEG_NUM];	//�r���ڒn������W�D

	for (int i = 0; i < HexapodConst::LEG_NUM; i++) { _ground_pos[i] = _current_node.leg_pos[i]; }

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (LegStateEdit::isGrounded(_current_node.leg_state, i) == true)
		{
			//���łɐڒn���Ă���r�͐ڒn�\�Ɍ��܂��Ă���̂�true�ɂ���D
			_is_groundable[i] = true;
			_ground_pos[i] = _current_node.leg_pos[i];
		}
		else
		{
			//���ݗV�r���̋r�͎��g�̋r��ԂŐڒn�ł��邩��������D
			my_vec::SVector _res_ground_pos;

			if (isGroundableLeg(i, _current_node, _res_ground_pos) == true)
			{
				_is_groundable[i] = true;	//�ڒn�\�ɂ���D
				_ground_pos[i] = _res_ground_pos;
			}
			else
			{
				_is_groundable[i] = false;	//�ڒn�s�\�ɂ���D
				ComType::checkAbleComTypeFromNotGroundableLeg(i, _is_able_type);	//�ڒn�s�\�ȋr�ɂ���āC�Ƃ�Ȃ�com type��S�Ă����D
			}
		}
	}


	//�q�m�[�h�𐶐�����D
	for (int i = 0; i < ComType::COM_TYPE_NUM; i++)
	{
		//���̏d�S�^�C�v���\�ł���΁C
		if (_is_able_type[i] == true)
		{
			SNode _res_node = _current_node;
			node_edit::changeNextNode(_res_node, _current_num, getNextMove(_current_node.next_move));

			//�V�r�E�ڒn������������D
			bool _temp_ground[HexapodConst::LEG_NUM] = {};
			ComType::getGroundLegFromComType(i, _temp_ground);

			for (int l = 0; l < HexapodConst::LEG_NUM; l++)
			{
				LegStateEdit::changeGround(_res_node.leg_state, l, _temp_ground[l]);

				_res_node.leg_pos[l] = _ground_pos[l];

				if (_temp_ground[l] == false) { _res_node.leg_pos[l].z = -40; }
			}

			_output_graph.push_back(_res_node);
		}

	}
}

EHexapodMove LegUpDownNodeCreator::getNextMove(const EHexapodMove& _last_move) const
{
	//�d�S�̏㉺�ړ������ꁨ�d�S�̕��s�ړ�
	//�r�̕��s�ړ������ꁨ�d�S�̏㉺�ړ�
	return EHexapodMove::LEG_HIERARCHY_CHANGE;

	if (_last_move == EHexapodMove::COM_UP_DOWN) { return EHexapodMove::COM_MOVE; }
	else { return EHexapodMove::COM_UP_DOWN; }
}

bool LegUpDownNodeCreator::isGroundableLeg(const int _leg_num, const SNode& _current_node, my_vec::SVector& _output_ground_pos)
{
	if (mp_Map == nullptr) { return false; }

	//�r���W��devide map�łǂ��ɓ����邩���ׂāC���̃}�X��2���2���͈͓̔���S�ĒT������D
	int _max_x_dev = mp_Map->getDevideMapNumX(_current_node.leg_pos[_leg_num].x) + 2;
	int _min_x_dev = mp_Map->getDevideMapNumX(_current_node.leg_pos[_leg_num].x) - 2;
	int _max_y_dev = mp_Map->getDevideMapNumY(_current_node.leg_pos[_leg_num].y) + 2;
	int _min_y_dev = mp_Map->getDevideMapNumY(_current_node.leg_pos[_leg_num].y) - 2;

	//�l��devide map�͈̔͊O�ɂ���Ƃ��͊ۂ߂�D
	_max_x_dev = (_max_x_dev >= MapConst::LP_DIVIDE_NUM) ? MapConst::LP_DIVIDE_NUM - 1 : _max_x_dev;
	_min_x_dev = (_min_x_dev < 0) ? 0 : _min_x_dev;
	_max_y_dev = (_max_y_dev >= MapConst::LP_DIVIDE_NUM) ? MapConst::LP_DIVIDE_NUM - 1 : _max_y_dev;
	_min_y_dev = (_min_y_dev < 0) ? 0 : _min_y_dev;


	//devide map����S�T�����āC���݂̋r�ʒu(���U��������)�ɓK�����r�ݒu�\�_�����݂��邩���ׂ�D

	std::vector<my_vec::SVector> _candidate_pos;		//���݂̋r�ʒu�ɍ��v��������W�Q�D
	const my_vec::SVector _leg_pos = m_Calc.getGlobalLeg2Pos(_current_node, _leg_num);		//���U����������4�̍��W�����炩���ߌv�Z���Ă���
	const my_vec::SVector _coxa_pos = m_Calc.getGlobalCoxaJointPos(_current_node, _leg_num);	//�r�̕t�����̍��W�D
	const int _leg_state = LegStateEdit::getLegState(_current_node.leg_state, _leg_num);			//�r�ʒu���擾(1�`7)

	//�͈͓��̓_��S�Ē��ׂ�D
	for (int x = _min_x_dev; x < _max_x_dev; x++)
	{
		for (int y = _min_y_dev; y < _max_y_dev; y++)
		{
			const int _pos_num = mp_Map->getPointNumFromDevideMap(x, y);

			for (int n = 0; n < _pos_num; n++)
			{
				//�r�ݒu�\�_�̍��W�����o���D
				my_vec::SVector _pos = mp_Map->getPosFromDevideMap(x, y, n);

				float _len = (_coxa_pos - _pos).length();	//�t��������r�ݒu�\�_�܂ł̒������擾����D

				//�ŏ����߂��C�܂��́C�ő��艓���Ȃ�΁C�ǉ������ɑ��s�Dcontinue�ɂ��Ă� http://www9.plala.or.jp/sgwr-t/c/sec06-7.html ���Ȃ݂ɓǂ݂Â炭�Ȃ�̂Ŗ{���͎g��Ȃ��ق��������ł��D
				if (_len < m_Calc.getMinLegR(_coxa_pos.z - _pos.z) || m_Calc.getMaxLegR(_coxa_pos.z - _pos.z) < _len)
				{
					continue;
				}

				//�����W�Ƃ��āC�K���Ă��Ȃ��Ȃ�Βǉ������ɑ��s�D
				if (isAbleLegPos(_leg_pos, _pos, _coxa_pos, _leg_state) == false)
				{
					continue;
				}

				_candidate_pos.push_back(_pos);
			}
		}
	}


	//���_��S�񋓂����̂��C���_������Ȃ����false
	if (_candidate_pos.size() == 0) { return false; }

	_output_ground_pos = m_Calc.getLocalLegPos(_current_node, _candidate_pos.front(), _leg_num);

	//���݂���Ȃ�C���̒��ōł��K�������̂����ʂƂ��ĕԂ��Ctrue
	return true;
}

bool LegUpDownNodeCreator::isAbleLegPos(const my_vec::SVector& _4pos, const my_vec::SVector& _candiatepos, const my_vec::SVector& _coxapos, const int _leg_state)
{
	//Leg2�Ɣ�r���āC�ǂ��ɂ��邩�ɂ���Ĉȉ��̂悤�ɗ��U�����Ă���D
	switch (_leg_state)
	{
	case 1:
		//���C��

		//�\���߂��Ȃ�΁C���U�������r�ʒu 4 �̏ꏊ�Ȃ̂œK���Ȃ��Dfalse
		if (LEG_MARGIN > (_4pos - _candiatepos).length()) { return false; }

		//���U�������r�ʒu 4 ��艺�ɂ��邩�m���߂�D
		if (_4pos.z - HIGH_MARGIN > _candiatepos.z)
		{
			//�x�N�g���̊O��a�~b��a��b�Ɖ�]����E�˂��̏�����ɂȂ�D�r�ʒu 4��a�C���_��b�Ƃ����Ƃ��ɁC�������ɂȂ�Ȃ�Ό��ɂ���͂��D
			if ((_4pos - _coxapos).cross(_candiatepos - _coxapos).z < 0)
			{
				return true;
			}
		}

		break;

	case 2:
		//���C��

		//�\���߂��Ȃ�΁C���U�������r�ʒu 4 �̏ꏊ�Ȃ̂œK���Ȃ��Dfalse
		if (LEG_MARGIN > (_4pos - _candiatepos).length()) { return false; }

		//���U�������r�ʒu 4 �Ɠ��������ɂ��邩�m���߂�D
		if (_4pos.z - HIGH_MARGIN < _candiatepos.z && _candiatepos.z < _4pos.z + HIGH_MARGIN)
		{
			//�x�N�g���̊O��a�~b��a��b�Ɖ�]����E�˂��̏�����ɂȂ�D�r�ʒu 4��a�C���_��b�Ƃ����Ƃ��ɁC�������ɂȂ�Ȃ�Ό��ɂ���͂��D
			if ((_4pos - _coxapos).cross(_candiatepos - _coxapos).z < 0)
			{
				return true;
			}
		}

		break;

	case 3:
		//���C��

		//�\���߂��Ȃ�΁C���U�������r�ʒu 4 �̏ꏊ�Ȃ̂œK���Ȃ��Dfalse
		if (LEG_MARGIN > (_4pos - _candiatepos).length()) { return false; }

		//���U�������r�ʒu 4 ����ɂ��邩�m���߂�D
		if (_4pos.z + HIGH_MARGIN < _candiatepos.z)
		{
			//�x�N�g���̊O��a�~b��a��b�Ɖ�]����E�˂��̏�����ɂȂ�D�r�ʒu 4��a�C���_��b�Ƃ����Ƃ��ɁC�������ɂȂ�Ȃ�Ό��ɂ���͂��D
			if ((_4pos - _coxapos).cross(_candiatepos - _coxapos).z < 0)
			{
				return true;
			}
		}

		break;

	case 4:
		//�^��

		//�\���߂��Ȃ�΁C�ǉ�����D
		if (LEG_MARGIN > (_4pos - _candiatepos).length()) { return true; }

		break;

	case 5:
		//�@�O�C��

		//�\���߂��Ȃ�΁C���U�������r�ʒu 4 �̏ꏊ�Ȃ̂œK���Ȃ��Dfalse
		if (LEG_MARGIN > (_4pos - _candiatepos).length()) { return false; }

		//���U�������r�ʒu 4 ��艺�ɂ��邩�m���߂�D
		if (_4pos.z - HIGH_MARGIN > _candiatepos.z)
		{
			//�x�N�g���̊O��a�~b��a��b�Ɖ�]����E�˂��̏�����ɂȂ�D�r�ʒu 4��a�C���_��b�Ƃ����Ƃ��ɁC������ɂȂ�Ȃ�ΑO�ɂ���͂��D
			if ((_4pos - _coxapos).cross(_candiatepos - _coxapos).z < 0)
			{
				return true;
			}
		}

		break;

	case 6:
		//�@�O�C��

		//�\���߂��Ȃ�΁C���U�������r�ʒu 4 �̏ꏊ�Ȃ̂œK���Ȃ��Dfalse
		if (LEG_MARGIN > (_4pos - _candiatepos).length()) { return false; }

		//���U�������r�ʒu 4 �Ɠ��������ɂ��邩�m���߂�D
		if (_4pos.z - HIGH_MARGIN < _candiatepos.z && _candiatepos.z < _4pos.z + HIGH_MARGIN)
		{
			//�x�N�g���̊O��a�~b��a��b�Ɖ�]����E�˂��̏�����ɂȂ�D�r�ʒu 4��a�C���_��b�Ƃ����Ƃ��ɁC������ɂȂ�Ȃ�ΑO�ɂ���͂��D
			if ((_4pos - _coxapos).cross(_candiatepos - _coxapos).z < 0)
			{
				return true;
			}
		}

		break;

	case 7:
		//�@�O�C��

		//�\���߂��Ȃ�΁C���U�������r�ʒu 4 �̏ꏊ�Ȃ̂œK���Ȃ��Dfalse
		if (LEG_MARGIN > (_4pos - _candiatepos).length()) { return false; }

		//���U�������r�ʒu 4 ����ɂ��邩�m���߂�D
		if (_4pos.z + HIGH_MARGIN < _candiatepos.z)
		{
			//�x�N�g���̊O��a�~b��a��b�Ɖ�]����E�˂��̏�����ɂȂ�D�r�ʒu 4��a�C���_��b�Ƃ����Ƃ��ɁC������ɂȂ�Ȃ�ΑO�ɂ���͂��D
			if ((_4pos - _coxapos).cross(_candiatepos - _coxapos).z < 0)
			{
				return true;
			}
		}

		break;

	default:
		//�Y�����Ȃ��Ȃ��false
		return false;
		break;

	}

	return false;
}
