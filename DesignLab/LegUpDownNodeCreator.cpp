#include "LegUpDownNodeCreator.h"
#include "ComType.h"
#include "LegState.h"
#include <algorithm>

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
	//ComType::checkAbleComTypeFromComPattern(_com_pattern, _is_able_type);



	//���ɋr���n�ʂɐڒn�\�����ׂ�D

	bool _is_groundable[HexapodConst::LEG_NUM];			//�r���ݒu�\�Ȃ��true�ɂȂ�D���ɐڒn���Ă���Ȃ��true�ɂȂ�D
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
			_res_node.changeNextNode(_current_num, getNextMove(_current_node.next_move));

			//�V�r�E�ڒn������������D
			bool _temp_ground[HexapodConst::LEG_NUM] = {};
			ComType::getGroundLegFromComType(i, _temp_ground);

			for (int j = 0; j < HexapodConst::LEG_NUM; j++)
			{
				LegStateEdit::changeGround(_res_node.leg_state, j, _temp_ground[j]);

				if (_temp_ground[j] == true)
				{
					_res_node.leg_pos[j] = _ground_pos[j];
				}
				else
				{
					_res_node.leg_pos[j].x = 160 * HexapodConst::DEFAULT_LEG_ANGLE_COS[j];
					_res_node.leg_pos[j].y = 160 * HexapodConst::DEFAULT_LEG_ANGLE_SIN[j];
					_res_node.leg_pos[j].z = -10;
				}
			}

			_output_graph.push_back(_res_node);
		}

	}

	//�o�͂����O���t��1�����Ȃ��(���g�Ɠ����O���t������������Ȃ��ꍇ��)�O���t��ǉ����Ȃ��D
	if (_output_graph.size() == 1) { _output_graph.clear(); }
}

bool LegUpDownNodeCreator::isGroundableLeg(const int _leg_num, const SNode& _current_node, my_vec::SVector& _output_ground_pos)
{
	//for���̒���continue�ɂ��Ă� http://www9.plala.or.jp/sgwr-t/c/sec06-7.html ���Q�ƁD���Ȃ݂ɓǂ݂Â炭�Ȃ�̂Ŗ{���͎g��Ȃ��ق��������D

	using my_vec::SVector;

	if (mp_Map == nullptr) { return false; }	//�}�b�v���Ȃ��Ƃ���false��Ԃ��D

	//�r���W��devide map�łǂ��ɓ����邩���ׂāC���̃}�X��2���2���͈͓̔���S�ĒT������D
	const my_vec::SVector LEG_POS = m_Calc.getGlobalLegPos(_current_node, _leg_num, false);

	int _max_x_dev = mp_Map->getDevideMapNumX(LEG_POS.x) + 1;
	int _min_x_dev = mp_Map->getDevideMapNumX(LEG_POS.x) - 1;
	int _max_y_dev = mp_Map->getDevideMapNumY(LEG_POS.y) + 1;
	int _min_y_dev = mp_Map->getDevideMapNumY(LEG_POS.y) - 1;

	//�l��devide map�͈̔͊O�ɂ���Ƃ��͊ۂ߂�D
	_max_x_dev = (_max_x_dev >= MapConst::LP_DIVIDE_NUM) ? MapConst::LP_DIVIDE_NUM - 1 : _max_x_dev;
	_min_x_dev = (_min_x_dev < 0) ? 0 : _min_x_dev;
	_max_y_dev = (_max_y_dev >= MapConst::LP_DIVIDE_NUM) ? MapConst::LP_DIVIDE_NUM - 1 : _max_y_dev;
	_min_y_dev = (_min_y_dev < 0) ? 0 : _min_y_dev;


	//devide map����S�T�����āC���݂̋r�ʒu(���U��������)�ɓK�����r�ݒu�\�_�����݂��邩���ׂ�D

	my_vec::SVector _candidate_pos;		//���݂̋r�ʒu�ɍ��v��������W�Q�D
	bool _is_candidate_pos = false;		//�����W�����݂��邩�ǂ����D

	//�͈͓��̓_��S�Ē��ׂ�D
	for (int x = _min_x_dev; x < _max_x_dev; x++)
	{
		for (int y = _min_y_dev; y < _max_y_dev; y++)
		{
			const int _pos_num = mp_Map->getPointNumFromDevideMap(x, y);

			for (int n = 0; n < _pos_num; n++)
			{
				SVector _pos = mp_Map->getPosFromDevideMap(x, y, n);	//�r�ݒu�\�_�̍��W�����o���D
				_pos = m_Calc.convertLocalLegPos(_current_node, _pos, _leg_num);

				//�r�ʒu���X�V�����m�[�h���쐬����D
				SNode _new_node = _current_node;

				_new_node.leg_pos[_leg_num] = _pos;


				//�O�̌��n�_�Ɣ�r���āC���ǂ����n�_�̎��̂ݎ��s������
				if (_is_candidate_pos == true)
				{
					if (_new_node.leg_base_pos[_leg_num].projectedXY().cross(_candidate_pos.projectedXY()) * _new_node.leg_base_pos[_leg_num].projectedXY().cross(_pos.projectedXY()) < 0)
					{
						continue;
					}

					if (_pos.projectedXY().cross(_candidate_pos.projectedXY()) * _pos.projectedXY().cross(_new_node.leg_base_pos[_leg_num].projectedXY()) < 0)
					{
						continue;
					}
				}

				LegStateEdit::changeGround(_new_node.leg_state, _leg_num, true);

				if (m_Calc.isLegInRange(_new_node, _leg_num) == false) { continue; }			//�r���͈͊O�Ȃ�Βǉ������ɑ��s�D

				//if (m_Calc.isLegInterfering(_new_node) == true) { continue; }					//�r�������Ă���Ȃ�Βǉ������ɑ��s�D

				//if (m_Calc.isAblePause(_new_node) == false) { continue; }						//�r���n�ʂɂ��Ă���Ȃ�Βǉ������ɑ��s�D

				if (isAbleLegPos(_new_node, _leg_num) == false) { continue; }	//�����W�Ƃ��āC�K���Ă��Ȃ��Ȃ�Βǉ������ɑ��s�D

				_is_candidate_pos = true;
				_candidate_pos = _pos;
			}
		}
	}


	//���_��S�񋓂����̂��C���_������Ȃ����false
	if (_is_candidate_pos == false) { return false; }

	//���݂���Ȃ�C���̒��ōł��K�������̂����ʂƂ��ĕԂ��Ctrue
	_output_ground_pos = _candidate_pos;

	return true;
}

bool LegUpDownNodeCreator::isAbleLegPos(const SNode& _node, const int _leg_num)
{
	int _leg_state = LegStateEdit::getLegState(_node.leg_state, _leg_num);		//�r�ʒu���擾(1�`7)

	//�܂��ŏ��ɋr�ʒu4�̂Ƃ���ɂȂ����m���߂�D
	if ((_node.leg_base_pos[_leg_num] - _node.leg_pos[_leg_num]).lengthSquare() < my_math::squared(LEG_MARGIN))
	{
		if (_leg_state == 4) { return true; }
		else { return false; }
	}
	else
	{
		if (_leg_state == 4) { return false; }
	}

	//�r�ʒu4�Ɣ�r���đO����납
	if (_node.leg_base_pos[_leg_num].projectedXY().cross(_node.leg_pos[_leg_num].projectedXY()) * _node.leg_pos[_leg_num].projectedXY().cross({ 1,0 }) > 0)
	{
		//�O

		if (_leg_state == 1 || _leg_state == 2 || _leg_state == 3)
		{
			return false;
		}
	}
	else
	{
		//���

		if (_leg_state == 7 || _leg_state == 6 || _leg_state == 5)
		{
			return false;
		}
	}


	//�r�ʒu4�Ɣ�r���ďォ����
	if (_leg_state == 1 || _leg_state == 5)
	{
		//�r�ʒu4�Ɣ�r���ĉ�
		if (_node.leg_base_pos[_leg_num].z - HIGH_MARGIN >= _node.leg_pos[_leg_num].z)
		{
			return true;
		}
	}
	else if (_leg_state == 3 || _leg_state == 7)
	{
		//�r�ʒu4�Ɣ�r���ď�
		if (_node.leg_base_pos[_leg_num].z + HIGH_MARGIN <= _node.leg_pos[_leg_num].z)
		{
			return true;
		}
	}
	else
	{
		//�r�ʒu4�Ɠ������炢
		if (std::abs(_node.leg_base_pos[_leg_num].z - _node.leg_pos[_leg_num].z) <= HIGH_MARGIN)
		{
			return true;
		}
	}

	return false;
}
