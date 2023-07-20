#include "LegUpDownNodeCreator.h"
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
					_res_node.leg_pos[j].x = (HexapodConst::COXA_LENGTH + HexapodConst::FEMUR_LENGTH) * cos(HexapodConst::DEFAULT_LEG_ANGLE[j]);
					_res_node.leg_pos[j].y = (HexapodConst::COXA_LENGTH + HexapodConst::FEMUR_LENGTH) * sin(HexapodConst::DEFAULT_LEG_ANGLE[j]);
					_res_node.leg_pos[j].z = -40;
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

	std::vector<SVector> _candidate_pos;		//���݂̋r�ʒu�ɍ��v��������W�Q�D
	const SVector _leg_pos = m_Calc.getGlobalLeg2Pos(_current_node, _leg_num);		//���U����������4�̍��W�����炩���ߌv�Z���Ă���
	const SVector _coxa_pos = m_Calc.getGlobalCoxaJointPos(_current_node, _leg_num);	//�r�̕t�����̍��W�D
	const int _leg_state = LegStateEdit::getLegState(_current_node.leg_state, _leg_num);			//�r�ʒu���擾(1�`7)

	//�͈͓��̓_��S�Ē��ׂ�D
	for (int x = _min_x_dev; x < _max_x_dev; x++)
	{
		for (int y = _min_y_dev; y < _max_y_dev; y++)
		{
			const int _pos_num = mp_Map->getPointNumFromDevideMap(x, y);

			for (int n = 0; n < _pos_num; n++)
			{
				SVector _pos = mp_Map->getPosFromDevideMap(x, y, n);	//�r�ݒu�\�_�̍��W�����o���D

				//�r�ʒu���X�V�����m�[�h���쐬����D
				SNode _new_node = _current_node;

				_new_node.leg_pos[_leg_num] = _pos - _coxa_pos;	//�r�ݒu�\�_���r�̕t�����ɍ��킹��D

				if (m_Calc.isLegInRange(_new_node, _leg_num) == false) { continue; }			//�r���͈͊O�Ȃ�Βǉ������ɑ��s�D

				if (m_Calc.isAblePause(_new_node) == false) { continue; }						//�]�Ԏp���Ȃ�Βǉ������ɑ��s�D

				if (m_Calc.isLegInterfering(_new_node) == true) { continue; }					//�r�������Ă���Ȃ�Βǉ������ɑ��s�D

				if (isAbleLegPos(_leg_pos, _pos, _coxa_pos, _leg_state) == false) { continue; }	//�����W�Ƃ��āC�K���Ă��Ȃ��Ȃ�Βǉ������ɑ��s�D


				_candidate_pos.push_back(_pos);
			}
		}
	}


	//���_��S�񋓂����̂��C���_������Ȃ����false
	if (_candidate_pos.size() == 0) { return false; }

	//���݂���Ȃ�C���̒��ōł��K�������̂����ʂƂ��ĕԂ��Ctrue
	_output_ground_pos = m_Calc.convertLocalLegPos(_current_node, _candidate_pos.front(), _leg_num);

	return true;
}

bool LegUpDownNodeCreator::isAbleLegPos(const my_vec::SVector& _4pos, const my_vec::SVector& _candiatepos, const my_vec::SVector& _coxapos, const int _leg_state)
{
	//�܂��ŏ��ɋr�ʒu4�̂Ƃ���ɂȂ����m���߂�D
	if ((_4pos - _candiatepos).lengthSquare() < my_math::squared(LEG_MARGIN))
	{
		if (_leg_state == 4) { return true; }
		else { return false; }
	}
	else
	{
		if (_leg_state == 4) { return false; }
	}

	//�r�ʒu4�Ɣ�r���đO����납
	my_vec::SVector2 _front_vec = { 1,0 };	//@todo ��]�p�����l������Ȃ�΁C���̃x�N�g������]������K�v������D

	if (_leg_state == 7 || _leg_state == 6 || _leg_state == 5)
	{
		if ((_4pos - _coxapos).projectedXY().cross(_front_vec) * (_candiatepos - _coxapos).projectedXY().cross(_front_vec) > 0) {}
		else { return false; }
	}
	else
	{
		if ((_4pos - _coxapos).projectedXY().cross(_front_vec) * (_candiatepos - _coxapos).projectedXY().cross(_front_vec) < 0) {}
		else { return false; }
	}


	//�r�ʒu4�Ɣ�r���ďォ����
	if (_leg_state == 1 || _leg_state == 5)
	{
		//�r�ʒu4�Ɣ�r���ĉ�
		if (_4pos.z - HIGH_MARGIN >= _candiatepos.z)
		{
			return true;
		}
	}
	else if (_leg_state == 3 || _leg_state == 7)
	{
		//�r�ʒu4�Ɣ�r���ď�
		if (_4pos.z + HIGH_MARGIN <= _candiatepos.z)
		{
			return true;
		}
	}
	else
	{
		//�r�ʒu4�Ɠ������炢
		if (std::abs(_4pos.z - _candiatepos.z) <= HIGH_MARGIN)
		{
			return true;
		}
	}

	return false;
}
