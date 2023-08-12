#include "leg_down_node_creator.h"

#include "leg_state.h"
#include "com_type.h"


void LegDownNodeCreator::create(const SNode& current_node, const int current_num, std::vector<SNode>* output_graph)
{
	//�r�̗V�r�E�ڒn�ɂ���Đ�����Ƃ肤��d�S��comtype�Ƃ��Ďd�����Ă���D(�ڂ�����Comtype.h���Q��)�D�܂��͑S��true�ɂ��Ă����D
	bool is_able_type[ComType::COM_TYPE_NUM];

	for (int i = 0; i < ComType::COM_TYPE_NUM; i++)
	{
		is_able_type[i] = true;
	}


	//�r���n�ʂɐڒn�\�����ׂ�D

	bool is_groundable_leg[HexapodConst::LEG_NUM];			//�r���ݒu�\�Ȃ��true�ɂȂ�D���ɐڒn���Ă���Ȃ��true�ɂȂ�D
	my_vec::SVector ground_pos[HexapodConst::LEG_NUM];	//�r���ڒn������W�D

	for (int i = 0; i < HexapodConst::LEG_NUM; i++) { ground_pos[i] = current_node.leg_pos[i]; }

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (dl_leg::isGrounded(current_node.leg_state, i))
		{
			//���łɐڒn���Ă���r�͐ڒn�\�Ɍ��܂��Ă���̂�true�ɂ��C���W�����̂܂ܓ����D
			is_groundable_leg[i] = true;
			ground_pos[i] = current_node.leg_pos[i];

			//�������낷�����������̂ŁC�ڒn�r�͗V�r�s�\�D����āC�Ƃ�Ȃ�com type��S�Ă����D
			ComType::checkAbleComTypeFromNotFreeLeg(i, is_able_type);
		}
		else
		{
			//���ݗV�r���̋r�͌��݂̋r��ԂŐڒn�ł��邩��������D
			my_vec::SVector res_ground_pos;

			if (isGroundableLeg(i, current_node, res_ground_pos))
			{
				is_groundable_leg[i] = true;	//�ڒn�\�ɂ���D
				ground_pos[i] = res_ground_pos;
			}
			else
			{
				is_groundable_leg[i] = false;	//�ڒn�s�\�ɂ���D
				ComType::checkAbleComTypeFromNotGroundableLeg(i, is_able_type);	//�ڒn�s�\�ȋr�ɂ���āC�Ƃ�Ȃ�com type��S�Ă����D
			}
		}
	}

	//�q�m�[�h�𐶐�����D
	for (int i = 0; i < ComType::COM_TYPE_NUM; ++i)
	{
		//���̏d�S�^�C�v����邱�Ƃ��\�ł����
		if (is_able_type[i])
		{
			SNode res_node = current_node;
			res_node.changeNextNode(current_num, m_next_move);

			//�V�r�E�ڒn������������D
			bool is_ground_list[HexapodConst::LEG_NUM] = {};
			ComType::getGroundLegFromComType(i, is_ground_list);

			for (int j = 0; j < HexapodConst::LEG_NUM; ++j)
			{
				dl_leg::changeGround(res_node.leg_state, j, is_ground_list[j]);

				if (is_ground_list[j])
				{
					res_node.leg_pos[j] = ground_pos[j];
				}
				else
				{
					res_node.leg_pos[j].x = 160 * HexapodConst::DEFAULT_LEG_ANGLE_COS[j];
					res_node.leg_pos[j].y = 160 * HexapodConst::DEFAULT_LEG_ANGLE_SIN[j];
					res_node.leg_pos[j].z = -10;
				}
			}

			(*output_graph).push_back(res_node);
		}

	}

	//�o�͂��ꂽ�m�[�h���Ȃ��Ȃ�΁C���̂܂܂̃m�[�h���o�͂���D
	if ((*output_graph).size() == 0)
	{
		SNode same_node = current_node;

		same_node.changeNextNode(current_num, m_next_move);

		(*output_graph).emplace_back(same_node);
	}
}


bool LegDownNodeCreator::isGroundableLeg(const int _leg_num, const SNode& _current_node, my_vec::SVector& _output_ground_pos)
{
	//for���̒���continue�ɂ��Ă� http://www9.plala.or.jp/sgwr-t/c/sec06-7.html ���Q�ƁD���Ȃ݂ɓǂ݂Â炭�Ȃ�̂Ŗ{���͎g��Ȃ��ق��������D

	using my_vec::SVector;

	if (mp_map == nullptr) { return false; }	//�}�b�v���Ȃ��Ƃ���false��Ԃ��D

	//�r���W��devide map�łǂ��ɓ����邩���ׂāC���̃}�X��2���2���͈͓̔���S�ĒT������D
	const my_vec::SVector kGlobalLegBasePos = m_calculator.getGlobalLegBasePos(_current_node, _leg_num, false);

	int max_x_dev = mp_map->getDevideMapNumX(kGlobalLegBasePos.x) + 1;
	int min_x_dev = mp_map->getDevideMapNumX(kGlobalLegBasePos.x) - 1;
	int max_y_dev = mp_map->getDevideMapNumY(kGlobalLegBasePos.y) + 1;
	int min_y_dev = mp_map->getDevideMapNumY(kGlobalLegBasePos.y) - 1;

	////�l��devide map�͈̔͊O�ɂ���Ƃ��͊ۂ߂�D
	max_x_dev = (max_x_dev >= MapConst::LP_DIVIDE_NUM) ? MapConst::LP_DIVIDE_NUM - 1 : max_x_dev;
	min_x_dev = (min_x_dev < 0) ? 0 : min_x_dev;
	max_y_dev = (max_y_dev >= MapConst::LP_DIVIDE_NUM) ? MapConst::LP_DIVIDE_NUM - 1 : max_y_dev;
	min_y_dev = (min_y_dev < 0) ? 0 : min_y_dev;

	//devide map����S�T�����āC���݂̋r�ʒu(���U��������)�ɓK�����r�ݒu�\�_�����݂��邩���ׂ�D

	std::vector<my_vec::SVector> _candidate_pos;		//���݂̋r�ʒu�ɍ��v��������W�Q�D

	//�͈͓��̓_��S�Ē��ׂ�D
	for (int x = min_x_dev; x < max_x_dev; x++)
	{
		for (int y = min_y_dev; y < max_y_dev; y++)
		{
			const int kPosNum = mp_map->getPointNumFromDevideMap(x, y);

			for (int n = 0; n < kPosNum; n++)
			{
				SVector _pos = mp_map->getPosFromDevideMap(x, y, n);	//�r�ݒu�\�_�̍��W�����o���D
				_pos = m_calculator.convertLocalLegPos(_current_node, _pos, _leg_num, false);

				//�r�ʒu���X�V�����m�[�h���쐬����D
				SNode _new_node = _current_node;

				_new_node.leg_pos[_leg_num] = _pos;


				//�O�̌��n�_�Ɣ�r���āC���ǂ����n�_�̎��̂ݎ��s������
				if (_candidate_pos.empty() == false)
				{
					//���Ε������ނ��Ă���ꍇ�͌��n�_�Ƃ��č̗p���Ȃ��D
					if (_new_node.leg_base_pos[_leg_num].projectedXY().cross(_candidate_pos.front().projectedXY()) * _new_node.leg_base_pos[_leg_num].projectedXY().cross(_pos.projectedXY()) < 0)
					{
						continue;
					}
				}

				dl_leg::changeGround(_new_node.leg_state, _leg_num, true);

				if (m_calculator.isLegInRange(_new_node, _leg_num) == false) { continue; }			//�r���͈͊O�Ȃ�Βǉ������ɑ��s�D

				if (isAbleLegPos(_new_node, _leg_num) == false) { continue; }	//�����W�Ƃ��āC�K���Ă��Ȃ��Ȃ�Βǉ������ɑ��s�D

				_candidate_pos.push_back(_pos);
			}
		}
	}


	//���_��S�񋓂����̂��C���_������Ȃ����false
	if (_candidate_pos.empty() == true) { return false; }

	_output_ground_pos = _candidate_pos.back();

	return true;
}

bool LegDownNodeCreator::isAbleLegPos(const SNode& _node, const int _leg_num)
{
	int _leg_state = dl_leg::getLegState(_node.leg_state, _leg_num);		//�r�ʒu���擾(1�`7)

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
