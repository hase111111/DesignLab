//#include "leg_down_node_creator.h"
//
//#include "leg_state.h"
//#include "com_type.h"
//
//
//void LegDownNodeCreator::create(const SNode& current_node, const int current_num, std::vector<SNode>* output_graph)
//{
//	//�r�̗V�r�E�ڒn�ɂ���Đ�����Ƃ肤��d�S��comtype�Ƃ��Ďd�����Ă���D(�ڂ�����Comtype.h���Q��)�D�܂��͑S��true�ɂ��Ă����D
//	boost::dynamic_bitset<> is_able_leg_ground_pattern(dl_com::getLegGroundPatternNum());
//
//	is_able_leg_ground_pattern.set();	//�S��true�ɂ���D
//
//
//	//�r���n�ʂɐڒn�\�����ׂ�D
//
//	bool is_groundable_leg[HexapodConst::LEG_NUM];		//�r���ݒu�\�Ȃ��true�ɂȂ�D���ɐڒn���Ă���Ȃ��true�ɂȂ�D
//	designlab::Vector3 ground_pos[HexapodConst::LEG_NUM];	//�r���ڒn������W�D
//
//	for (int i = 0; i < HexapodConst::LEG_NUM; i++) { ground_pos[i] = current_node.leg_pos[i]; }
//
//	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
//	{
//		if (dl_leg::isGrounded(current_node.leg_state, i))
//		{
//			//���łɐڒn���Ă���r�͐ڒn�\�Ɍ��܂��Ă���̂�true�ɂ��C���W�����̂܂ܓ����D
//			is_groundable_leg[i] = true;
//			ground_pos[i] = current_node.leg_pos[i];
//
//			//�������낷�����������̂ŁC�ڒn�r�͗V�r�s�\�D����āC�Ƃ�Ȃ�com type��S�Ă����D
//			dl_com::banLegGroundPatternFromNotFreeLeg(i, &is_able_leg_ground_pattern);
//		}
//		else
//		{
//			//���ݗV�r���̋r�͌��݂̋r��ԂŐڒn�ł��邩��������D
//			designlab::Vector3 res_ground_pos;
//
//			if (isGroundableLeg(i, current_node, res_ground_pos))
//			{
//				is_groundable_leg[i] = true;	//�ڒn�\�ɂ���D
//				ground_pos[i] = res_ground_pos;
//			}
//			else
//			{
//				is_groundable_leg[i] = false;	//�ڒn�s�\�ɂ���D
//				dl_com::banLegGroundPatternFromNotGroundableLeg(i, &is_able_leg_ground_pattern);
//			}
//		}
//	}
//
//
//	//�q�m�[�h�𐶐�����D
//	for (int i = 0; i < dl_com::getLegGroundPatternNum(); ++i)
//	{
//		//���̏d�S�^�C�v����邱�Ƃ��\�ł����
//		if (is_able_leg_ground_pattern[i])
//		{
//			SNode res_node = current_node;
//
//			res_node.changeNextNode(current_num, m_next_move);
//
//
//			//�V�r�E�ڒn������������D
//			dl_leg::LegGroundedBit new_is_ground = dl_com::getLegGroundedBitFromLegGroundPatternIndex(i);
//
//			dl_leg::changeAllLegGround(new_is_ground, &res_node.leg_state);
//
//
//			for (int j = 0; j < HexapodConst::LEG_NUM; ++j)
//			{
//				if (new_is_ground[j])
//				{
//					res_node.leg_pos[j] = ground_pos[j];
//
//					res_node.leg_base_pos[j] = ground_pos[j];
//				}
//			}
//
//			(*output_graph).push_back(res_node);
//		}
//
//	}
//
//	//�o�͂��ꂽ�m�[�h���Ȃ��Ȃ�΁C���̂܂܂̃m�[�h���o�͂���D
//	if ((*output_graph).size() == 0)
//	{
//		SNode same_node = current_node;
//
//		same_node.changeNextNode(current_num, m_next_move);
//
//		(*output_graph).emplace_back(same_node);
//	}
//}
//
//
//bool LegDownNodeCreator::isGroundableLeg(const int _leg_num, const SNode& _current_node, designlab::Vector3& _output_ground_pos)
//{
//	//for���̒���continue�ɂ��Ă� http://www9.plala.or.jp/sgwr-t/c/sec06-7.html ���Q�ƁD���Ȃ݂ɓǂ݂Â炭�Ȃ�̂Ŗ{���͎g��Ȃ��ق��������D
//
//	using designlab::Vector3;
//
//	if (mp_map == nullptr) { return false; }	//�}�b�v���Ȃ��Ƃ���false��Ԃ��D
//
//	//�r���W��devide map�łǂ��ɓ����邩���ׂāC���̃}�X��2���2���͈͓̔���S�ĒT������D
//	const designlab::Vector3 kGlobalLegBasePos = m_calculator.getGlobalLegBasePos(_current_node, _leg_num, false);
//
//	int max_x_dev = mp_map->getDevideMapNumX(kGlobalLegBasePos.x) + 1;
//	int min_x_dev = mp_map->getDevideMapNumX(kGlobalLegBasePos.x) - 1;
//	int max_y_dev = mp_map->getDevideMapNumY(kGlobalLegBasePos.y) + 1;
//	int min_y_dev = mp_map->getDevideMapNumY(kGlobalLegBasePos.y) - 1;
//
//	////�l��devide map�͈̔͊O�ɂ���Ƃ��͊ۂ߂�D
//	max_x_dev = (max_x_dev >= MapConst::LP_DIVIDE_NUM) ? MapConst::LP_DIVIDE_NUM - 1 : max_x_dev;
//	min_x_dev = (min_x_dev < 0) ? 0 : min_x_dev;
//	max_y_dev = (max_y_dev >= MapConst::LP_DIVIDE_NUM) ? MapConst::LP_DIVIDE_NUM - 1 : max_y_dev;
//	min_y_dev = (min_y_dev < 0) ? 0 : min_y_dev;
//
//	//devide map����S�T�����āC���݂̋r�ʒu(���U��������)�ɓK�����r�ݒu�\�_�����݂��邩���ׂ�D
//
//	std::vector<designlab::Vector3> _candidate_pos;		//���݂̋r�ʒu�ɍ��v��������W�Q�D
//
//	//�͈͓��̓_��S�Ē��ׂ�D
//	for (int x = min_x_dev; x < max_x_dev; x++)
//	{
//		for (int y = min_y_dev; y < max_y_dev; y++)
//		{
//			const int kPosNum = mp_map->getPointNumFromDevideMap(x, y);
//
//			for (int n = 0; n < kPosNum; n++)
//			{
//				Vector3 _pos = mp_map->getPosFromDevideMap(x, y, n);	//�r�ݒu�\�_�̍��W�����o���D
//				_pos = m_calculator.convertLocalLegPos(_current_node, _pos, _leg_num, false);
//
//				//�r�ʒu���X�V�����m�[�h���쐬����D
//				SNode _new_node = _current_node;
//
//				_new_node.leg_pos[_leg_num] = _pos;
//
//
//				//�O�̌��n�_�Ɣ�r���āC���ǂ����n�_�̎��̂ݎ��s������
//				if (!_candidate_pos.empty())
//				{
//					//���Ε������ނ��Ă���ꍇ�͌��n�_�Ƃ��č̗p���Ȃ��D
//					if (_new_node.leg_base_pos[_leg_num].projectedXY().cross(_candidate_pos.front().projectedXY()) * _new_node.leg_base_pos[_leg_num].projectedXY().cross(_pos.projectedXY()) < 0)
//					{
//						continue;
//					}
//				}
//
//				dl_leg::changeGround(_leg_num, true, &_new_node.leg_state);
//
//				if (!m_calculator.isLegInRange(_new_node, _leg_num)) { continue; }			//�r���͈͊O�Ȃ�Βǉ������ɑ��s�D
//
//				if (!isAbleLegPos(_new_node, _leg_num)) { continue; }	//�����W�Ƃ��āC�K���Ă��Ȃ��Ȃ�Βǉ������ɑ��s�D
//
//				_candidate_pos.push_back(_pos);
//			}
//		}
//	}
//
//
//	//���_��S�񋓂����̂��C���_������Ȃ����false
//	if (_candidate_pos.empty()) { return false; }
//
//	_output_ground_pos = _candidate_pos.back();
//
//	return true;
//}
//
//bool LegDownNodeCreator::isAbleLegPos(const SNode& _node, const int _leg_num)
//{
//	EDiscreteLegPos discrete_leg_pos = dl_leg::getLegState(_node.leg_state, _leg_num);		//�r�ʒu���擾(1�`7)
//
//	//�܂��ŏ��ɋr�ʒu4�̂Ƃ���ɂȂ����m���߂�D
//	if ((_node.leg_base_pos[_leg_num] - _node.leg_pos[_leg_num]).lengthSquare() < dl_math::squared(LEG_MARGIN))
//	{
//		if (discrete_leg_pos == EDiscreteLegPos::CENTER) { return true; }
//		else { return false; }
//	}
//	else
//	{
//		if (discrete_leg_pos == EDiscreteLegPos::CENTER) { return false; }
//	}
//
//	//�r�ʒu4�Ɣ�r���đO����납
//	if (_node.leg_base_pos[_leg_num].projectedXY().cross(_node.leg_pos[_leg_num].projectedXY()) * _node.leg_pos[_leg_num].projectedXY().cross({ 1,0 }) > 0)
//	{
//		//�O
//
//		if (discrete_leg_pos == EDiscreteLegPos::LOWER_FRONT || discrete_leg_pos == EDiscreteLegPos::FRONT || discrete_leg_pos == EDiscreteLegPos::UPPER_FRONT)
//		{
//			return false;
//		}
//	}
//	else
//	{
//		//���
//
//		if (discrete_leg_pos == EDiscreteLegPos::LOWER_BACK || discrete_leg_pos == EDiscreteLegPos::BACK || discrete_leg_pos == EDiscreteLegPos::UPPER_BACK)
//		{
//			return false;
//		}
//	}
//
//
//	//�r�ʒu4�Ɣ�r���ďォ����
//	if (discrete_leg_pos == EDiscreteLegPos::LOWER_BACK || discrete_leg_pos == EDiscreteLegPos::LOWER_FRONT)
//	{
//		//�r�ʒu4�Ɣ�r���ĉ�
//		if (_node.leg_base_pos[_leg_num].z - HIGH_MARGIN >= _node.leg_pos[_leg_num].z)
//		{
//			return true;
//		}
//	}
//	else if (discrete_leg_pos == EDiscreteLegPos::UPPER_BACK || discrete_leg_pos == EDiscreteLegPos::UPPER_FRONT)
//	{
//		//�r�ʒu4�Ɣ�r���ď�
//		if (_node.leg_base_pos[_leg_num].z + HIGH_MARGIN <= _node.leg_pos[_leg_num].z)
//		{
//			return true;
//		}
//	}
//	else
//	{
//		//�r�ʒu4�Ɠ������炢
//		if (std::abs(_node.leg_base_pos[_leg_num].z - _node.leg_pos[_leg_num].z) <= HIGH_MARGIN)
//		{
//			return true;
//		}
//	}
//
//	return false;
//}
