#include "LegUpNodeCreator.h"
#include "com_type.h"
#include "LegState.h"


void LegUpNodeCreator::create(const SNode& _current_node, const int _current_num, std::vector<SNode>* output_graph)
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

	//���ɗV�r�̋r��ڒn���邱�Ƃ͂ł��Ȃ��D
	{
		std::vector<int> _lifted_leg_num;
		LegStateEdit::getLiftedLegNumWithVector(_current_node.leg_state, _lifted_leg_num);

		for (auto& i : _lifted_leg_num)
		{
			ComType::checkAbleComTypeFromNotGroundableLeg(i, _is_able_type);
		}
	}

	//�q�m�[�h�𐶐�����D
	for (int i = 0; i < ComType::COM_TYPE_NUM; i++)
	{
		//���̏d�S�^�C�v���\�ł���΁C
		if (_is_able_type[i] == true)
		{
			SNode _res_node = _current_node;
			_res_node.changeNextNode(_current_num, m_next_move);

			//�V�r�E�ڒn������������D
			bool _temp_ground[HexapodConst::LEG_NUM] = {};
			ComType::getGroundLegFromComType(i, _temp_ground);

			for (int j = 0; j < HexapodConst::LEG_NUM; j++)
			{
				LegStateEdit::changeGround(_res_node.leg_state, j, _temp_ground[j]);

				if (_temp_ground[j] == false)
				{
					_res_node.leg_pos[j].x = 160 * HexapodConst::DEFAULT_LEG_ANGLE_COS[j];
					_res_node.leg_pos[j].y = 160 * HexapodConst::DEFAULT_LEG_ANGLE_SIN[j];
					_res_node.leg_pos[j].z = -10;
				}
			}

			//�\�Ȏp���Ȃ�΁C�q�m�[�h�Ƃ��Ēǉ�����D
			if (m_calculator.isAblePause(_res_node) == true)
			{
				(*output_graph).push_back(_res_node);
			}
		}
	}

	//�o�͂��ꂽ�m�[�h���Ȃ��Ȃ�΁C���̂܂܂̃m�[�h���o�͂���D
	if ((*output_graph).size() == 0)
	{
		SNode _res_node = _current_node;

		_res_node.changeNextNode(_current_num, m_next_move);

		(*output_graph).push_back(_res_node);
	}

}
