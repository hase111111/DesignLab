#include "abstract_hexapod_state_calculator.h"

#include "cassert_define.h"
#include "leg_state.h"


float AbstractHexapodStateCalculator::CalculateStabilityMargin(const dl_leg::LegStateBit& leg_state, const std::array<designlab::Vector3, HexapodConst::LEG_NUM>& leg_pos) const
{
	// std::min ���J�b�R�ň͂�ł���̂́C�}�N���� min �Ɣ�邽�߁D(std::min) �Ə����Ɩ��O���Փ˂��Ȃ�

	std::array<designlab::Vector2,HexapodConst::LEG_NUM> ground_leg_pos;	// xy���ʂɓ��˂����C�d�S�����_�Ƃ������[�J��(���{�b�g)���W�n�ŁC�r�̈ʒu���v�Z����D
	int ground_leg_pos_num = 0;												// ���x�̊֌W�� vector�łȂ�array���g���D

	//�ڒn�r�̂ݒǉ�����
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (dl_leg::IsGrounded(leg_state, i))
		{
			ground_leg_pos[ground_leg_pos_num] = leg_pos[i].ProjectedXY() + GetLocalLegBasePosition(i).ProjectedXY();
			ground_leg_pos_num++;
		}
	}


	float min_margin = 0;	// ���p�`�̕ӂƏd�S�̋����̍ŏ��l
	bool is_first = true;	// ���񂩂ǂ����C�ŏ��͕K���l���X�V����

	for (int i = 0; i < ground_leg_pos_num; i++)
	{
		designlab::Vector2 i_to_i_plus_1 = ground_leg_pos[(i + 1) % ground_leg_pos_num] - ground_leg_pos[i];
		i_to_i_plus_1.Normalize();
		designlab::Vector2 i_to_com = designlab::Vector2{ 0,0 } - ground_leg_pos[i];

		float margin = i_to_com.Cross(i_to_i_plus_1);	// ���p�`�̕ӂƏd�S�̋���(�ÓI����]�T)

		if (is_first) 
		{
			min_margin = margin;
			is_first = false;
		}
		else 
		{
			min_margin = (std::min)(min_margin, margin);
		}
	}

	return min_margin;
}
