#include "movement_locus_renderer.h"

#include "DxLib.h"

#include "dxlib_util.h"

namespace dldu = designlab::dxlib_util;


MovementLocusRenderer::MovementLocusRenderer() :
	kHiddenLocusLineColor(GetColor(173, 187, 50)),
	kDisplayLocusLineColor(GetColor(239, 237, 84)),
	kHiddenLocusLineAlpha(128),
	kLocusLineMaxLength(300.0f),
	kLocusLineRadius(5),
	is_high_quality_(false)
{
}


void MovementLocusRenderer::set_simulation_end_indexes(const std::vector<size_t>& index)
{
	simulation_end_indexes_.clear();

	for (const auto& i : index)
	{
		simulation_end_indexes_.push_back(i);
	}
}


void MovementLocusRenderer::set_move_locus_point(const std::vector<SNode>& locus)
{
	move_locus_point_.clear();

	for (const auto& i : locus)
	{
		move_locus_point_.push_back(i.global_center_of_mass);
	}
}


void MovementLocusRenderer::Draw(const size_t draw_simu_num, const bool draw_all_simulation)  const
{
	const size_t kSize = move_locus_point_.size();

	if (kSize < 2)return;

	for (size_t i = 0; i < kSize - 1; i++)
	{
		//�͈͊O�A�N�Z�X��h��
		if (i < 0 && kSize - 1 <= i) { break; }


		size_t now_simu_num = simulation_end_indexes_.size();	//���݂̃V�~�����[�V������

		bool do_draw = true;	//�`�悷�邩�ǂ���


		//�n�_�̃C���f�b�N�X���V�~�����[�V�����I���C���f�b�N�X�Ɋ܂܂�Ă���Ȃ�Ε`����΂�
		for (size_t j = 0; j < simulation_end_indexes_.size(); j++)
		{
			if (i == simulation_end_indexes_[j]) { do_draw = false; }

			if (i < simulation_end_indexes_[j])
			{
				now_simu_num = j;
				break;
			}
		}

		//�n�_�ƏI�_�̍��W��`����W�ɕϊ�����
		VECTOR start = dldu::ConvertToDxlibVec(move_locus_point_.at(i));
		VECTOR end = dldu::ConvertToDxlibVec(move_locus_point_.at(i + 1));


		//�`��
		if (do_draw)
		{
			int kDivNum = 6;

			if (draw_simu_num == now_simu_num || draw_all_simulation)
			{
				if (is_high_quality_)
				{
					DrawCapsule3D(start, end, kLocusLineMaxLength, kDivNum, kDisplayLocusLineColor, kDisplayLocusLineColor, TRUE);
				}
				else
				{
					DrawLine3D(start, end, kDisplayLocusLineColor);
				}
			}
			else
			{
				SetDrawBlendMode(DX_BLENDMODE_ALPHA, kHiddenLocusLineAlpha);

				if (is_high_quality_)
				{
					DrawCapsule3D(start, end, kLocusLineMaxLength, kDivNum, kHiddenLocusLineColor, kHiddenLocusLineColor, TRUE);
				}
				else
				{
					DrawLine3D(start, end, kHiddenLocusLineColor);
				}

				SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);
			}
		}
	}
}
