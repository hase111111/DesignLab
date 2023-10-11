//! @file movement_locus_renderer.h
//! @brief ���{�b�g�̈ړ��O�Ղ�`�悷��N���X�D

#ifndef DESIGNLAB_MOVEMENT_LOCUS_RENDERER_H_
#define DESIGNLAB_MOVEMENT_LOCUS_RENDERER_H_

#include <vector>

#include "designlab_vector3.h"
#include "robot_state_node.h"


//! @class MovementLocusRenderer
//! @brief ���{�b�g�̈ړ��O�Ղ�`�悷��N���X�D
class MovementLocusRenderer final
{
public:
	MovementLocusRenderer();


	//! @brief ���{�b�g�̈ړ��O�Ղ��L�^����D�m�[�h��vector����C�d�S�ʒu�̋O�Ղ��擾����
	//! @param [in] locus �I�����ꂽ�m�[�h��vector
	void SetMoveLocusPoint(const std::vector<RobotStateNode>& locus);

	//! @brief �V�~�����[�V�����̏I���_���擾����
	//! @param [in] index �V�~�����[�V�����̏I���_�̔z��
	void SetSimulationEndIndexes(const std::vector<size_t>& index);

	//! @brief ���掿���[�h�ɂ��邩�ǂ�����ݒ肷��
	//! @param [in] is_high_quality ���掿���[�h�ɂ��邩�ǂ���
	inline void SetIsHighQuality(const bool is_high_quality) { is_high_quality_ = is_high_quality; }

	//! @brief ���{�b�g�̈ړ��O�Ղ�`�悷��D
	//! @param [in] draw_simu_num �`����s���V�~�����[�V�����̔ԍ�( 0, 1, 2, ...)
	//! @param [in] draw_all_simulation ��̃p�����[�^�𖳎����āC���ׂẴV�~�����[�V�����ɂ��ĕ`�悷��
	void Draw(const size_t draw_simu_num, bool draw_all_simulation = false) const;

private:

	const unsigned int kHiddenLocusLineColor;	//!< ���ݕ\�����łȂ��O���̐��̐F

	const unsigned int kDisplayLocusLineColor;	//!< ���ݕ\�����̋O���̐��̐F

	const int kHiddenLocusLineAlpha;			//!< ���ݕ\�����łȂ��O�Ղ̓����x

	const float kLocusLineMaxLength;			//!< �O�Ղ̐��̍ő咷���D���̒l������ꍇ�͕\�����Ȃ��D

	const int kLocusLineRadius;					//!< �O�Ղ̐��̔��a


	std::vector<designlab::Vector3> move_locus_point_;	//!< ���{�b�g�̓����̑J�ڂ��L�^����vector

	std::vector<size_t> simulation_end_indexes_;	//!< �V�~�����[�V�����̏I���_�̃C���f�b�N�X�̔z��

	bool is_high_quality_;							//!< ���掿���[�h���ǂ���	
};


#endif // !DESIGNLAB_MOVEMENT_LOCUS_RENDERER_H_