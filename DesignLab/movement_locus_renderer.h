#pragma once

#include <vector>

#include "designlab_vector.h"
#include "node.h"


//! @class MovementLocusRenderer
//! @date 2023/08/23
//! @author ���J��
//! @brief ���{�b�g�̈ړ��O�Ղ�`�悷��N���X�D
class MovementLocusRenderer
{
public:
	MovementLocusRenderer();


	//! @brief �V�~�����[�V�����̏I���_���擾����
	//! @param [in] index �V�~�����[�V�����̏I���_�̔z��
	void setSimuEndIndex(const std::vector<size_t>& index);


	//! @brief ���{�b�g�̈ړ��O�Ղ��L�^����D
	//! @param [in] locus ���{�b�g�̈ړ��O�Ղ̔z��D
	void setMovementLocus(const std::vector<SNode>& locus);


	//! @brief ���{�b�g�̈ړ��O�Ղ�`�悷��D
	void draw() const;

private:

	const unsigned int LOCUS_BASE_COLOR;			//!< ���ݕ\�����łȂ��C�O��

	const unsigned int LOCUS_DISPLAY_LINE_COLOR;	//!< ���ݕ\�����̋O���̐��̐F

	const int LOCUS_ALPHA;							//!< �O�Ղ̓����x

	const float LOCUS_LINE_MAX_WIDTH = 200.0f;		//!< �O�Ղ̐��̍ő咷��


	std::vector<dl_vec::SVector> m_movement_locus;	//!< ���{�b�g�̓����̑J�ڂ��L�^����vector

	std::vector<size_t> m_simu_end_index;			//!< �V�~�����[�V�����̏I���_�̔z��

	int m_display_simu_num;							//!< �V�~�����[�V������

};


//! @file movement_locus_renderer.h
//! @date 2023/08/23
//! @author ���J��
//! @brief ���{�b�g�̈ړ��O�Ղ�`�悷��N���X�D