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


	//! @brief ���{�b�g�̈ړ��O�Ղ�ǉ�����D
	//! @param [in] locus �ǉ����郍�{�b�g�̈ړ��O�ՁD
	void addMovementLocusPoint(const dl_vec::SVector& locus);

	//! @brief ���{�b�g�̈ړ��O�Ղ��L�^����D
	//! @param [in] locus ���{�b�g�̈ړ��O�Ղ̔z��D
	void setMovementLocus(const std::vector<SNode>& locus);


	//! @brief ���{�b�g�̈ړ��O�Ղ�`�悷��D
	void draw() const;

private:

	const unsigned int LOCUS_COLOR;					//!< �ԐF

	const float LOCUS_LINE_MAX_WIDTH = 200.0f;		//!< �O�Ղ̐��̍ő咷��


	std::vector<dl_vec::SVector> m_movement_locus;	//!< ���{�b�g�̓����̑J�ڂ��L�^����vector

};


//! @file movement_locus_renderer.h
//! @date 2023/08/23
//! @author ���J��
//! @brief ���{�b�g�̈ړ��O�Ղ�`�悷��N���X�D