#pragma once

#include <vector>

#include "designlab_vector.h"
#include "node.h"

//! @class RobotGraundPointRenderer
//! @date 2033/08/29
//! @author ���J��
//! @brief ���{�b�g�̋r�ڒn�_�̍��W��`�悷��N���X
class RobotGraundPointRenderer final
{
public:

	RobotGraundPointRenderer();


	//! ���{�b�g�̋r�ڒn�_�̍��W��ݒ肷��D
	//! @param [in] node ���{�b�g�̋r�ڒn�_�̍��W
	void setNode(const std::vector<SNode>& node, const std::vector<size_t>& simu_end_node_index);


	//! ���{�b�g�̋r�ڒn�_�̕`����s���D
	//! @param [in] draw_simu_num �`����s���V�~�����[�V�����̔ԍ�( 0, 1, 2, ...)
	//! @param [in] draw_all_simulation ��̃p�����[�^�𖳎����āC���ׂẴV�~�����[�V�����ɂ��ĕ`�悷��
	void draw(const size_t draw_simu_num, const bool draw_all_simulation = false) const;

private:

	const unsigned int GRAUND_POINT_COLOR_RIGHT;			//!< �r�ڒn�_�̐F

	const unsigned int GRAUND_POINT_COLOR_LEFT;			//!< �r�ڒn�_�̐F

	const unsigned int GRAUND_POINT_COLOR_BLACK_RIGHT;		//!< ���ׂẴV�~�����[�V�����ɂ��ĕ`�悷��ꍇ�C���݂̃V�~�����[�V�����ȊO�̐F

	const unsigned int GRAUND_POINT_COLOR_BLACK_LEFT;		//!< ���ׂẴV�~�����[�V�����ɂ��ĕ`�悷��ꍇ�C���݂̃V�~�����[�V�����ȊO�̐F


	std::vector<size_t> m_simu_end_index;			//!< �V�~�����[�V�����̏I���C���f�b�N�X

	size_t m_loaded_node_num;						//!< �ǂݍ��񂾃m�[�h�̐�

	std::vector<std::vector<std::pair<dl_vec::SVector, int>>> m_graund_point;	//!< ���{�b�g�̋r�ڒn�_�̍��W
};



//! @file robot_graund_point_renderer.h
//! @date 2033/08/29
//! @author ���J��
//! @brief ���{�b�g�̋r�ڒn�_�̍��W��`�悷��N���X
//! @n �s�� : @lineinfo