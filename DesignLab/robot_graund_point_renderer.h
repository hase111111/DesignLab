#pragma once


//! @class RobotGraundPointRenderer
//! @date 2033/08/29
//! @author ���J��
//! @brief ���{�b�g�̋r�ڒn�_�̍��W��`�悷��N���X
class RobotGraundPointRenderer final
{
public:

	RobotGraundPointRenderer();


	//! ���{�b�g�̋r�ڒn�_�̕`����s���D
	//! @param [in] draw_simu_num �`����s���V�~�����[�V�����̔ԍ�( 0, 1, 2, ...)
	//! @param [in] draw_all_simulation ��̃p�����[�^�𖳎����āC���ׂẴV�~�����[�V�����ɂ��ĕ`�悷��
	void draw(const int draw_simu_num, const bool draw_all_simulation = false) const;

private:

	const unsigned int GRAUND_POINT_COLOR;			//!< �r�ڒn�_�̐F

	const unsigned int GRAUND_POINT_COLOR_BLACK;	//!< ���ׂẴV�~�����[�V�����ɂ��ĕ`�悷��ꍇ�C���݂̃V�~�����[�V�����ȊO�̐F

};



//! @file robot_graund_point_renderer.h
//! @date 2033/08/29
//! @author ���J��
//! @brief ���{�b�g�̋r�ڒn�_�̍��W��`�悷��N���X
//! @n �s�� : @lineinfo