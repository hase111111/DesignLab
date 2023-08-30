#pragma once


//! @class WorldGridRenderer
//! @date 2023/08/23
//! @author ���J��
//! @brief ���[���h�̊i�q����`�悷��N���X
class WorldGridRenderer final
{
public:

	WorldGridRenderer();

	//! @brief ���[���h�̊i�q����`�悷��
	void draw() const;


private:

	const unsigned int MAIN_GRID_X_COLOR;		//!< �i�q���̐F

	const unsigned int MAIN_GRID_Y_COLOR;		//!< �i�q���̐F

	const unsigned int SUB_GRID_X_COLOR;		//!< �ׂ��i�q���̐F

	const unsigned int SUB_GRID_Y_COLOR;		//!< �ׂ��i�q���̐F

	const float MAIN_GRID_INTERVAL;				//!< �i�q���̊Ԋu

	const int SUB_GRID_DEVIDE_NUM;				//!< ���C���̊i�q�������������ăT�u�̊i�q��������邩

	const float GRID_LINE_Z;					//!< �i�q����Z���W
};


//! @file world_grid_renderer.h
//! @date 2023/08/23
//! @author ���J��
//! @brief ���[���h�̊i�q����`�悷��N���X
//! @n �s�� : @lineinfo