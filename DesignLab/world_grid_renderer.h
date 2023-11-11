//! @file world_grid_renderer.h
//! @brief ���[���h�̊i�q����`�悷��N���X

#ifndef DESIGNLAB_WORLD_GRID_RENDERER_H_
#define DESIGNLAB_WORLD_GRID_RENDERER_H_


//! @class WorldGridRenderer
//! @brief ���[���h�̊i�q����`�悷��N���X
class WorldGridRenderer final
{
public:

	WorldGridRenderer();

	//! @brief ���[���h�̊i�q����`�悷��
	void Draw() const;


private:

	const unsigned int kMainGridXColor;		//!< �i�q���̐F

	const unsigned int kMainGridYColor;		//!< �i�q���̐F

	const unsigned int kSubGridXColor;		//!< �ׂ��i�q���̐F

	const unsigned int kSubGridYColor;		//!< �ׂ��i�q���̐F

	const int kMainGridNum;					//!< ���C���̊i�q���̐�

	const float kMainGridInterval;			//!< �i�q���̊Ԋu

	const int kSubGridDevideNum;			//!< ���C���̊i�q�������������ăT�u�̊i�q��������邩

	const float kGridLineZPos;				//!< �i�q����Z���W
};


#endif // DESIGNLAB_WORLD_GRID_RENDERER_H_