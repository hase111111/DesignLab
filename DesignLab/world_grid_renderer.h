//! @file world_grid_renderer.h
//! @brief ワールドの格子線を描画するクラス

#ifndef DESIGNLAB_WORLD_GRID_RENDERER_H_
#define DESIGNLAB_WORLD_GRID_RENDERER_H_


//! @class WorldGridRenderer
//! @brief ワールドの格子線を描画するクラス
class WorldGridRenderer final
{
public:

	WorldGridRenderer();

	//! @brief ワールドの格子線を描画する
	void Draw() const;


private:

	const unsigned int kMainGridXColor;		//!< 格子線の色

	const unsigned int kMainGridYColor;		//!< 格子線の色

	const unsigned int kSubGridXColor;		//!< 細い格子線の色

	const unsigned int kSubGridYColor;		//!< 細い格子線の色

	const int kMainGridNum;					//!< メインの格子線の数

	const float kMainGridInterval;			//!< 格子線の間隔

	const int kSubGridDevideNum;			//!< メインの格子線を何分割してサブの格子線をいれるか

	const float kGridLineZPos;				//!< 格子線のZ座標
};


#endif // DESIGNLAB_WORLD_GRID_RENDERER_H_