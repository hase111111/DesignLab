//! @file map_renderer.h
//! @brief �}�b�v�̕`����s���N���X�D


#ifndef DESIGNLAB_MAP_RENDERER_H_
#define DESIGNLAB_MAP_RENDERER_H_

#include "devide_map_state.h"
#include "designlab_vector3.h"
#include "map_state.h"


//! @class MapRenderer
//! @brief Map�̕`����s���N���X�D
class MapRenderer final
{
public:
	MapRenderer();
	~MapRenderer() = default;

	//! @brief ���{�b�g�̏d�S�̃O���[�o�����W��ݒ肷��D
	//! @n DevideMap�̓��{�b�g�̏d�S�𒆐S�ɂ��Ă���̂ŁC���{�b�g�̏d�S�̃O���[�o�����W��ݒ肷��K�v������D
	//! @n ����Ɠ�����DevideMap���X�V�����D
	//! @param [in] pos ���{�b�g�̏d�S�̃O���[�o�����W�D
	void SetHexapodPosition(const designlab::Vector3& pos);

	//! @brief �}�b�v�̏�Ԃ�ݒ肷��D����Ɠ�����DevideMap���X�V�����D
	//! @param [in] map �}�b�v�̏�ԁD
	void SetMapState(const MapState& map);

	//! @brief �}�b�v�̕`����s���D
	//! @param [in] map �}�b�v�̏�ԁD
	void Draw() const;

private:

	const unsigned int kColorGray;
	const unsigned int kColorLightGray;
	const unsigned int kColorDarkGray;
	const float kCubeSize;

	MapState map_;

	DevideMapState devide_map_;

	designlab::Vector3 hexapod_pos_;
};


#endif //DESIGNLAB_MAP_RENDERER_H_