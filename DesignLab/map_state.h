//! @file map_state.h
//! @brief �}�b�v�̃f�[�^�����N���X�D


#ifndef DESIGNLAB_MAP_STATE_H_
#define DESIGNLAB_MAP_STATE_H_


#include <map>
#include <vector>

#include "cassert_define.h"
#include "designlab_vector3.h"
#include "map_const.h"


//! @class MapState
//! @brief �}�b�v��\���N���X�D
//! @details ���̌����̎�@�ł́C���{�b�g�������}�b�v�͋r�ݒu�\�_�̏W���ŕ\�������D�ʂł͂Ȃ��_�̏W���D
//! @n �r�ݒu�\�n�_�̃f�[�^�͈ʒu�x�N�g���̔z��Ŏ������Ă���D
//! @n �����I��std::vector<designlab::Vector3>�̃��b�p�[�N���X�Ƃ�����D
//! @n �����o�ϐ��̃f�[�^�ւ̃A�N�Z�X�́C�����o�֐���Get????�n�̊֐��ōs���D
//! @n ���ڃf�[�^�̂������s��Ȃ��̂́C���f�[�^�ł���ƒl��ύX�\�ɂȂ��Ă��܂�����ł���D
//! @n const�Ȋ֐����g���΁C�Ԉ���Ă��l�̕ύX���ł��Ȃ��̂ŁC�f�[�^�̂����ɗD���D
class MapState final
{
public:
	MapState() : map_point_({}) {};
	MapState(const std::vector<designlab::Vector3>& map_point) : map_point_(map_point) {};
	MapState(const MapState& other) = default;					//!< �R�s�[�R���X�g���N�^
	MapState(MapState&& other) noexcept = default;				//!< ���[�u�R���X�g���N�^
	MapState& operator = (const MapState& other) = default;		//!< ������Z�q

	//! @brief �r�ݒu�\�_�̍��W��Ԃ��D
	//! @param [in] index ���Ԗڂ̋r�ݒu�\�_�̍��W��Ԃ����D
	//! @n �͈͊O�ɃA�N�Z�X�����ꍇ�Cassert�Ŏ~�܂�D
	//! @return Vector3 �r�ݒu�\�_�̍��W�D
	inline designlab::Vector3 GetMapPoint(const size_t index) const noexcept
	{
		assert(index < map_point_.size());

		return map_point_[index];
	}

	//! @brief �r�ݒu�\�_�̑�����Ԃ��D
	//! @return size_t �r�ݒu�\�_�̑���
	inline size_t GetMapPointSize() const noexcept
	{
		return map_point_.size();
	}

	//! @brief �r�ݒu�\�_�̍��W��1�I�я㏑������D
	//! @n �ꉞ��������ǁC�g�����Ƃ͂Ȃ��Ǝv���D
	//! @n ���e���������������Ȃ��Clear������CAddMapPoint���g�����ƁD
	//! @param [in] index �ύX����r�ݒu�\�_�̔ԍ��D
	//! @n �͈͊O�ɃA�N�Z�X�����ꍇ�Cassert�Ŏ~�܂�D
	//! @param [in] point �r�ݒu�\�_�̍��W�D
	inline void SetMapPoint(const size_t index, const designlab::Vector3& point) noexcept
	{
		assert(index < map_point_.size());
		map_point_[index] = point;
	}

	//! @brief �r�ݒu�\�_�̍��W��ݒ肷��
	//! @param [in] point �r�ݒu�\�_�̍��W�D
	inline void SetMapPointVec(const std::vector<designlab::Vector3>& point) noexcept
	{
		map_point_ = point;
	}

	//! @brief �r�ݒu�\�_�̍��W��ǉ�����D
	//! @param [in] point �r�ݒu�\�_�̍��W�D
	inline void AddMapPoint(const designlab::Vector3& point) noexcept
	{
		map_point_.push_back(point);
	}

	//! @brief �r�ݒu�\�_�̍��W����������D
	inline void ClearMapPoint() noexcept
	{
		map_point_.clear();
	}


	static constexpr float kMapPointDistance = 20.0f;	//!< z������(�ォ��)�݂��Ƃ��C�i�q�_��ɕ�����ꂽ�r�ڒn�\�_�̊Ԋu [mm]�D

private:

	// friend�ɂ��邱�ƂŁCprivate�ȃ����o�ϐ��ɃA�N�Z�X�ł���悤�ɂȂ�D
	template <class Char>
	friend std::basic_ostream<Char>& operator <<(std::basic_ostream<Char>& os, const MapState& v);

	std::vector<designlab::Vector3> map_point_;	//!< ���{�b�g�������}�b�v�D�r�ݒu�\�_�̏W���ŕ\�������D


	static_assert(kMapPointDistance > 0.0f, "kMapPointDistance�͐��̎����ł���K�v������܂��D");
};


template <class Char>
std::basic_ostream<Char>& operator <<(std::basic_ostream<Char>& os, const MapState& map)
{
	for (const auto &i : map.map_point_)
	{
		os << i << "\n";
	}

	return os;
}


#endif // !DESIGNLAB_MAP_STATE_H_