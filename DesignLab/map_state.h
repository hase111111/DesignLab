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

private:

	// friend�ɂ��邱�ƂŁCprivate�ȃ����o�ϐ��ɃA�N�Z�X�ł���悤�ɂȂ�D
	template <class Char>
	friend std::basic_ostream<Char>& operator <<(std::basic_ostream<Char>& os, const MapState& v);

	std::vector<designlab::Vector3> map_point_;	//!< ���{�b�g�������}�b�v�D�r�ݒu�\�_�̏W���ŕ\�������D
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


//! @class DevideMapState
//! @details �������y�����邽�߂ɁC�}�b�v�����݂���̈�𒷕��`�ɐ؂蕪���āC���̒��ɑ��݂���r�ݒu�\�_���W�߂����̂�devided_map_point_�D
//! @n devide_map�̗v�f�� https://atcoder.jp/contests/APG4b/tasks/APG4b_t �� �u1�����̔z��𑽎����z��Ƃ��Ďg���v�̗v�̂ŕ���ł���D
class DevideMapState
{
public:
	DevideMapState();
	DevideMapState(const MapState& map_state);	//!< �����o�֐���Init���Ăяo���D

	//! @brief Devide�}�b�v�̃f�[�^������������D
	//! @n �}�b�v�̃f�[�^���i�q��ɕ������C���̒��ɑ��݂���r�ݒu�\�_���W�߂�D
	//! @param [in] map_state �}�b�v�̃f�[�^�D
	void Init(const MapState& map_state);

	void Clear();

	constexpr bool IsInMap(const designlab::Vector3& pos) const
	{
		if (pos.x < MapConst::MAP_MIN_FORWARD || pos.x > MapConst::MAP_MAX_FORWARD) { return false; }
		if (pos.y < MapConst::MAP_MIN_HORIZONTAL || pos.y > MapConst::MAP_MAX_HORIZONTAL) { return false; }
		return true;
	}

	constexpr bool IsInMap(const float x, const float y) const
	{
		if (x < MapConst::MAP_MIN_FORWARD || x > MapConst::MAP_MAX_FORWARD) { return false; }
		if (y < MapConst::MAP_MIN_HORIZONTAL || y > MapConst::MAP_MAX_HORIZONTAL) { return false; }
		return true;
	}

	static constexpr int GetDevideMapIndexX(const float posx) 
	{
		const int res = static_cast<int>(
			(posx - MapConst::MAP_MIN_FORWARD) / 
			(((float)MapConst::MAP_MAX_FORWARD - MapConst::MAP_MIN_FORWARD) / 
				MapConst::LP_DIVIDE_NUM)
		);
		return res;
	}

	static constexpr int GetDevideMapIndexY(const float posy) 
	{
		const int res = static_cast<int>((posy - MapConst::MAP_MIN_HORIZONTAL) / (((float)MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / MapConst::LP_DIVIDE_NUM));
		return res;
	}

	//! @brief �����`��ɐ؂蕪����ꂽ�}�b�v����C�r�ݒu�\�_�̐����擾����D
	//! @n �͈͊O�̒l���w�肵���ꍇ�́C0��Ԃ��D
	//! @param [in] x_index X���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @param [in] y_index Y���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @return int �r�ݒu�\�_�̐�
	int GetPointNum(int x_index, int y_index) const;

	//! @brief �����`��ɐ؂蕪����ꂽ�}�b�v����C�r�ݒu�\�_�̎��ۂ̍��W���擾����D
	//! @n �͈͊O�̒l���w�肵���ꍇ�́C(0,0,0)��Ԃ��D
	//! @param [in] x_index x���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @param [in] y_index y���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @param [in] devide_map_index ���Ԗڂ̋r�ݒu�\�_���D
	//! @return Vector3 �r�ݒu�\�_�̍��W�D
	designlab::Vector3 GetPointPos(int x_index, int y_index, int devide_map_index) const;

	//! @brief �����`��ɐ؂蕪����ꂽ�}�b�v����C�r�ݒu�\�_vector���擾����
	//! @n �͈͊O�̒l���w�肵���ꍇ�́C���vector��Ԃ��D
	//! @param [in] x_index x���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @param [in] y_index y���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @param [out] std::vector<Vector3> point_vec �r�ݒu�\�_�̍��W�D
	void GetPointVector(int x_index, int y_index, std::vector<designlab::Vector3>* point_vec) const;

	//! @brief �����`��ɐ؂蕪����ꂽ�}�b�v����C�ł�����Z���W��Ԃ��D
	//! @param [in] x_index X���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @param [in] y_index Y���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @return float �ł�����Z���W�D
	float GetTopZ(int x_index, int y_index) const;

private:
	float kMapMinZ = -100000.0f;	//!< �}�b�v�̍Œ��Z���W

	constexpr int GetDevideMapIndex(const int x_index, const int y_index) const
	{
		return x_index * MapConst::LP_DIVIDE_NUM + y_index;
	}

	std::vector<std::vector<designlab::Vector3> > devided_map_point_;	//!< �}�b�v�����݂���̈�𐳕��`�ɐ؂蕪���āC���̒��ɑ��݂���r�ݒu�\�_���W�߂����́D

	std::vector<float> devided_map_top_z_;							//!< devided_map_point_�̒��̍ł�����z���W���܂Ƃ߂����́C�v�f�����݂��Ȃ��Ȃ�CkMapMinZ������D
};


#endif // !DESIGNLAB_MAP_STATE_H_