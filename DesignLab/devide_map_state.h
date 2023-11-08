//! @file devide_map_state.h
//! @brief �}�b�v�𕪊����ĊǗ�����N���X�D

#ifndef DESIGNLAB_DEVIDE_MAP_STATE_H_
#define DESIGNLAB_DEVIDE_MAP_STATE_H_


#include "map_state.h"
#include "designlab_vector3.h"

//! @class DevideMapState
//! @brief �������y�����邽�߂ɁC�}�b�v�����݂���̈�𒷕��`�ɐ؂蕪���āC���̒��ɑ��݂���r�ݒu�\�_���W�߂����̂�devided_map_point_�D
//! @n devide_map�̗v�f�� https://atcoder.jp/contests/APG4b/tasks/APG4b_t �� �u1�����̔z��𑽎����z��Ƃ��Ďg���v�̗v�̂ŕ���ł���D
//! @n ���W�̓O���[�o�����W�ł���D
class DevideMapState
{
public:
	DevideMapState();

	//! @brief Devide�}�b�v�̃f�[�^������������D
	//! @n �}�b�v�̃f�[�^���i�q��ɕ������C���̒��ɑ��݂���r�ݒu�\�_���W�߂�D
	//! @param [in] map_state �}�b�v�̃f�[�^�D
	//! @param [in] global_robot_com ���{�b�g�̏d�S�̃O���[�o�����W�D
	void Init(const MapState& map_state, const dl::Vector3 global_robot_com);

	//! @brief Devide�}�b�v�̃f�[�^������������D
	void Clear();

	//! @brief �w�肵�����W��Devide�}�b�v�͈͓̔��ɑ��݂��邩�ǂ�����Ԃ��D
	//! @param [in] x �O���[�o�����W�D
	//! @param [in] y �O���[�o�����W�D
	//! @return bool �͈͓��ɑ��݂���Ȃ�true�D
	constexpr bool IsInMap(const float x, const float y) const
	{
		if (x < global_robot_com_.x + kDevideMapMinX || global_robot_com_.x + kDevideMapMaxX < x) { return false; }
		if (y < global_robot_com_.y + kDevideMapMinY || global_robot_com_.y + kDevideMapMaxY < y) { return false; }

		return true;
	}

	//! @brief �w�肵�����W��Devide�}�b�v�͈͓̔��ɑ��݂��邩�ǂ�����Ԃ��D
	//! @param [in] pos �O���[�o�����W�D
	//! @return bool �͈͓��ɑ��݂���Ȃ�true�D
	constexpr bool IsInMap(const designlab::Vector3& pos) const noexcept
	{
		return IsInMap(pos.x, pos.y);
	}

	//! @brief �w�肵�����W��Devide�}�b�v��index�ɂ����Ăǂ̈ʒu�ɂ��邩��Ԃ��D
	//! @param [in] posx �O���[�o�����W��x���W�D
	//! @return int ���Ԗڂ̋r�ݒu�\�_���D
	constexpr int GetDevideMapIndexX(const float posx) const noexcept
	{
		return static_cast<int>((posx - global_robot_com_.x - kDevideMapMinX) * static_cast<float>(kDevideNum) / (kDevideMapMaxX - kDevideMapMinX));
	}

	//! @brief �w�肵�����W��Devide�}�b�v��index�ɂ����Ăǂ̈ʒu�ɂ��邩��Ԃ��D
	//! @param [in] posy �O���[�o�����W��y���W�D
	//! @return int ���Ԗڂ̋r�ݒu�\�_���D
	constexpr int GetDevideMapIndexY(const float posy) const noexcept
	{
		return static_cast<int>((posy - global_robot_com_.y - kDevideMapMinY) * static_cast<float>(kDevideNum) / (kDevideMapMaxY - kDevideMapMinY));
	}

	//! @brief �w�肵�����W��Devide�}�b�v��index�͈͓̔��ɂȂ�悤�Ɋۂ߂�D
	static constexpr int ClampDevideMapIndex(const int index) noexcept
	{
		if (index < 0) { return 0; }
		if (kDevideNum <= index) { return kDevideNum - 1; }

		return index;
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

	//! @brief �����`��ɐ؂蕪����ꂽ�}�b�v����C�r�ݒu�\�_��vector���擾����
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

	static constexpr int kDevideMapPointNum = 4;	//!< 1�̃}�X�ɑ��݂���r�ݒu�\�_�̐���kDevideMapPointNum �~ kDevideMapPointNum �D
	static constexpr float kDevideAreaLength = MapState::kMapPointDistance * kDevideMapPointNum;	//!< 1�̃}�X�̈�ӂ̒����D

	static constexpr int kDevideNum = 15;
	static constexpr float kDevideMapMaxX = kDevideAreaLength * kDevideNum / 2.0f;	//!< �}�b�v�̍ő��X���W
	static constexpr float kDevideMapMinX = -kDevideMapMaxX;						//!< �}�b�v�̍ŏ���X���W
	static constexpr float kDevideMapMaxY = kDevideAreaLength * kDevideNum / 2.0f;	//!< �}�b�v�̍ő��Y���W
	static constexpr float kDevideMapMinY = -kDevideMapMaxY;						//!< �}�b�v�̍ŏ���Y���W

	static constexpr float kMapMinZ = -100000.0f;	//!< �}�b�v�̍Œ��Z���W

	//! @brief Devide Map�ł́C2�����̔z���1�����̔z��Ƃ��Ĉ����Ă���D
	//! @n ���̂��߁C2�����̔z��̃C���f�b�N�X��1�����̔z��̃C���f�b�N�X�ɕϊ�����D
	//! @param [in] x_index x���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @param [in] y_index y���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @return int 1�����̔z��̃C���f�b�N�X�D
	constexpr int GetDevideMapIndex(const int x_index, const int y_index) const noexcept
	{
		return x_index * kDevideNum + y_index;
	}

	//! @brief �^����ꂽ�C���f�b�N�X���L���Ȓl���ǂ�����Ԃ��D
	//! @param [in] index �}�b�v�̃C���f�b�N�X�D
	//! @return bool �L���Ȓl�Ȃ�true�D
	constexpr bool IsVaildIndex(const int index) const noexcept
	{
		if (index < 0 || kDevideNum <= index) { return false; }

		return true;
	}

	designlab::Vector3 global_robot_com_;	//!< ���{�b�g�̏d�S�̃O���[�o�����W�D

	//!< �}�b�v�����݂���̈���i�q��ɐ؂蕪���āC���̒��ɑ��݂���r�ݒu�\�_���W�߂����́D
	std::vector<std::vector<designlab::Vector3> > devided_map_point_;

	//!< devided_map_point_�̒��̍ł�����z���W���܂Ƃ߂����́C�v�f�����݂��Ȃ��Ȃ�CkMapMinZ������D
	std::vector<float> devided_map_top_z_;


	static_assert(kDevideMapPointNum > 0, "kDevideMapPointNum�͐��̐����ł���K�v������܂��D");
	static_assert(kDevideAreaLength > 0.0f, "kDevideAreaLength�͐��̎����ł���K�v������܂��D");
	static_assert(kDevideNum > 0, "kDevideNum�͐��̐����ł���K�v������܂��D");
	static_assert(kDevideMapMaxX > kDevideMapMinX, "kDevideMapMaxX��Min���傫���K�v������܂��D");
	static_assert(kDevideMapMaxY > kDevideMapMinY, "kDevideMapMaxY��Min���傫���K�v������܂��D");
};
#endif