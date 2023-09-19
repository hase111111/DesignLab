#pragma once

#include <map>
#include <vector>

#include "designlab_vector.h"
#include "map_const.h"


//! @class MapState
//! @brief �}�b�v��\���N���X�D
//! @details ���̌����̎�@�ł́C���{�b�g�������}�b�v�͋r�ݒu�\�_�̏W���ŕ\�������D�ʂł͂Ȃ��_�̏W���D
//! @n �r�ݒu�\�n�_�̃f�[�^�͈ʒu�x�N�g���̔z��Ŏ������Ă���D
//! @n �����o�ϐ��̃f�[�^�ւ̃A�N�Z�X�́C�����o�֐���get????�n�̊֐��ōs���D
//! @n ���ڃf�[�^�̂������s��Ȃ��̂́C���f�[�^�ł���ƒl��ύX�\�ɂȂ��Ă��܂�����ł���D
//! @n const�Ȋ֐����g���΁C�Ԉ���Ă��l�̕ύX���ł��Ȃ��̂ŁC�f�[�^�̂����ɗD���D
class MapState
{
public:
	MapState() : map_point_({}) {};
	MapState(const MapState& other) : map_point_({}) { map_point_ = other.map_point_; };
	MapState& operator = (const MapState& other);

	//getter setter

	//! @brief �r�ݒu�\�_�̍��W��Ԃ��D
	//! @param [in] num ���Ԗڂ̋r�ݒu�\�_���D
	//! @return SVector �r�ݒu�\�_�̍��W�D
	inline dl_vec::SVector map_point(const size_t num) const
	{
		return map_point_[num];
	}

	//! @brief �r�ݒu�\�_�̍��W��ݒ肷��
	//! @n �ꉞ��������ǁC�g�����Ƃ͂Ȃ��Ǝv���DAddMapPoint���g�����ƁD
	//! @param [in] num ���Ԗڂ̋r�ݒu�\�_���D
	//! @param [in] point �r�ݒu�\�_�̍��W�D
	inline void set_map_point(const size_t num, const dl_vec::SVector& point)
	{
		map_point_[num] = point;
	}

	//! @brief �r�ݒu�\�_�̍��W��ݒ肷��
	//! @param [in] point �r�ݒu�\�_�̍��W�D
	inline void set_map_point(const std::vector<dl_vec::SVector>& point)
	{
		map_point_ = point;
	}


	//! @brief �r�ݒu�\�_�̑�����Ԃ��D
	//! @return size_t �r�ݒu�\�_�̑���
	inline size_t GetMapPointSize() const
	{
		return map_point_.size();
	}

	//! @brief �r�ݒu�\�_�̍��W��ǉ�����D
	//! @param [in] point �r�ݒu�\�_�̍��W�D
	inline void AddMapPoint(const dl_vec::SVector& point)
	{
		map_point_.push_back(point);
	}

	//! @brief �r�ݒu�\�_�̍��W����������D
	inline void ClearMapPoint()
	{
		map_point_.clear();
	}


private:

	std::vector<dl_vec::SVector> map_point_;	//!< ���{�b�g�������}�b�v�D�r�ݒu�\�_�̏W���ŕ\�������D
};


//! @class DevideMapState
//! @details �������y�����邽�߂ɁC�}�b�v�����݂���̈�𒷕��`�ɐ؂蕪���āC���̒��ɑ��݂���r�ݒu�\�_���W�߂����̂�devided_map_point_
//! @n devide_map�̗v�f�� https://atcoder.jp/contests/APG4b/tasks/APG4b_t �� �u1�����̔z��𑽎����z��Ƃ��Ďg���v�̗v�̂ŕ���ł���D
class DevideMapState
{
public:
	DevideMapState();
	DevideMapState(const MapState& map_state);

	void Init(const MapState& map_state);

	void Clear();

	constexpr bool IsInMap(const dl_vec::SVector& pos) const
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
		const int res = static_cast<int>((posx - MapConst::MAP_MIN_FORWARD) / (((float)MapConst::MAP_MAX_FORWARD - MapConst::MAP_MIN_FORWARD) / MapConst::LP_DIVIDE_NUM));
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
	//! @return SVector �r�ݒu�\�_�̍��W�D
	dl_vec::SVector GetPointPos(int x_index, int y_index, int devide_map_index) const;

	//! @brief �����`��ɐ؂蕪����ꂽ�}�b�v����C�r�ݒu�\�_vector���擾����
	//! @n �͈͊O�̒l���w�肵���ꍇ�́C���vector��Ԃ��D
	//! @param [in] x_index x���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @param [in] y_index y���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @param [out] std::vector<SVector> point_vec �r�ݒu�\�_�̍��W�D
	void GetPointVector(int x_index, int y_index, std::vector<dl_vec::SVector>* point_vec) const;

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

	std::vector<std::vector<dl_vec::SVector> > devided_map_point_;	//!< �}�b�v�����݂���̈�𐳕��`�ɐ؂蕪���āC���̒��ɑ��݂���r�ݒu�\�_���W�߂����́D

	std::vector<float> devided_map_top_z_;							//!< devided_map_point_�̒��̍ł�����z���W���܂Ƃ߂����́C�v�f�����݂��Ȃ��Ȃ�CkMapMinZ������D
};




//class MapState_Old final
//{
//public:
//
//	MapState_Old() = default;
//
//	MapState_Old& operator = (const MapState_Old& other);
//
//	//! @breif �n�`�f�[�^�̏��������s���D
//	//! @param [in] mode �񋓑� EMapCreateMode�łǂ̂悤�Ȓn�`�𐶐����邩�w�肷��D
//	//! @param [in] option �����o�ϐ��� OPTION_????�Ő�������}�b�v�̃I�v�V�������w�肷��Dmode��ReadFromFile���w�肳��Ă���Ȃ疳�������D
//	//! @n �܂��Cbit���Z�𗘗p���ĕ����w��ł���D�Ⴆ�Ό��������C�K�i��ɂ������Ȃ�΁COPTION_PERFORATED | OPTION_STEP �Ǝw�肷��D
//	//! @param [in] do_output ���������}�b�v���t�@�C���o�͂���Ȃ�true�C���Ȃ��Ȃ��false�D����ꍇ�͐��������u�ԂɃt�@�C���o�͂����D
//	void init(const EMapCreateMode mode, const int option, const bool do_output);
//
//
//	//! @brief �O���[�o����x���W�̒l����Cm_devide_map�̂ǂ��������Ă��邩�v�Z���ĕԂ��D
//	//! @n �͈͊O�̒l���w�肵���ꍇ�́C0�܂��̓}�b�v�̒[�̍��W��Ԃ��D
//	//! @param [in] posx �O���[�o����y���W�C�O���[�o���̓}�b�v�̌��_��0�Ƃ�����W�n�D���W�n�̌�����Svector�\���̂��Q�ƁD
//	//! @return int m_devide_map��x���W�̂ǂ��������Ă��邩
//	inline static int getDevideMapNumX(const float posx)
//	{
//		const int res = static_cast<int>((posx - MapConst::MAP_MIN_FORWARD) / (((float)MapConst::MAP_MAX_FORWARD - MapConst::MAP_MIN_FORWARD) / MapConst::LP_DIVIDE_NUM));
//		if (res < 0)return 0;
//		if (res >= MapConst::LP_DIVIDE_NUM)return MapConst::LP_DIVIDE_NUM - 1;
//		return res;
//	}
//
//
//	//! @brief �O���[�o����y���W�̒l����Cm_devide_map�̂ǂ��������Ă��邩�v�Z���ĕԂ��D
//	//! @n �͈͊O�̒l���w�肵���ꍇ�́C0�܂��̓}�b�v�̒[�̍��W��Ԃ��D
//	//! @param [in] posy �O���[�o����y���W�C�O���[�o���̓}�b�v�̌��_��0�Ƃ�����W�n�D���W�n�̌����͍\����SVector���Q�ƁD
//	//! @return int m_devide_map��y���W�̂ǂ��������Ă��邩
//	inline static int getDevideMapNumY(const float posy)
//	{
//		const int res = static_cast<int>((posy - MapConst::MAP_MIN_HORIZONTAL) / (((float)MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / MapConst::LP_DIVIDE_NUM));
//		if (res < 0) { return 0; }
//		if (res >= MapConst::LP_DIVIDE_NUM) { return MapConst::LP_DIVIDE_NUM - 1; }
//		return res;
//	}
//
//	//! @brief �����`��ɐ؂蕪����ꂽ�}�b�v����C�r�ݒu�\�_�̐����擾����D
//	//! @n �͈͊O�̒l���w�肵���ꍇ�́C0��Ԃ��D
//	//! @param [in] x X���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
//	//! @param [in] y Y���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
//	//! @return int �r�ݒu�\�_�̐�
//	int getPointNumFromDevideMap(const int x, const int y) const;
//
//
//	//! @brief �����`��ɐ؂蕪����ꂽ�}�b�v����C�r�ݒu�\�_�̎��ۂ̍��W���擾����D
//	//! @n �͈͊O�̒l���w�肵���ꍇ�́C(0,0,0)��Ԃ��D
//	//! @param [in] x x���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
//	//! @param [in] y y���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
//	//! @param [in] num ���Ԗڂ̋r�ݒu�\�_���D 
//	//! @return SVector �r�ݒu�\�_�̍��W�D
//	dl_vec::SVector getPosFromDevideMap(const int x, const int y, const int num) const;
//
//
//	//! @brief �����`��ɐ؂蕪����ꂽ�}�b�v����C�ł�����Z���W��Ԃ��D
//	//! @param [in] x X���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
//	//! @param [in] y Y���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
//	//! @return float �ł�����Z���W�D
//	inline float getTopZFromDevideMap(const int x, const int y) const
//	{
//		return m_devide_map_top_z[getDevideMapNum(x, y)];
//	};
//
//
//	//! @brief �r�ݒu�\�_�̍��W���o�͂���D�����`��ɐ؂蕪����ꂽ�}�b�v����l���擾����킯�ł͂Ȃ��̂ŁC�`���f�o�b�O�݂̂ɗ��p���邱�Ƃ𐄏��D
//	//! @param [in] num 
//	//! @return SVector �r�ݒu�\�_�̍��W�D
//	dl_vec::SVector getPos(const int num) const;
//
//
//	//! @brief �r�ݒu�\�_�̍��W�̐����o�͂���DgetPos�֐��ƕ��p���Ďg�p����D
//	//! @return int �r�ݒu�\�_�̑���
//	inline int getPosNum() const
//	{
//		return static_cast<int>(m_map_data.size());
//	}
//
//
//private:
//
//	const float kMapMinZ = -100000.0f;		//�}�b�v�̍Œ��Z���W
//
//
//	// m_map_data �����Ƃ� m_devide_map�ɒl���Z�b�g����
//	void makeDevideMap();
//
//	// m_devide_map�ł͗v�f��1�����̔z��Ƃ��ĕ���ł���̂ŁC2�̓��͂���C�ǂ̒l���Q�Ƃ���΂悢�̂����v�Z����D
//	inline int getDevideMapNum(const int x, const int y) const { return x * MapConst::LP_DIVIDE_NUM + y; }
//
//
//	std::vector<dl_vec::SVector> m_map_data;					//���{�b�g�������}�b�v�D�r�ݒu�\�_�̏W���ŕ\�������D
//
//	std::vector<std::vector<dl_vec::SVector> > m_devide_map;	//�}�b�v�����݂���̈�𐳕��`�ɐ؂蕪���āC���̒��ɑ��݂���r�ݒu�\�_���W�߂����́D
//
//	std::vector<float> m_devide_map_top_z;						//m_devide_map�̒��̍ł�����z���W���܂Ƃ߂����́C�v�f�����݂��Ȃ��Ȃ�Cfloat�^�̍ŏ��l������D
//
//};


//! @file map_state.h
//! @date 2023/08/06
//! @author ���J��
//! @brief �}�b�v�̃f�[�^���Ǘ�����N���X�D
//! @n �s�� : @lineinfo
