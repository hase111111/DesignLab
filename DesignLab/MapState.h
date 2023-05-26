#pragma once
#include "vectorFunc.h"
#include <vector>
#include <map>
#include "MapCreator.h"

class MapState final
{
private:

	// m_map_data �����Ƃ� m_devide_map�ɒl���Z�b�g����
	void makeDevideMap();

	//���{�b�g�����邭�}�b�v�D�r�ݒu�\�_�̏W���ŕ\�������D
	std::vector<myvector::SVector> m_map_data;

	//�}�b�v�����݂���̈�𐳕��`�ɐ؂蕪���āC���̒��ɑ��݂���r�ݒu�\�_���W�߂����́D
	//�v�f�� https://atcoder.jp/contests/APG4b/tasks/APG4b_t �� �u1�����̔z��𑽎����z��Ƃ��Ďg���v�̗v�̂ŕ���ł���D
	std::vector<std::vector<myvector::SVector> > m_devide_map;

	// m_devide_map�ł͗v�f��1�����̔z��Ƃ��ĕ���ł���̂ŁC2�̓��͂���C�ǂ̒l���Q�Ƃ���΂悢�̂����v�Z����D
	inline int getDevideMapNum(const int _x, const int _y) const { return _x * MapConst::LP_DIVIDE_NUM + _y; }

public:
	MapState() = default;

	//���������s���D
	void init(const EMapCreateMode _mode, const int _option, const bool _do_output);

	//�����`��ɐ؂蕪����ꂽ�}�b�v����C�r�ݒu�\�_�̐����擾����D
	int getPointNumFromDevideMap(const int _x, const int _y) const;

	//�����`��ɐ؂蕪����ꂽ�}�b�v����C�r�ݒu�\�_�̍��W���擾����
	myvector::SVector getPosFromDevideMap(const int _x, const int _y, const int _num) const;

	//�r�ݒu�\�_�̍��W���o�͂���D�`��ɂ̂ݗ��p�D
	myvector::SVector getPos(const int _num) const;

	//�r�ݒu�\�_�̍��W�̐����o�͂���DgetPos�֐��ƕ��p���Ďg�p����D
	inline size_t getPosNum() const { return m_map_data.size();; }

	// x���W�̒l����Cm_devide_map�̂ǂ��������Ă��邩�v�Z���ĕԂ�
	inline static int getDevideMapNumX(const float _posx)
	{
		int tmp = (int)((_posx - MapConst::MAP_X_MIN) / (((float)MapConst::MAP_X_MAX - MapConst::MAP_X_MIN) / MapConst::LP_DIVIDE_NUM));
		if (tmp < 0)return 0;
		if (tmp >= MapConst::LP_DIVIDE_NUM * MapConst::LP_DIVIDE_NUM)return MapConst::LP_DIVIDE_NUM * MapConst::LP_DIVIDE_NUM - 1;
		return tmp;
	}

	// y���W�̒l����Cm_devide_map�̂ǂ��������Ă��邩�v�Z���ĕԂ�
	inline static int getDevideMapNumY(const float _posy)
	{
		int tmp = (int)( (_posy - MapConst::MAP_Y_MIN) / ( ((float)MapConst::MAP_Y_MAX - MapConst::MAP_Y_MIN) / MapConst::LP_DIVIDE_NUM) );
		if (tmp < 0)return 0;
		if (tmp >= MapConst::LP_DIVIDE_NUM * MapConst::LP_DIVIDE_NUM)return MapConst::LP_DIVIDE_NUM * MapConst::LP_DIVIDE_NUM - 1;
		return tmp;
	}

};
