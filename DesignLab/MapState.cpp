#include "MapState.h"

MapState::MapState(const EMapCreateMode _mode, const int _option, const bool _do_output)
{
	//�R���X�g���N�^�̎w��ʂ�Ƀ}�b�v�𐶐�����D
	map_creator::createMap(_mode, _option, m_map_data, _do_output);

	//�n�`�𕪂���D
	makeDevideMap();
}

int MapState::getPointNumFromDevideMap(const int _x, const int _y) const
{
	if (getDevideMapNum(_x, _y) >= MapConst::LP_DIVIDE_NUM * MapConst::LP_DIVIDE_NUM) { return 0; }
	return (int)m_devide_map.at(getDevideMapNum(_x, _y)).size();
}

myvector::SVector MapState::getPosFromDevideMap(const int _x, const int _y, const int _num) const
{
	//���݂��Ă��Ȃ���ΑS��0�̃x�N�g����Ԃ��D
	if (getDevideMapNum(_x, _y) >= MapConst::LP_DIVIDE_NUM * MapConst::LP_DIVIDE_NUM) { return myvector::SVector(0, 0, 0); }

	if (_num < 0 || m_devide_map.at(getDevideMapNum(_x, _y)).size() <= _num ) { return myvector::SVector(0, 0, 0); }

	//���݂��Ă���Ȃ�Βl��Ԃ��D
	return m_devide_map.at(getDevideMapNum(_x, _y)).at(_num);
}

myvector::SVector MapState::getPos(const int _num) const
{
	if (_num < 0 || m_map_data.size() <= _num) { return myvector::SVector(0, 0, 0); }

	return m_map_data.at(_num);
}

void MapState::makeDevideMap()
{
	//�}�b�v��؂蕪����l�p�`�̕ӂ̒������Z�o����D
	const float _lengthX = (MapConst::MAP_X_MAX - MapConst::MAP_X_MIN) / (float)MapConst::LP_DIVIDE_NUM;
	const float _lengthY = (MapConst::MAP_Y_MAX - MapConst::MAP_Y_MIN) / (float)MapConst::LP_DIVIDE_NUM;

	m_devide_map.resize(MapConst::LP_DIVIDE_NUM * MapConst::LP_DIVIDE_NUM);

	//�}�b�v�̃f�[�^�S�Ăɏ���������D
	for (const auto &i : m_map_data)
	{
		//xy�����̃u���b�N�ԍ������ꂼ�ꋁ�߂�
		int x = (int)((i.x - (float)MapConst::MAP_X_MIN) / _lengthX);
		int y = (int)((i.y - (float)MapConst::MAP_Y_MIN) / _lengthY);

		//�}�b�v�͈͓̔��ɂ��鎞�̂ݒǉ�����
		if (0 <= x && x < MapConst::LP_DIVIDE_NUM) 
		{
			if (0 <= y && y < MapConst::LP_DIVIDE_NUM)
			{
				//�l��}������
				m_devide_map.at(getDevideMapNum(x, y)).push_back(i);
			}
		}
	}
}
