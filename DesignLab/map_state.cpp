#include "map_state.h"


MapState& MapState::operator=(const MapState& other)
{
	//�������g�ւ̑���͖�������D
	if (this == &other) { return *this; }

	//�����o�ϐ����R�s�[����D
	m_map_data.clear();

	for (const auto& i : other.m_map_data)
	{
		m_map_data.push_back(i);
	}


	m_devide_map.clear();

	for (const auto& i : other.m_devide_map)
	{
		m_devide_map.push_back(i);
	}


	m_devide_map_top_z.clear();

	for (const auto& i : other.m_devide_map_top_z)
	{
		m_devide_map_top_z.push_back(i);
	}


	return *this;
}

void MapState::init(const EMapCreateMode mode, const int option, const bool do_output)
{
	//�����̎w��ʂ�Ƀ}�b�v�𐶐�����D
	MapCreator creator;
	creator.create(mode, option, do_output, &m_map_data);

	//�n�`�𕪂���D
	makeDevideMap();
}


int MapState::getPointNumFromDevideMap(const int x, const int y) const
{
	if (getDevideMapNum(x, y) >= MapConst::LP_DIVIDE_NUM * MapConst::LP_DIVIDE_NUM) { return 0; }
	return static_cast<int>(m_devide_map[getDevideMapNum(x, y)].size());
}


dl_vec::SVector MapState::getPosFromDevideMap(const int x, const int y, const int num) const
{
	//���݂��Ă��Ȃ���ΑS��0�̃x�N�g����Ԃ��D
	if (getDevideMapNum(x, y) >= MapConst::LP_DIVIDE_NUM * MapConst::LP_DIVIDE_NUM) { return dl_vec::SVector{0, 0, 0}; }

	if (num < 0 || static_cast<int>(m_devide_map[getDevideMapNum(x, y)].size()) <= num) { return dl_vec::SVector{0, 0, 0}; }

	//���݂��Ă���Ȃ�Βl��Ԃ��D
	return m_devide_map[getDevideMapNum(x, y)][num];
}


dl_vec::SVector MapState::getPos(const int num) const
{
	if (num < 0 || static_cast<int>(m_map_data.size()) <= num) { return dl_vec::SVector{0, 0, 0}; }

	return m_map_data[num];
}

void MapState::makeDevideMap()
{
	//�}�b�v��؂蕪����l�p�`�̕ӂ̒������Z�o����D
	const float x_length = (MapConst::MAP_MAX_FORWARD - MapConst::MAP_MIN_FORWARD) / static_cast<float>(MapConst::LP_DIVIDE_NUM);
	const float y_length = (MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / static_cast<float>(MapConst::LP_DIVIDE_NUM);

	m_devide_map.resize(MapConst::LP_DIVIDE_NUM * MapConst::LP_DIVIDE_NUM);

	//�}�b�v�̃f�[�^�S�Ăɏ���������D
	for (const auto& i : m_map_data)
	{
		//xy�����̃u���b�N�ԍ������ꂼ�ꋁ�߂�
		int x = (int)((i.x - (float)MapConst::MAP_MIN_FORWARD) / x_length);
		int y = (int)((i.y - (float)MapConst::MAP_MIN_HORIZONTAL) / y_length);

		//�}�b�v�͈͓̔��ɂ��鎞�̂ݒǉ�����
		if (0 <= x && x < MapConst::LP_DIVIDE_NUM)
		{
			if (0 <= y && y < MapConst::LP_DIVIDE_NUM)
			{
				//�l��}������
				m_devide_map[getDevideMapNum(x, y)].push_back(i);
			}
		}
	}

	// m_devide_map_top_z���X�V����D
	for (int i = 0; i < MapConst::LP_DIVIDE_NUM * MapConst::LP_DIVIDE_NUM; i++)
	{
		m_devide_map_top_z.push_back(kMapMinZ);	 //�����l�͍ŏ��l�D

		//�S�Ă̗v�f���Q�Ƃ���D
		for (const auto& i : m_devide_map[i])
		{
			//���ݒl�Ɣ�ׂđ傫�����̂�ǉ��D
			m_devide_map_top_z.back() = std::max(i.z, m_devide_map_top_z.back());
		}
	}
}
