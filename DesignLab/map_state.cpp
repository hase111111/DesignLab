#include "map_state.h"


MapState_Old& MapState_Old::operator=(const MapState_Old& other)
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

void MapState_Old::init(const EMapCreateMode mode, const int option, const bool do_output)
{
	//�����̎w��ʂ�Ƀ}�b�v�𐶐�����D
	MapCreator creator;
	creator.create(mode, option, do_output, &m_map_data);

	//�n�`�𕪂���D
	makeDevideMap();
}


int MapState_Old::getPointNumFromDevideMap(const int x, const int y) const
{
	if (getDevideMapNum(x, y) >= MapConst::LP_DIVIDE_NUM * MapConst::LP_DIVIDE_NUM) { return 0; }
	return static_cast<int>(m_devide_map[getDevideMapNum(x, y)].size());
}


dl_vec::SVector MapState_Old::getPosFromDevideMap(const int x, const int y, const int num) const
{
	//���݂��Ă��Ȃ���ΑS��0�̃x�N�g����Ԃ��D
	if (getDevideMapNum(x, y) >= MapConst::LP_DIVIDE_NUM * MapConst::LP_DIVIDE_NUM) { return dl_vec::SVector{0, 0, 0}; }

	if (num < 0 || static_cast<int>(m_devide_map[getDevideMapNum(x, y)].size()) <= num) { return dl_vec::SVector{0, 0, 0}; }

	//���݂��Ă���Ȃ�Βl��Ԃ��D
	return m_devide_map[getDevideMapNum(x, y)][num];
}


dl_vec::SVector MapState_Old::getPos(const int num) const
{
	if (num < 0 || static_cast<int>(m_map_data.size()) <= num) { return dl_vec::SVector{0, 0, 0}; }

	return m_map_data[num];
}

void MapState_Old::makeDevideMap()
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




MapState& MapState::operator=(const MapState& other)
{
	//�������g�ւ̑���͖�������D
	if (this == &other) { return *this; }

	//�����o�ϐ����R�s�[����D
	map_point_.clear();

	map_point_ = other.map_point_;

	return *this;
}


DevideMapState::DevideMapState() :
	devided_map_point_(MapConst::LP_DIVIDE_NUM* MapConst::LP_DIVIDE_NUM),
	devided_map_top_z_(MapConst::LP_DIVIDE_NUM* MapConst::LP_DIVIDE_NUM)
{
	// �R���X�g���N�^�ŁCvector�̑傫�����m�ۂ��Ă����D
	Clear();
}

DevideMapState::DevideMapState(const MapState& map_state) :
	devided_map_point_(MapConst::LP_DIVIDE_NUM* MapConst::LP_DIVIDE_NUM),
	devided_map_top_z_(MapConst::LP_DIVIDE_NUM* MapConst::LP_DIVIDE_NUM)
{
	Init(map_state);
}

void DevideMapState::Init(const MapState& map_state)
{
	Clear();

	//�}�b�v��؂蕪����l�p�`�̕ӂ̒������Z�o����D
	constexpr float kLengthX = (MapConst::MAP_MAX_FORWARD - MapConst::MAP_MIN_FORWARD) / static_cast<float>(MapConst::LP_DIVIDE_NUM);
	constexpr float kLengthY = (MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / static_cast<float>(MapConst::LP_DIVIDE_NUM);

	//�}�b�v�̃f�[�^�S�Ă��Q�Ƃ��C�؂蕪����
	const size_t kMapPointSize = map_state.GetMapPointSize();

	for (size_t i = 0; i < kMapPointSize; ++i)
	{
		//xy�����̃u���b�N�ԍ������ꂼ�ꋁ�߂�
		const dl_vec::SVector point = map_state.map_point(i);

		const int x = static_cast<int>((point.x - static_cast<float>(MapConst::MAP_MIN_FORWARD)) / kLengthX);
		const int y = static_cast<int>((point.y - static_cast<float>(MapConst::MAP_MIN_HORIZONTAL)) / kLengthY);

		//�}�b�v�͈͓̔��ɂ��鎞�̂ݒǉ�����
		if (0 <= x && x < MapConst::LP_DIVIDE_NUM)
		{
			if (0 <= y && y < MapConst::LP_DIVIDE_NUM)
			{
				//�l��}������
				devided_map_point_[GetDevideMapIndex(x, y)].push_back(point);
			}
		}
	}

	// devided_map_top_z_���X�V����D

	for (size_t i = 0; i < MapConst::LP_DIVIDE_NUM * MapConst::LP_DIVIDE_NUM; ++i)
	{
		devided_map_top_z_[i] = kMapMinZ;	 //�����l�͍ŏ��l�D

		//�S�Ă̗v�f���Q�Ƃ���

		for (const auto& point : devided_map_point_[i])
		{
			//���ݒl�Ɣ�ׂđ傫�����̂�ǉ��D
			devided_map_top_z_[i] = std::max(point.z, devided_map_top_z_[i]);
		}
	}
}

void DevideMapState::Clear()
{
	for (auto& i : devided_map_point_)
	{
		i.clear();
	}

	for (auto& i : devided_map_top_z_)
	{
		i = kMapMinZ;
	}
}

int DevideMapState::GetPointNum(const int x, const int y) const
{
	if (GetDevideMapIndex(x, y) >= MapConst::LP_DIVIDE_NUM * MapConst::LP_DIVIDE_NUM) { return 0; }
	return static_cast<int>(devided_map_point_[GetDevideMapIndex(x, y)].size());
}

dl_vec::SVector DevideMapState::GetPointPos(int x_index, int y_index, int devide_map_index) const
{
	//���݂��Ă��Ȃ���ΑS��0�̃x�N�g����Ԃ��D
	if (GetDevideMapIndex(x_index, y_index) >= MapConst::LP_DIVIDE_NUM * MapConst::LP_DIVIDE_NUM) { return dl_vec::SVector{0, 0, 0}; }

	if (devide_map_index < 0 || static_cast<int>(devided_map_point_[GetDevideMapIndex(x_index, y_index)].size()) <= devide_map_index) { return dl_vec::SVector{0, 0, 0}; }

	//���݂��Ă���Ȃ�Βl��Ԃ��D
	return devided_map_point_[GetDevideMapIndex(x_index, y_index)][devide_map_index];
}

void DevideMapState::GetPointVector(int x_index, int y_index, std::vector<dl_vec::SVector>* point_vec) const
{
	if (point_vec == nullptr) { return; }

	//���݂��Ă��Ȃ���ΏI��
	if (GetDevideMapIndex(x_index, y_index) >= MapConst::LP_DIVIDE_NUM * MapConst::LP_DIVIDE_NUM) { return; }

	(*point_vec) = devided_map_point_[GetDevideMapIndex(x_index, y_index)];
}