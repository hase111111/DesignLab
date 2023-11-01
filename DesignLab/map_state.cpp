#include "map_state.h"

#include "designlab_math_util.h"


namespace dlm = ::designlab::math_util;


DevideMapState::DevideMapState() :
	devided_map_point_(dlm::Squared(kDevideNum)),
	devided_map_top_z_(dlm::Squared(kDevideNum))
{
	// �R���X�g���N�^�ŁCvector�̑傫�����m�ۂ��Ă����D
	Clear();
}

DevideMapState::DevideMapState(const MapState& map_state) :
	devided_map_point_(dlm::Squared(kDevideNum)),
	devided_map_top_z_(dlm::Squared(kDevideNum))
{
	assert(false);
	Init(map_state, {});
}

void DevideMapState::Init(const MapState& map_state, const dl::Vector3 global_robot_com)
{
	Clear();


	//�}�b�v�̃f�[�^�S�Ă��Q�Ƃ��C�؂蕪����
	const size_t kMapPointSize = map_state.GetMapPointSize();

	for (size_t i = 0; i < kMapPointSize; ++i)
	{
		//_ xy�����̃u���b�N�ԍ������ꂼ�ꋁ�߂�
		const designlab::Vector3 point = map_state.GetMapPoint(i);

		//�͈͓��ɂ��Ȃ��Ȃ�Ώ�������߁Ccontinue
		if (point.x < global_robot_com.x + kDevideMapMinX || global_robot_com.x + kDevideMapMaxX < point.x) { continue; }
		if (point.y < global_robot_com.y + kDevideMapMinY || global_robot_com.y + kDevideMapMaxY < point.y) { continue; }

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

designlab::Vector3 DevideMapState::GetPointPos(int x_index, int y_index, int devide_map_index) const
{
	//���݂��Ă��Ȃ���ΑS��0�̃x�N�g����Ԃ��D
	if (GetDevideMapIndex(x_index, y_index) >= MapConst::LP_DIVIDE_NUM * MapConst::LP_DIVIDE_NUM) { return designlab::Vector3{0, 0, 0}; }

	if (devide_map_index < 0 || static_cast<int>(devided_map_point_[GetDevideMapIndex(x_index, y_index)].size()) <= devide_map_index) { return designlab::Vector3{0, 0, 0}; }

	//���݂��Ă���Ȃ�Βl��Ԃ��D
	return devided_map_point_[GetDevideMapIndex(x_index, y_index)][devide_map_index];
}

void DevideMapState::GetPointVector(int x_index, int y_index, std::vector<designlab::Vector3>* point_vec) const
{
	if (point_vec == nullptr) { return; }

	//���݂��Ă��Ȃ���ΏI��
	if (GetDevideMapIndex(x_index, y_index) >= MapConst::LP_DIVIDE_NUM * MapConst::LP_DIVIDE_NUM) { return; }

	(*point_vec) = devided_map_point_[GetDevideMapIndex(x_index, y_index)];
}

float DevideMapState::GetTopZ(int x_index, int y_index) const
{
	if (GetDevideMapIndex(x_index, y_index) >= MapConst::LP_DIVIDE_NUM * MapConst::LP_DIVIDE_NUM) { return kMapMinZ; }
	return devided_map_top_z_[GetDevideMapIndex(x_index, y_index)];
}
