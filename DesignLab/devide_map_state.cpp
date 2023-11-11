#include "devide_map_state.h"

#include "designlab_math_util.h"


namespace dl = ::designlab;
namespace dlm = ::designlab::math_util;


DevideMapState::DevideMapState() :
	global_robot_com_{},
	devided_map_point_(dlm::Squared(kDevideNum)),
	devided_map_top_z_(dlm::Squared(kDevideNum))
{
	// �R���X�g���N�^�ŁCvector�̑傫�����m�ۂ��Ă����D
	Clear();
}

void DevideMapState::Init(const MapState& map_state, const dl::Vector3 global_robot_com)
{
	assert(devided_map_point_.size() == dlm::Squared(kDevideNum));	//vector�̑傫�����m�ۂ���Ă��邩�m�F
	assert(devided_map_top_z_.size() == dlm::Squared(kDevideNum));	//vector�̑傫�����m�ۂ���Ă��邩�m�F

	Clear();

	global_robot_com_ = global_robot_com;	//���{�b�g�̈ʒu���X�V����D

	//�}�b�v�̃f�[�^�S�Ă��Q�Ƃ��C�؂蕪����
	const size_t kMapPointSize = map_state.GetMapPointSize();

	for (size_t i = 0; i < kMapPointSize; ++i)
	{
		//_ xy�����̃u���b�N�ԍ������ꂼ�ꋁ�߂�
		const designlab::Vector3 point = map_state.GetMapPoint(i);

		//�͈͓��ɂ��Ȃ��Ȃ�Ώ�������߁Ccontinue
		if (!IsInMap(point)) { continue; }

		const int x = GetDevideMapIndexX(point.x);
		const int y = GetDevideMapIndexY(point.y);

		//�}�b�v�͈͓̔��ɂ��鎞�̂ݒǉ�����
		if (IsVaildIndex(x) && IsVaildIndex(y))
		{
			devided_map_point_[GetDevideMapIndex(x, y)].push_back(point);

			//�ő�l���X�V����
			devided_map_top_z_[GetDevideMapIndex(x, y)] = std::max(point.z, devided_map_top_z_[GetDevideMapIndex(x, y)]);
		}
	}
}

void DevideMapState::Clear()
{
	// vector�̒��g��S�ăN���A����D
	for (auto& i : devided_map_point_)
	{
		i.clear();
	}

	// �ŏ��l�Ŗ��߂�D
	for (auto& i : devided_map_top_z_)
	{
		i = kMapMinZ;
	}
}

int DevideMapState::GetPointNum(const int x_index, const int y_index) const
{
	//���݂��Ă��Ȃ���ΏI��
	if (!IsVaildIndex(x_index) || !IsVaildIndex(y_index)) { return 0; }

	return static_cast<int>(devided_map_point_[GetDevideMapIndex(x_index, y_index)].size());
}

designlab::Vector3 DevideMapState::GetPointPos(int x_index, int y_index, int devide_map_index) const
{
	//���݂��Ă��Ȃ���ΏI��
	if (!IsVaildIndex(x_index) || !IsVaildIndex(y_index)) { return designlab::Vector3{ 0, 0, 0 }; }

	if (devide_map_index < 0 || static_cast<int>(devided_map_point_[GetDevideMapIndex(x_index, y_index)].size()) <= devide_map_index) { return designlab::Vector3{ 0, 0, 0 }; }

	//���݂��Ă���Ȃ�Βl��Ԃ��D
	return devided_map_point_[GetDevideMapIndex(x_index, y_index)][devide_map_index];
}

void DevideMapState::GetPointVector(int x_index, int y_index, std::vector<designlab::Vector3>* point_vec) const
{
	if (point_vec == nullptr) { return; }

	//���݂��Ă��Ȃ���ΏI��
	if (!IsVaildIndex(x_index) || !IsVaildIndex(y_index)) { return; }

	(*point_vec) = devided_map_point_[GetDevideMapIndex(x_index, y_index)];
}

float DevideMapState::GetTopZ(int x_index, int y_index) const
{
	//���݂��Ă��Ȃ���ΏI��
	if (!IsVaildIndex(x_index) || !IsVaildIndex(y_index)) { return kMapMinZ; }

	return devided_map_top_z_[GetDevideMapIndex(x_index, y_index)];
}