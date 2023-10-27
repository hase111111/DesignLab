#include "pch.h"

#include "../DesignLab/designlab_vector3.h"


namespace dl = ::designlab;


namespace 
{
	// �e�X�g�p�̃w���p�[�֐�

	//! @brief �^����ꂽ�l����Vector3���쐬����
	dl::Vector3 MakeVec3(float x, float y, float z)
	{
		dl::Vector3 vec;
		vec.x = x;
		vec.y = y;
		vec.z = z;

		return vec;
	}

	//! @brief �^����ꂽ�l����Vector2���쐬����
	dl::Vector2 MakeVec2(float x, float y)
	{
		dl::Vector2 vec;
		vec.x = x;
		vec.y = y;

		return vec;
	}

	//! @brief Vector3�𕶎���ɕϊ�����
	std::string ToString(const dl::Vector3& vec)
	{
		std::stringstream ss;
		ss << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";

		return ss.str();
	}

	//! @brief Vector2�𕶎���ɕϊ�����
	std::string ToString(const dl::Vector2& vec)
	{
		std::stringstream ss;
		ss << "(" << vec.x << ", " << vec.y << ")";

		return ss.str();
	}
}


namespace designlab::test::common::math
{
	TEST(Vector3Test, GetSquaredLength)
	{
		std::vector<std::tuple<dl::Vector3, float>> testcase_list{
			std::make_tuple(MakeVec3(0,0,0),			0.f),
			std::make_tuple(MakeVec3(1,1,1),			3.f),
			std::make_tuple(MakeVec3(1,1,-1),			3.f),
			std::make_tuple(MakeVec3(-1,-1,-1),			3.f),
			std::make_tuple(MakeVec3(0, 5, -12),		169.f),
			std::make_tuple(MakeVec3(4, -10, 5),		141.f),
			std::make_tuple(MakeVec3(100, -80, -35),	17625.f),
			std::make_tuple(MakeVec3(0.1f, -0.7f, 3.5f),12.75f),
		};

		for (const auto& data : testcase_list)
		{
			const dl::Vector3 vec = std::get<0>(data);
			const float length = std::get<1>(data);

			std::string error_case_message = ToString(vec) + "�̒����̓��͐�������" + std::to_string(length) + "�ł���D";
			EXPECT_FLOAT_EQ(vec.GetSquaredLength(), length) << error_case_message;
		}
	}

	TEST(Vector3Test, GetLength)
	{
		std::vector<std::tuple<dl::Vector3, float>> testcase_list{
			std::make_tuple(MakeVec3(0,0,0),			0.f),
			std::make_tuple(MakeVec3(1,1,1),			1.7320508f),
			std::make_tuple(MakeVec3(1,1,-1),			1.7320508f),
			std::make_tuple(MakeVec3(-1,-1,-1),			1.7320508f),
			std::make_tuple(MakeVec3(0, 5, -12),		13.f),
			std::make_tuple(MakeVec3(4, -10, 5),		11.874342f),
			std::make_tuple(MakeVec3(100, -80, -35),	132.75919f),
		};

		for (const auto &i : testcase_list)
		{
			const dl::Vector3 vec = std::get<0>(i);
			const float length = std::get<1>(i);

			std::string error_case_message = ToString(vec) + "�̒����͐�������" + std::to_string(length) + "�ł���D";
			EXPECT_FLOAT_EQ(vec.GetLength(), length) << error_case_message;
		}
	}

	TEST(Vector3Test, GetDistanceFrom)
	{
		std::vector<std::tuple<dl::Vector3, dl::Vector3, float>> testcase_list{
			std::make_tuple(MakeVec3(0,0,0),	MakeVec3(0.f, 0.f, 0.f),		0.f),
			std::make_tuple(MakeVec3(0,0,0),	MakeVec3(8.f, 7.f, -10.f),		14.594519f),
			std::make_tuple(MakeVec3(0,0,0),	MakeVec3(16.f, -30.f, 100.f),	105.621967f),
			std::make_tuple(MakeVec3(0,0,0),	MakeVec3(-4.6f, 0.4f, 34.6f),	34.906732f),
			std::make_tuple(MakeVec3(0,0,0),	MakeVec3(0.1f, -0.7f, 3.5f),	3.570714f),

			std::make_tuple(MakeVec3(10.f, 0, 5.f),			MakeVec3(26.f, -30.f, 105.f),	105.621967f),
			std::make_tuple(MakeVec3(26.f, -30.f, 105.f),	MakeVec3(10.f, 0, 5.f),			105.621967f),
			std::make_tuple(MakeVec3(0.2f, 1.f, -3.f),	MakeVec3(-4.4f, 1.4f, 31.6f),		34.906732f),
		};

		for (const auto &i : testcase_list)
		{
			const dl::Vector3 vec = std::get<0>(i);
			const dl::Vector3 other = std::get<1>(i);
			const float distance = std::get<2>(i);

			std::string error_case_message = ToString(vec) + "��" + ToString(other) + "�̋����͐�������" + std::to_string(distance) + "�ł���D";
			EXPECT_FLOAT_EQ(vec.GetDistanceFrom(other), distance) << error_case_message;
		}
	}

	TEST(Vector3Test, GetNormalized)
	{
		std::vector<std::tuple<dl::Vector3, dl::Vector3>> testcase_list{
			std::make_tuple(MakeVec3(500.f, 0.f, 0.f),			MakeVec3(1.f, 0.f, 0.f)),
			std::make_tuple(MakeVec3(0.f, 54.10f, 0.f),			MakeVec3(0.f, 1.f, 0.f)),
			std::make_tuple(MakeVec3(0.f, 0.f, 1.f),			MakeVec3(0.f, 0.f, 1.f)),
			std::make_tuple(MakeVec3(-23445.f, 0.f, 0.f),		MakeVec3(-1.f, 0.f, 0.f)),
			std::make_tuple(MakeVec3(5.4f, 5.4f, 5.4f),			MakeVec3(0.57735f, 0.57735f, 0.57735f)),
			std::make_tuple(MakeVec3(-94, -94, -94),			MakeVec3(-0.57735f, -0.57735f, -0.57735f)),
			std::make_tuple(MakeVec3(17, 17, -17),				MakeVec3(0.57735f, 0.57735f, -0.57735f)),
			std::make_tuple(MakeVec3(3, 2, -1),					MakeVec3(0.801784f, 0.534522f, -0.267261f)),
			std::make_tuple(MakeVec3(62.51f, -94.78f, 25.89f),	MakeVec3(0.536788f, -0.813898f, 0.222324f)),
		};

		for (const auto& i : testcase_list)
		{
			const dl::Vector3 vec = std::get<0>(i);
			const dl::Vector3 normalized_vec = std::get<1>(i);

			std::string error_case_message = ToString(vec) + "�̐��K���͐�������" + ToString(normalized_vec) + "�ł���D";
			EXPECT_EQ(vec.GetNormalized(), normalized_vec) << error_case_message;

			ASSERT_FLOAT_EQ(normalized_vec.GetNormalized().GetLength(), 1.f) << "�e�X�g�P�[�X" + ToString(vec) + "�̓����̃x�N�g���̒�����1�ł͂���܂���D";
			EXPECT_FLOAT_EQ(vec.GetNormalized().GetLength(), 1.f) << ToString(vec) + "�̐��K�����ꂽ�x�N�g���̒�����1�ɂȂ�D";
		}
	}

	TEST(Vector3Test, GetNormalizedTestZeroVec) 
	{
		dl::Vector3 zero_vec = MakeVec3(0.f, 0.f, 0.f);
		EXPECT_EQ(zero_vec.GetNormalized(), zero_vec) << "������0�̃x�N�g����n�����ƁC���̂܂�0�x�N�g����Ԃ��D";
	}

	TEST(Vector3Test, IsZero)
	{
		dl::Vector3 zero_vec = MakeVec3(0.f, 0.f, 0.f);
		EXPECT_TRUE(zero_vec.IsZero()) << "������0�̃x�N�g����n������true�ɂȂ�D";

		std::vector<dl::Vector3> false_data_list{
			MakeVec3(-10.f, 0.f, 0.f),
			MakeVec3(512.f, 0.f, 0.f),
			MakeVec3(0.f, 45.f, 0.f),
			MakeVec3(0.f, -3.f, 0.f),
			MakeVec3(0.f, 0.f, 0.5f),
			MakeVec3(0.f, 0.f, -0.2f),
			MakeVec3(0.f, 100.f, -35.f),
			MakeVec3(67.f, 0.f, 26.8f),
			MakeVec3(-71.f, 0.7f, 0.f),
			MakeVec3(24.f, 67.f, 89.f),
		};

		for (const auto& vec : false_data_list)
		{
			EXPECT_FALSE(vec.IsZero()) << ToString(vec) + "�͒�����0�ł͂Ȃ��̂�false�ɂȂ�D";
		}
	}

	TEST(Vector3Test, Dot)
	{
		std::vector<std::tuple<dl::Vector3, dl::Vector3, float>> testcase_list{
			std::make_tuple(MakeVec3(0,0,0),			MakeVec3(0.f, 0.f, 0.f),		0.f),
			std::make_tuple(MakeVec3(1,1,1),			MakeVec3(1.f, 1.f, 1.f),		3.f),
			std::make_tuple(MakeVec3(1,1,-1),			MakeVec3(1.f, 1.f, -1.f),		3.f),
			std::make_tuple(MakeVec3(-1,-1,-1),			MakeVec3(-1.f, -1.f, -1.f),		3.f),
			std::make_tuple(MakeVec3(0, 5, -12),		MakeVec3(0.f, 5.f, -12.f),		169.f),
			std::make_tuple(MakeVec3(4, -10, 5),		MakeVec3(4.f, -10.f, 5.f),		141.f),
			std::make_tuple(MakeVec3(100, -80, -35),	MakeVec3(100.f, -80.f, -35.f),	17625.f),
			std::make_tuple(MakeVec3(0.1f, -0.7f, 3.5f),MakeVec3(0.1f, -0.7f, 3.5f),	12.75f),
		};

		for (const auto& i : testcase_list)
		{
			const dl::Vector3 vec = std::get<0>(i);
			const dl::Vector3 other = std::get<1>(i);
			const float dot = std::get<2>(i);

			std::string error_case_message = ToString(vec) + "��" + ToString(other) + "�̓��ς͐�������" + std::to_string(dot) + "�ł���D";
			EXPECT_FLOAT_EQ(vec.Dot(other), dot) << error_case_message;
		}
	}

	TEST(Vector3Test, Cross)
	{
		std::vector<std::tuple<dl::Vector3, dl::Vector3, dl::Vector3>> testcase_list{
			std::make_tuple(MakeVec3(0,0,0),			MakeVec3(0.f, 0.f, 0.f),		MakeVec3(0.f, 0.f, 0.f)),
			std::make_tuple(MakeVec3(3.f, 4.f, 1.f),		MakeVec3(3.f, 7.f, 5.f),		MakeVec3(13.f, -12.f, 9.f)),
			std::make_tuple(MakeVec3(3.f, 7.f, 5.f),		MakeVec3(3.f, 4.f, 1.f),		MakeVec3(-13.f, 12.f, -9.f)),
			std::make_tuple(MakeVec3(1.f, 1.f, 1.f),		MakeVec3(3.f, 4.f, 1.f),		MakeVec3(-3.f, 2.f, 1.f)),
		};

		for (const auto& i : testcase_list)
		{
			const dl::Vector3 vec = std::get<0>(i);
			const dl::Vector3 other = std::get<1>(i);
			const dl::Vector3 cross = std::get<2>(i);

			std::string error_case_message = ToString(vec) + "��" + ToString(other) + "�̊O�ς͐�������" + ToString(cross) + "�ł���D";
			EXPECT_EQ(vec.Cross(other), cross) << error_case_message;
		}
	}

	TEST(Vector3Test, ProjectXY) 
	{
		std::vector<std::tuple<dl::Vector3, dl::Vector2>> testcase_list{
			std::make_tuple(MakeVec3(0,0,0),			MakeVec2(0.f, 0.f)),
			std::make_tuple(MakeVec3(1,1,1),			MakeVec2(1.f, 1.f)),
			std::make_tuple(MakeVec3(1,1,-1),			MakeVec2(1.f, 1.f)),
			std::make_tuple(MakeVec3(-1,10,-1),			MakeVec2(-1.f, 10.f)),
		};

		for (const auto& i : testcase_list)
		{
			const dl::Vector3 vec = std::get<0>(i);
			const dl::Vector2 projected_vec = std::get<1>(i);

			std::string error_case_message = ToString(vec) + "��XY���ʂɎˉe�����x�N�g���͐�������" + ToString(projected_vec) + "�ł���D";
			EXPECT_EQ(vec.ProjectedXY(), projected_vec) << error_case_message;
		}
	}

}	// namespace designlab::test::common::math
