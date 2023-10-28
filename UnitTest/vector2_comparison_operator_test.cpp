#include "pch.h"

#include "../DesignLab/designlab_vector2.h"


namespace dl = ::designlab;


namespace
{
	// �e�X�g�p�̃w���p�[�֐�

	//! @brief �^����ꂽ�l����Vector2���쐬����
	dl::Vector2 MakeVec2(const float x, const float y)
	{
		dl::Vector2 vec;
		vec.x = x;
		vec.y = y;

		return vec;
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
	TEST(Vector2Test, EqualityOperatorTest)
	{
		std::vector<std::tuple<dl::Vector2, dl::Vector2, bool>> testcase_list{
			{ MakeVec2(10.f, -5.f),			MakeVec2(10.f, -5.f),			true, },
			{ MakeVec2(-20.f, 60.f),		MakeVec2(-20.f, 60.f),			true, },
			{ MakeVec2(0.4f, 6.6f),			MakeVec2(0.4f, 6.6f),			true, },
			{ MakeVec2(100000.f, -5200.f),	MakeVec2(100000.f, -5200.f),	true, },

			{ MakeVec2(10.f, -5.f),			MakeVec2(13.f, -5.f),			false, },
			{ MakeVec2(10.f, -5.f),			MakeVec2(10.f, 55.f),			false, },
			{ MakeVec2(4.f, 8.f),			MakeVec2(-9.f, 3.f),			false, },
		};

		for (const auto& i : testcase_list)
		{
			const dl::Vector2& vec1 = std::get<0>(i);
			const dl::Vector2& vec2 = std::get<1>(i);
			const bool expected = std::get<2>(i);

			if (expected == true)
			{
				std::string error_case_message = ToString(vec1) + "��" + ToString(vec2) + "�͓������͂��ł���D";
				EXPECT_TRUE(vec1 == vec2) << error_case_message;
			}
			else
			{
				std::string error_case_message = ToString(vec1) + "��" + ToString(vec2) + "�͓������Ȃ��͂��ł���D";
				EXPECT_FALSE(vec1 == vec2) << error_case_message;
			}
		}
	}

	TEST(Vector2Test, InequalityOperatorTest)
	{
		std::vector<std::tuple<dl::Vector2, dl::Vector2, bool>> testcase_list{
			{ MakeVec2(10.f, -5.f),			MakeVec2(10.f, -5.f),			false, },
			{ MakeVec2(-20.f, 60.f),		MakeVec2(-20.f, 60.f),			false, },
			{ MakeVec2(0.4f, 6.6f),			MakeVec2(0.4f, 6.6f),			false, },
			{ MakeVec2(100000.f, -5200.f),	MakeVec2(100000.f, -5200.f),	false, },

			{ MakeVec2(10.f, -5.f),			MakeVec2(13.f, -5.f),			true, },
			{ MakeVec2(10.f, -5.f),			MakeVec2(10.f, 55.f),			true, },
			{ MakeVec2(4.f, 8.f),			MakeVec2(-9.f, 3.f),			true, },
		};

		for (const auto& i : testcase_list)
		{
			const dl::Vector2& vec1 = std::get<0>(i);
			const dl::Vector2& vec2 = std::get<1>(i);
			const bool expected = std::get<2>(i);

			if (expected == true)
			{
				std::string error_case_message = ToString(vec1) + "��" + ToString(vec2) + "�͓������Ȃ��͂��ł���D";
				EXPECT_TRUE(vec1 != vec2) << error_case_message;
			}
			else
			{
				std::string error_case_message = ToString(vec1) + "��" + ToString(vec2) + "�͓������͂��ł���D";
				EXPECT_FALSE(vec1 != vec2) << error_case_message;
			}
		}
	}

}	// namespace designlab::test::common::math