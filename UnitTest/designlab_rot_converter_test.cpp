#include "pch.h"

#include "../DesignLab/designlab_rot_converter.h"
#include "../DesignLab/designlab_rot_converter.cpp"


namespace dl = ::designlab;
namespace dlm = ::designlab::math_util;


namespace designlab::test::common::math 
{
	TEST(RotConverterTest, ToRotationMatrixTestIsSameRotCaseOfEulerXYZ) 
	{
		// ��]���������ǂ����̃e�X�g�C��]�����������͊m���߂Ȃ��D

		std::vector<dl::EulerXYZ> testcase_euler_list{
			{dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(0.f)},
			{dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(0.f)},
			{dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(30.f)},
			{dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(0.f)},
		};

		std::vector<dl::Vector3> testcase_vec_list{
			{0.f, 0.f, 0.f},
			{100.f, 100.f, 100.f},
			{-100.f, -100.f, -100.f},
			{214.35f, -93.67f, 193665.5f},
			{0.4f, 0.84f, -0.24f},
			{-0.4f, -0.84f, 0.24f},
		};

		for (const auto& euler : testcase_euler_list)
		{
			for (const auto& vec : testcase_vec_list) 
			{
				const dl::RotationMatrix3x3 rot_mat = dl::ToRotationMatrix(euler);

				std::string error_mes = " euler : " + euler.ToString() + "\n" + 
					" ���̍��W : " + vec.ToString();

				EXPECT_EQ(dl::RotateVector3(vec, rot_mat), dl::RotateVector3(vec, euler));
			}
		}
	}
}