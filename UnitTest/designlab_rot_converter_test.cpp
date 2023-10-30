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
			{ dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(0.f)},
			{ dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(0.f)},
			{ dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(30.f)},
			{ dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(0.f)},
			{ dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(105.f) },
			{ dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(215.f) },
			{ dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(-60.f) },
			{ dlm::ConvertDegToRad(45.f),dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(-135.f) },
			{ dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(-240.f) },
			{ dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(-10.f),dlm::ConvertDegToRad(105.f) },
			{ dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(25.f),dlm::ConvertDegToRad(215.f) },
			{ dlm::ConvertDegToRad(40.f),dlm::ConvertDegToRad(20.f),dlm::ConvertDegToRad(-60.f) },
			{ dlm::ConvertDegToRad(-45.f),dlm::ConvertDegToRad(15.f),dlm::ConvertDegToRad(-135.f) },
			{ dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(-30.f),dlm::ConvertDegToRad(-240.f) },
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

				EXPECT_EQ(dl::RotateVector3(vec, rot_mat), dl::RotateVector3(vec, euler)) << error_mes;
			}
		}
	}

	TEST(RotConverterTest, ToRotationMatrixTestIsSameRotCaseOfQuaternion)
	{
		// ��]���������ǂ����̃e�X�g�C��]�����������͊m���߂Ȃ��D

		std::vector<dl::Quaternion> testcase_qua_list{
			{ 1.f, { 0.f, 0.f, 0.f } },
			{ 0.965926f,	{0.258819f,		0.000000f,		0.000000f} },
			{ 0.965926f,	{0.000000f,		0.258819f,		0.000000f} },
			{ 0.965926f,	{0.000000f,		0.000000f,		0.258819f} },
			{ 0.588018f,	{0.157559f,		0.205335f,		0.766321f} },
			{ -0.290460f,	{-0.077828f,	0.246840f,		0.921220f} },
			{ 0.771739f,	{0.373286f,		-0.027098f,		-0.514143f} },
			{ 0.396677f,	{-0.033783f,	0.396677f,		-0.827136f} },
			{ -0.408494f,	{-0.341506f,	-0.091506f,		-0.841506f} },
			{ 0.f, 1.f, 0.f, 0.f },
			{ 0.f, 0.f, 1.f, 0.f },
			{ 0.f, 0.f, 0.f, 1.f },
			{ 0.5f, 0.5f, 0.5f, 0.5f },
			{ 0.5f, -0.5f, 0.5f, -0.5f },
		};

		std::vector<dl::Vector3> testcase_vec_list{
			{ 100.f, 100.f, 100.f },
			{ 0.f, 0.f, 0.f },
			{ -100.f, -100.f, -100.f },
			{ 214.35f, -93.67f, 351.4f },
			{ 0.4f, 0.84f, -0.24f },
			{ -0.4f, -0.84f, 0.24f },
		};

		for (const auto& qua : testcase_qua_list)
		{
			for (const auto& vec : testcase_vec_list)
			{
				ASSERT_TRUE(dlm::IsEqual(qua.GetNorm(), 1.f)) << "�N�H�[�^�j�I���̃m������1�ł���K�v������܂��D�e�X�g�P�[�X���Ԉ���Ă��܂��D";

				const dl::RotationMatrix3x3 rot_mat = dl::ToRotationMatrix(qua);

				const dl::Vector3 rot_vec = dl::RotateVector3(vec, qua, true);
				const dl::Vector3 rot_vec_by_mat = dl::RotateVector3(vec, rot_mat);

				std::string error_mes = " �N�H�[�^�j�I�� : " + qua.ToString() + "\n" +
					" ��]�p�s�� : \n" + rot_mat.ToString() + "\n" +
					" ���̍��W : " + vec.ToString() + "\n" +
					" ��]��(�N�H�[�^�j�I��) : " + rot_vec.ToString() + "\n" +
					" ��]��(�s��) : " + rot_vec_by_mat.ToString() + "\n_";

				EXPECT_TRUE(dlm::IsEqual(rot_vec.x, rot_vec_by_mat.x)) << error_mes;
				EXPECT_TRUE(dlm::IsEqual(rot_vec.y, rot_vec_by_mat.y)) << error_mes;
				EXPECT_TRUE(dlm::IsEqual(rot_vec.z, rot_vec_by_mat.z)) << error_mes;
			}
		}
	}

	TEST(RotConverterTest, ToQuaternionTestIsSameRotCaseOfEulerXYZ)
	{
		// ��]���������ǂ����̃e�X�g�C��]�����������͊m���߂Ȃ��D

		std::vector<dl::EulerXYZ> testcase_euler_list{
			{dlm::ConvertDegToRad(30.f), dlm::ConvertDegToRad(0.f), dlm::ConvertDegToRad(0.f)},
			{ dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(0.f) },
			{ dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(30.f) },
			{ dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(0.f) },
			{ dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(105.f) },
			{ dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(215.f) },
			{ dlm::ConvertDegToRad(40.f),dlm::ConvertDegToRad(20.f),dlm::ConvertDegToRad(-60.f) },
			{ dlm::ConvertDegToRad(-45.f),dlm::ConvertDegToRad(15.f),dlm::ConvertDegToRad(-135.f) },
			{ dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(-30.f),dlm::ConvertDegToRad(-240.f) },
		};

		std::vector<dl::Vector3> testcase_vec_list{
			{ 100.f, 100.f, 100.f },
			{ 0.f, 0.f, 0.f},
			{ -100.f, -100.f, -100.f },
			{ 214.35f, -93.67f, 5351.4f },
			{ 0.4f, 0.84f, -0.24f },
			{ -0.4f, -0.84f, 0.24f },
		};

		for (const auto& euler : testcase_euler_list)
		{
			for (const auto& vec : testcase_vec_list)
			{
				const dl::Quaternion rot_qua = dl::ToQuaternion(euler);

				ASSERT_FLOAT_EQ(rot_qua.GetNorm(), 1.f) << "�N�H�[�^�j�I���̃m������1�ł���K�v������܂��D";

				const dl::Vector3 rot_vec = dl::RotateVector3(vec, euler);
				const dl::Vector3 rot_vec_by_qua = dl::RotateVector3(vec, rot_qua, true);

				std::string error_mes = " euler : " + euler.ToStringDeg() + "\n" +
					" �N�H�[�^�j�I�� ; " + rot_qua.ToString() + "\n" +
					" ���̍��W : " + vec.ToString() + "\n" +
					" ��]��(�I�C���[) : " + rot_vec.ToString() + "\n" +
					" ��]��(�l����) : " + rot_vec_by_qua.ToString() + "\n_";
					
				EXPECT_EQ(rot_vec, rot_vec_by_qua) << error_mes;
			}
		}
	}

	TEST(RotConverterTest, ToQuaternionTestIsSameRotCaseOfRotationMatrix)
	{
		// ��]���������ǂ����̃e�X�g�C��]�����������͊m���߂Ȃ��D

		std::vector<dl::EulerXYZ> testcase_euler_list{
			{ dlm::ConvertDegToRad(30.f), dlm::ConvertDegToRad(0.f), dlm::ConvertDegToRad(0.f)},
			{ dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(0.f) },
			{ dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(30.f) },
			{ dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(0.f) },
			{ dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(105.f) },
			{ dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(215.f) },
			{ dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(-60.f) },
			{ dlm::ConvertDegToRad(45.f),dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(-135.f) },
			{ dlm::ConvertDegToRad(0.f),dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(-240.f) },
			{ dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(-10.f),dlm::ConvertDegToRad(105.f) },
			{ dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(25.f),dlm::ConvertDegToRad(215.f) },
			{ dlm::ConvertDegToRad(40.f),dlm::ConvertDegToRad(20.f),dlm::ConvertDegToRad(-60.f) },
			{ dlm::ConvertDegToRad(-45.f),dlm::ConvertDegToRad(15.f),dlm::ConvertDegToRad(-135.f) },
			{ dlm::ConvertDegToRad(30.f),dlm::ConvertDegToRad(-30.f),dlm::ConvertDegToRad(-240.f) },
		};

		std::vector<dl::Vector3> testcase_vec_list{
			{0.f, 0.f, 0.f},
			{ 100.f, 100.f, 100.f },
			{ -100.f, -100.f, -100.f },
			{ 214.35f, -93.67f, 665.5f },
			{ 0.4f, 0.84f, -0.24f },
			{ -0.4f, -0.84f, 0.24f },
		};

		for (const auto& euler : testcase_euler_list)
		{
			for (const auto& vec : testcase_vec_list)
			{
				const dl::Quaternion rot_qua = dl::ToQuaternion(euler);

				ASSERT_FLOAT_EQ(rot_qua.GetNorm(), 1.f) << "�N�H�[�^�j�I���̃m������1�ł���K�v������܂��D";

				const dl::Vector3 rot_vec = dl::RotateVector3(vec, euler);
				const dl::Vector3 rot_vec_by_qua = dl::RotateVector3(vec, rot_qua, true);

				std::string error_mes = " euler : " + euler.ToStringDeg() + "\n" +
					" �N�H�[�^�j�I�� ; " + rot_qua.ToString() + "\n" +
					" ���̍��W : " + vec.ToString() + "\n" +
					" ��]��(�I�C���[) : " + rot_vec.ToString() + "\n" +
					" ��]��(�l����) : " + rot_vec_by_qua.ToString() + "\n_";

				EXPECT_TRUE(dlm::IsEqual(rot_vec.x, rot_vec_by_qua.x)) << error_mes;
				EXPECT_TRUE(dlm::IsEqual(rot_vec.y, rot_vec_by_qua.y)) << error_mes;
				EXPECT_TRUE(dlm::IsEqual(rot_vec.z, rot_vec_by_qua.z)) << error_mes;
			}
		}
	}

	TEST(RotConverterTest, ToEulerXYZTestIsSameRotCaseOfRotationMatrix)
	{
		// ��]���������ǂ����̃e�X�g�C��]�����������͊m���߂Ȃ��D

		std::vector<dl::RotationMatrix3x3> testcase_euler_list{
			{ 1.000000f,	0.000000f,	0.000000f,	0.000000f,	 0.866025f, -0.500000f, 0.000000f,	0.500000f,	0.866025f },
			{ 0.866025f,	0.000000f,	0.500000f,	0.000000f,	1.000000f,	0.000000f,	-0.500000f, 0.000000f,	0.866025f },
			{ 0.866025f,	-0.500000f,	0.000000f,	0.500000f,	0.866025f,	0.000000f,	0.000000f,	0.000000f,	1.000000f },
			{ 1.000000f,	0.000000f,	0.000000f,	0.000000f,	0.866025f,	-0.500000f, 0.000000f,	0.500000f,	0.866025f },
			{ -0.258819f,	-0.836516f, 0.482963f,	0.965926f,	-0.224144f, 0.129410f,	0.000000f,	0.500000f,	0.866025f },
			{ -0.819152f,	0.496732f,	-0.286788f, -0.573577f, -0.709406f, 0.409576f,	0.000000f,	0.500000f,	0.866025f },
			{ 0.500000f,	0.750000f,	-0.433013f, -0.866025f, 0.433013f,	-0.250000f, 0.000000f,	0.500000f,	0.866025f },
			{ -0.707107f,	0.500000f,	-0.500000f, -0.707107f, -0.500000f, 0.500000f,	0.000000f,	0.707107f,	0.707107f },
			{ -0.433013f,	-0.866025f, -0.250000f, 0.750000f,	-0.500000f, 0.433013f,	-0.500000f, 0.000000f,	0.866025f },
			{ -0.254887f,	-0.814045f, 0.521885f,	0.951251f,	-0.308010f, -0.015850f, 0.173648f,	0.492404f,	0.852868f },
			{ -0.742404f,	0.323638f,	-0.586596f, -0.519837f, -0.830608f, 0.199648f,	-0.422618f, 0.453154f,	0.784886f },
			{ 0.469846f,	0.773337f,	-0.425669f, -0.813798f, 0.192630f,	-0.548295f, -0.342020f, 0.604023f,	0.719846f },
			{ -0.683013f,	0.629409f,	0.370590f,	-0.683013f, -0.370590f, -0.629409f, -0.258819f, -0.683013f, 0.683013f },
			{ -0.433013f,	-0.625000f, 0.649519f,	0.750000f,	-0.649519f, -0.125000f, 0.500000f,	0.433013f,	0.750000f },
		};

		std::vector<dl::Vector3> testcase_vec_list{
			{0.f, 0.f, 0.f},
			{ 100.f, 100.f, 100.f },
			{ -100.f, -100.f, -100.f },
			{ 214.35f, -93.67f, 665.5f },
			{ 0.4f, 0.84f, -0.24f },
			{ -0.4f, -0.84f, 0.24f },
		};

		for (const auto& rot_mat : testcase_euler_list)
		{
			for (const auto& vec : testcase_vec_list)
			{
				const dl::EulerXYZ rot_euler = dl::ToEulerXYZ(rot_mat);
				const dl::Vector3 rot_vec = dl::RotateVector3(vec, rot_mat);
				const dl::Vector3 rot_vec_by_euler = dl::RotateVector3(vec, rot_euler);

				std::string error_mes = " ��]�p�s�� : \n" + rot_mat.ToString() + "\n" +
					" �I�C���[�p : " + rot_euler.ToStringDeg() + "\n" +
					" ���̍��W : " + vec.ToString() + "\n" +
					" ��]��(�s��) : " + rot_vec.ToString() + "\n" +
					" ��]��(�I�C���[�p) : " + rot_vec_by_euler.ToString() + "\n_";

				EXPECT_TRUE(dlm::IsEqual(rot_vec.x, rot_vec_by_euler.x)) << error_mes;
				EXPECT_TRUE(dlm::IsEqual(rot_vec.y, rot_vec_by_euler.y)) << error_mes;
				EXPECT_TRUE(dlm::IsEqual(rot_vec.z, rot_vec_by_euler.z)) << error_mes;
			}
		}
	}

	TEST(RotConverterTest, ToEulerXYZTestIsSameRotCaseOfQuaternion)
	{
		// ��]���������ǂ����̃e�X�g�C��]�����������͊m���߂Ȃ��D

		std::vector<dl::Quaternion> testcase_qua_list{
			{ 1.f, { 0.f, 0.f, 0.f } },
			{ 0.965926f,	{0.258819f,		0.000000f,		0.000000f} },
			{ 0.965926f,	{0.000000f,		0.258819f,		0.000000f} },
			{ 0.965926f,	{0.000000f,		0.000000f,		0.258819f} },
			{ 0.588018f,	{0.157559f,		0.205335f,		0.766321f} },
			{ -0.290460f,	{-0.077828f,	0.246840f,		0.921220f} },
			{ 0.771739f,	{0.373286f,		-0.027098f,		-0.514143f} },
			{ 0.396677f,	{-0.033783f,	0.396677f,		-0.827136f} },
			{ -0.408494f,	{-0.341506f,	-0.091506f,		-0.841506f} },
			{ 0.f, 1.f, 0.f, 0.f },
			{ 0.f, 0.f, 1.f, 0.f },
			{ 0.f, 0.f, 0.f, 1.f },
			{ 0.5f, 0.5f, 0.5f, 0.5f },
			{ 0.5f, -0.5f, 0.5f, -0.5f },
		};

		std::vector<dl::Vector3> testcase_vec_list{
			{ 100.f, 100.f, 100.f },
			{ 0.f, 0.f, 0.f },
			{ -100.f, -100.f, -100.f },
			{ 214.35f, -93.67f, 351.4f },
			{ 0.4f, 0.84f, -0.24f },
			{ -0.4f, -0.84f, 0.24f },
		};

		for (const auto& qua : testcase_qua_list)
		{
			for (const auto& vec : testcase_vec_list)
			{
				ASSERT_TRUE(dlm::IsEqual(qua.GetNorm(), 1.f)) << "�N�H�[�^�j�I���̃m������1�ł���K�v������܂��D�e�X�g�P�[�X���Ԉ���Ă��܂��D";

				const dl::EulerXYZ rot_euler = dl::ToEulerXYZ(qua);

				const dl::Vector3 rot_vec = dl::RotateVector3(vec, qua, true);
				const dl::Vector3 rot_vec_by_euler = dl::RotateVector3(vec, rot_euler);

				std::string error_mes = " �N�H�[�^�j�I�� : " + qua.ToString() + "\n" +
					" �I�C���[�p : " + rot_euler.ToStringDeg() + "\n" +
					" ���̍��W : " + vec.ToString() + "\n" +
					" ��]��(�N�H�[�^�j�I��) : " + rot_vec.ToString() + "\n" +
					" ��]��(�I�C���[�p) : " + rot_vec_by_euler.ToString() + "\n_";

				EXPECT_TRUE(dlm::IsEqual(rot_vec.x, rot_vec_by_euler.x)) << error_mes;
				EXPECT_TRUE(dlm::IsEqual(rot_vec.y, rot_vec_by_euler.y)) << error_mes;
				EXPECT_TRUE(dlm::IsEqual(rot_vec.z, rot_vec_by_euler.z)) << error_mes;
			}
		}
	}
}