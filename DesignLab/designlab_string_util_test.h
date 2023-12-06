#pragma once

#include <doctest.h>

#include "designlab_string_util.h"


TEST_SUITE("namespace string_util")
{
	std::string StringVectorToString(const std::vector<std::string>&str_vec)
	{
		std::string str = "[";

		for (const auto& s : str_vec)
		{
			str += s + "]�E[";
		}

		str += "]";

		return str;
	}


	TEST_CASE("Split")
	{
		SUBCASE("������ [a,b,c] �� std::string�^��[,] �ŋ�؂�ƁC[a]�E[b]�E[c] �ɕ�������")
		{
			const std::string str = "a,b,c";
			const std::string delim = ",";

			const std::vector<std::string> ans = { "a", "b", "c" };
			const std::vector<std::string> result = designlab::string_util::Split(str, delim);

			INFO("�������ꂽ������͎��̒ʂ�ł���" << StringVectorToString(result));

			CHECK(result == ans);
		}

		SUBCASE("������ [abc,d, e,f,] �� std::string�^��[,] �ŋ�؂�ƁC[abc]�E[d]�E[ e]�E[f] �ɕ�������")
		{
			std::string str = "abc,d, e,f,";
			const std::string delim = ",";

			const std::vector<std::string> ans = { "abc", "d", " e", "f" };
			const std::vector<std::string> result = designlab::string_util::Split(str, delim);

			INFO("�������ꂽ������͎��̒ʂ�ł���" << StringVectorToString(result));

			CHECK(result == ans);
		}

		SUBCASE("������ [ ,,,there are ,many empty area,  ,] �� std::string�^��[,] �ŋ�؂�ƁC"
			"[ ]�E[]�E[]�E[there are]�E[many empty area]�E[  ]�ɕ�������")
		{
			std::string str = " ,,,there are ,many empty area,  ,";
			const std::string delim = ",";

			const std::vector<std::string> ans = { " ", "", "", "there are ", "many empty area", "  " };
			const std::vector<std::string> result = designlab::string_util::Split(str, delim);

			INFO("�������ꂽ������͎��̒ʂ�ł���" << StringVectorToString(result));

			CHECK(result == ans);
		}

		SUBCASE("������ [��ʑ�w�͑傫����؂ŗY��ȑ��] �� std::string�^��[��] �ŋ�؂�ƁC[���]�E[�w��]�E[����]�E[�؂ŗY]�E[��]�E[��]�ɕ�������")
		{
			std::string str = "��ʑ�w�͑傫����؂ŗY��ȑ��";
			const std::string delim = "��";

			const std::vector<std::string> ans = { "���", "�w��", "����", "�؂ŗY", "��", "��" };
			const std::vector<std::string> result = designlab::string_util::Split(str, delim);

			INFO("�������ꂽ������͎��̒ʂ�ł���" << StringVectorToString(result));

			CHECK(result == ans);
		}
	}
}