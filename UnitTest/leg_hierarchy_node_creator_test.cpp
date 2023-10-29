#include "pch.h"

#include <cmath>

#include "../DesignLab/leg_hierarchy_node_creator.h"
#include "../DesignLab/leg_hierarchy_node_creator.cpp"
#include "../DesignLab/designlab_string_util.cpp"
#include "../DesignLab/robot_state_node.cpp"
#include "../DesignLab/designlab_euler.cpp"
#include "../DesignLab/designlab_rotation_matrix.cpp"
#include "../DesignLab/hexapod_const.cpp"


namespace dl = ::designlab;
namespace dllf = ::designlab::leg_func;


namespace 
{
	const std::vector<std::array<bool, HexapodConst::kLegNum>> oneleg_lifted_testcase{
		{ false, true, true, true, true, true },
		{ true, false, true, true, true, true },
		{ true, true, false, true, true, true },
		{ true, true, true, false, true, true },
		{ true, true, true, true, false, true },
		{ true, true, true, true, true, false }
	};

	const std::vector<std::array<bool, HexapodConst::kLegNum>> twoleg_lifted_testcase{
		{ false, false, true, true, true, true },
		{ false, true, false, true, true, true },
		{ false, true, true, false, true, true },
		{ false, true, true, true, false, true },
		{ false, true, true, true, true, false },
		{ true, false, false, true, true, true },
		{ true, false, true, false, true, true },
		{ true, false, true, true, false, true },
		{ true, false, true, true, true, false },
		{ true, true, false, false, true, true },
		{ true, true, false, true, false, true },
		{ true, true, false, true, true, false },
		{ true, true, true, false, false, true },
		{ true, true, true, false, true, false },
		{ true, true, true, true, false, false }
	};

	const std::vector<std::array<bool, HexapodConst::kLegNum>> threeleg_lifted_testcase{
		//{ false, false, false, true, true, true },
		{ false, false, true, false, true, true },
		{ false, false, true, true, false, true },
		//{ false, false, true, true, true, false },
		{ false, true, false, false, true, true },
		{ false, true, false, true, false, true },
		{ false, true, false, true, true, false },
		{ false, true, true, false, false, true },
		{ false, true, true, false, true, false },
		//{ false, true, true, true, false, false },
		//{ true, false, false, false, true, true },
		{ true, false, false, true, false, true },
		{ true, false, false, true, true, false },
		{ true, false, true, false, false, true },
		{ true, false, true, false, true, false },
		{ true, false, true, true, false, false },
		//{ true, true, false, false, false, true },
		{ true, true, false, false, true, false },
		{ true, true, false, true, false, false },
		//{ true, true, true, false, false, false } //���ۂ̃��{�b�g�͘A������3�r��V�r����Ɠ|���̂ŁC�����������D
	};

	const std::vector<DiscreteLegPos> all_discrete_leg_pos({
		DiscreteLegPos::kBack,
		DiscreteLegPos::kCenter,
		DiscreteLegPos::kFront,
		DiscreteLegPos::kLowerBack,
		DiscreteLegPos::kLowerFront,
		DiscreteLegPos::kUpperBack,
		DiscreteLegPos::kUpperFront
		}
	);

	//! @brief �e�X�g�p�̃m�[�h���쐬����֐��D
	//! @param [in] is_ground �ڒn���Ă��邩�ǂ����̔z��
	RobotStateNode MakeTestNode(const std::array<bool, HexapodConst::kLegNum>& is_ground)
	{
		RobotStateNode test_node;

		std::array<DiscreteLegPos, HexapodConst::kLegNum> discrete_leg_pos{
			DiscreteLegPos::kCenter, DiscreteLegPos::kCenter, DiscreteLegPos::kCenter,
			DiscreteLegPos::kCenter, DiscreteLegPos::kCenter, DiscreteLegPos::kCenter,
		};


		test_node.leg_state = dllf::MakeLegStateBit(DiscreteComPos::kCenterBack, is_ground, discrete_leg_pos);

		test_node.leg_pos = {
			dl::Vector3{ 0.f, -120.f, -120.f }, dl::Vector3{ 0.f,-120.f,-120.f }, dl::Vector3{ 0.f,-120.f,-120.f },
			dl::Vector3{ 0.f, 120.f, -120.f }, dl::Vector3{ 0.f,120.f,-120.f }, dl::Vector3{ 0.f,120.f,-120.f }
		};
		test_node.leg_reference_pos = {
			dl::Vector3{ 0.f, -120.f, -120.f }, dl::Vector3{ 0.f,-120.f,-120.f }, dl::Vector3{ 0.f,-120.f,-120.f },
			dl::Vector3{ 0.f, 120.f, -120.f }, dl::Vector3{ 0.f,120.f,-120.f }, dl::Vector3{ 0.f,120.f,-120.f }
		};
		test_node.global_center_of_mass = dl::Vector3{ 100.f, 0.f, 0.f };
		test_node.rot = dl::EulerXYZ{ 0.f, 0.f, 1.57f };

		test_node.next_move = HexapodMove::kLegHierarchyChange;
		test_node.depth = 0;
		test_node.parent_num = -1;

		return test_node;
	}

	//! @brief �ڒn�E�V�r�̔z��𕶎���ɕϊ�����֐��D
	std::string ToString(const std::array<bool, HexapodConst::kLegNum>& is_ground)
	{
		std::string str = "{";

		for (const auto& i : is_ground)
		{
			str += i ? "�ڒn" : "�V�r";
			str += ",";
		}

		str += "}";

		return str;
	}
}


namespace designlab::test::node::node_creator 
{
	//�e�X�g�t�B�N�X�`��
	class LegHierarchyNodeCreatorTest : public ::testing::Test
	{
	protected:

		const HexapodMove next_move_ = HexapodMove::kLegHierarchyChange;

		const int next_node_index_ = 0;

		std::unique_ptr<LegHierarchyNodeCreator> creator_ptr_;

		virtual void SetUp() 
		{
			creator_ptr_ = std::make_unique<LegHierarchyNodeCreator>(next_move_);
		}

		virtual void TearDown() 
		{
			creator_ptr_.reset();
		}
	};

	TEST_F(LegHierarchyNodeCreatorTest, CreateTestNodeNumCheckCaseOfOneLegLifted)
	{
		// 1�{�̋r���V�r���Ă���ꍇ�C�o�͂����m�[�h�̐��̃e�X�g

		for (const auto &i : oneleg_lifted_testcase)
		{
			const RobotStateNode test_node = MakeTestNode(i);

			std::vector<RobotStateNode> output_nodes;

			// �e�X�g�Ώۂ̊֐������s
			creator_ptr_->Create(test_node, next_node_index_, &output_nodes);

			// �o�͂��ꂽ�m�[�h�̐����m�F
			std::string error_message = ToString(i) + "\n" + 
				test_node.ToString() + "\n_";

			EXPECT_EQ(output_nodes.size(), all_discrete_leg_pos.size()) << error_message;
		}
	}

	TEST_F(LegHierarchyNodeCreatorTest, CreateTestNodeNumCheckCaseOfTwoLegLifted)
	{
		// 2�{�̋r���V�r���Ă���ꍇ�C�o�͂����m�[�h�̐��̃e�X�g

		for (const auto& i : twoleg_lifted_testcase)
		{
			const RobotStateNode test_node = MakeTestNode(i);

			std::vector<RobotStateNode> output_nodes;

			// �e�X�g�Ώۂ̊֐������s
			creator_ptr_->Create(test_node, next_node_index_, &output_nodes);

			// �o�͂��ꂽ�m�[�h�̐����m�F
			std::string error_message = ToString(i) + "\n" +
				"�e�m�[�h" + test_node.ToString() + "\n_";

			EXPECT_EQ(output_nodes.size(), pow(all_discrete_leg_pos.size(), 2)) << error_message;
		}
	}

	TEST_F(LegHierarchyNodeCreatorTest, CreateTestNodeNumCheckCaseOfThreeLegLifted)
	{
		// 3�{�̋r���V�r���Ă���ꍇ�C�o�͂����m�[�h�̐��̃e�X�g

		for (const auto& i : threeleg_lifted_testcase)
		{
			const RobotStateNode test_node = MakeTestNode(i);

			std::vector<RobotStateNode> output_nodes;

			// �e�X�g�Ώۂ̊֐������s
			creator_ptr_->Create(test_node, next_node_index_, &output_nodes);

			// �o�͂��ꂽ�m�[�h�̐����m�F
			std::string error_message = ToString(i) + "\n" +
				"�e�m�[�h" + test_node.ToString() + "\n_";

			EXPECT_EQ(output_nodes.size(), pow(all_discrete_leg_pos.size(), 3)) << error_message;
		}
	}

	TEST_F(LegHierarchyNodeCreatorTest, CreateTestNodeNumCheckCaseOfNoLegLifted)
	{
		// �S�Ă̋r���ڒn���Ă���ꍇ�C�o�͂����m�[�h�̐��̃e�X�g

		std::array<bool, HexapodConst::kLegNum> is_ground = { true, true, true, true, true, true };

		const RobotStateNode test_node = MakeTestNode(is_ground);

		std::vector<RobotStateNode> output_nodes;

		// �e�X�g�Ώۂ̊֐������s
		creator_ptr_->Create(test_node, next_node_index_, &output_nodes);

		// �o�͂��ꂽ�m�[�h�̐����m�F
		std::string error_message = ToString(is_ground) + "\n" +
			"�e�m�[�h" + test_node.ToString() + "\n_";

		EXPECT_EQ(output_nodes.size(), 1) << error_message;
	}

	TEST_F(LegHierarchyNodeCreatorTest, CreateTestParentCheckCaseOfOneLegLifted)
	{
		// 1�{�̋r���V�r���Ă���ꍇ�C�o�͂����m�[�h�̐e�����������ǂ����̃e�X�g

		for (const auto& i : oneleg_lifted_testcase)
		{
			const RobotStateNode test_node = MakeTestNode(i);

			std::vector<RobotStateNode> output_nodes;

			// �e�X�g�Ώۂ̊֐������s
			creator_ptr_->Create(test_node, next_node_index_, &output_nodes);

			for (const auto& j : output_nodes)
			{
				EXPECT_EQ(j.parent_num, next_node_index_) << "Create�֐��̈����Ŏw�肵��index�ɂȂ�K�v������܂��D";
				EXPECT_EQ(j.depth, test_node.depth + 1) << "�[����1�[���Ȃ�K�v������܂��D";
				EXPECT_EQ(j.next_move, next_move_) << "���̓��삪�w�肵�����̂ɂȂ��Ă���K�v������܂��D";
			}
		}
	}

	TEST_F(LegHierarchyNodeCreatorTest, CreateTestParentCheckCaseOfTwoLegLifted)
	{
		// 2�{�̋r���V�r���Ă���ꍇ�C�o�͂����m�[�h�̐e�����������ǂ����̃e�X�g

		for (const auto& i : twoleg_lifted_testcase)
		{
			const RobotStateNode test_node = MakeTestNode(i);

			std::vector<RobotStateNode> output_nodes;

			// �e�X�g�Ώۂ̊֐������s
			creator_ptr_->Create(test_node, next_node_index_, &output_nodes);

			for (const auto& j : output_nodes)
			{
				EXPECT_EQ(j.parent_num, next_node_index_) << "Create�֐��̈����Ŏw�肵��index�ɂȂ�K�v������܂��D";
				EXPECT_EQ(j.depth, test_node.depth + 1) << "�[����1�[���Ȃ�K�v������܂��D";
				EXPECT_EQ(j.next_move, next_move_) << "���̓��삪�w�肵�����̂ɂȂ��Ă���K�v������܂��D";
			}
		}
	}

	TEST_F(LegHierarchyNodeCreatorTest, CreateTestParentCheckCaseOfThreeLegLifted)
	{
		// 3�{�̋r���V�r���Ă���ꍇ�C�o�͂����m�[�h�̐e�����������ǂ����̃e�X�g

		for (const auto& i : threeleg_lifted_testcase)
		{
			const RobotStateNode test_node = MakeTestNode(i);

			std::vector<RobotStateNode> output_nodes;

			// �e�X�g�Ώۂ̊֐������s
			creator_ptr_->Create(test_node, next_node_index_, &output_nodes);

			for (const auto& j : output_nodes)
			{
				EXPECT_EQ(j.parent_num, next_node_index_) << "Create�֐��̈����Ŏw�肵��index�ɂȂ�K�v������܂��D";
				EXPECT_EQ(j.depth, test_node.depth + 1) << "�[����1�[���Ȃ�K�v������܂��D";
				EXPECT_EQ(j.next_move, next_move_) << "���̓��삪�w�肵�����̂ɂȂ��Ă���K�v������܂��D";
			}
		}
	}

	TEST_F(LegHierarchyNodeCreatorTest, CreateTestParentCheckCaseOfNoLegLifted)
	{
		// �S�Ă̋r���ڒn���Ă���ꍇ�C�o�͂����m�[�h�̐e�����������ǂ����̃e�X�g

		std::array<bool, HexapodConst::kLegNum> is_ground = { true, true, true, true, true, true };

		const RobotStateNode test_node = MakeTestNode(is_ground);

		std::vector<RobotStateNode> output_nodes;

		// �e�X�g�Ώۂ̊֐������s
		creator_ptr_->Create(test_node, next_node_index_, &output_nodes);

		for (const auto& j : output_nodes)
		{
			EXPECT_EQ(j.parent_num, next_node_index_) << "Create�֐��̈����Ŏw�肵��index�ɂȂ�K�v������܂��D";
			EXPECT_EQ(j.depth, test_node.depth + 1) << "�[����1�[���Ȃ�K�v������܂��D";
			EXPECT_EQ(j.next_move, next_move_) << "���̓��삪�w�肵�����̂ɂȂ��Ă���K�v������܂��D";
		}
	}

	TEST_F(LegHierarchyNodeCreatorTest, CreateTestOtherValueCheckCaseOfOneLegLifted)
	{
		// 1�{�̋r���V�r���Ă���ꍇ�C���̑��̕ύX����Ă͂����Ȃ��l���ύX����Ă��Ȃ����̃e�X�g

		for (const auto& i : oneleg_lifted_testcase)
		{
			const RobotStateNode test_node = MakeTestNode(i);

			std::vector<RobotStateNode> output_nodes;

			// �e�X�g�Ώۂ̊֐������s
			creator_ptr_->Create(test_node, next_node_index_, &output_nodes);

			for (const auto& j : output_nodes)
			{
				// �o�͂��ꂽ�m�[�h���m�F����D
				std::string error_message = ToString(i) + "\n" +
					"�������ꂽ�m�[�h" + j.ToString() + "\n_";

				EXPECT_EQ(j.leg_pos, test_node.leg_pos) << error_message << "�r�ʒu�͕ω����܂���D\n_";
				EXPECT_EQ(j.leg_reference_pos, test_node.leg_reference_pos) << error_message << "�r�̊�ʒu�͕ω����܂���D\n_";
				EXPECT_EQ(j.global_center_of_mass, test_node.global_center_of_mass) << error_message << "�d�S�ʒu�͕ω����܂���D\n_";
				EXPECT_EQ(j.rot, test_node.rot) << error_message << "�p���͕ω����܂���D\n_";

			}
		}
	}

	TEST_F(LegHierarchyNodeCreatorTest, CreateTestOtherValueCheckCaseOfTwoLegLifted)
	{
		// 2�{�̋r���V�r���Ă���ꍇ�C���̑��̕ύX����Ă͂����Ȃ��l���ύX����Ă��Ȃ����̃e�X�g

		for (const auto& i : twoleg_lifted_testcase)
		{
			const RobotStateNode test_node = MakeTestNode(i);

			std::vector<RobotStateNode> output_nodes;

			// �e�X�g�Ώۂ̊֐������s
			creator_ptr_->Create(test_node, next_node_index_, &output_nodes);

			for (const auto& j : output_nodes)
			{
				// �o�͂��ꂽ�m�[�h���m�F����D
				std::string error_message = ToString(i) + "\n" +
					"�������ꂽ�m�[�h" + j.ToString() + "\n_";

				EXPECT_EQ(j.leg_pos, test_node.leg_pos) << error_message << "�r�ʒu�͕ω����܂���D\n_";
				EXPECT_EQ(j.leg_reference_pos, test_node.leg_reference_pos) << error_message << "�r�̊�ʒu�͕ω����܂���D\n_";
				EXPECT_EQ(j.global_center_of_mass, test_node.global_center_of_mass) << error_message << "�d�S�ʒu�͕ω����܂���D\n_";
				EXPECT_EQ(j.rot, test_node.rot) << error_message << "�p���͕ω����܂���D\n_";

			}
		}
	}

	TEST_F(LegHierarchyNodeCreatorTest, CreateTestOtherValueCheckCaseOfThreeLegLifted)
	{
		// 3�{�̋r���V�r���Ă���ꍇ�C���̑��̕ύX����Ă͂����Ȃ��l���ύX����Ă��Ȃ����̃e�X�g

		for (const auto& i : threeleg_lifted_testcase)
		{
			const RobotStateNode test_node = MakeTestNode(i);

			std::vector<RobotStateNode> output_nodes;

			// �e�X�g�Ώۂ̊֐������s
			creator_ptr_->Create(test_node, next_node_index_, &output_nodes);

			for (const auto& j : output_nodes)
			{
				// �o�͂��ꂽ�m�[�h���m�F����D
				std::string error_message = ToString(i) + "\n" +
					"�������ꂽ�m�[�h" + j.ToString() + "\n_";

				EXPECT_EQ(j.leg_pos, test_node.leg_pos) << error_message << "�r�ʒu�͕ω����܂���D\n_";
				EXPECT_EQ(j.leg_reference_pos, test_node.leg_reference_pos) << error_message << "�r�̊�ʒu�͕ω����܂���D\n_";
				EXPECT_EQ(j.global_center_of_mass, test_node.global_center_of_mass) << error_message << "�d�S�ʒu�͕ω����܂���D\n_";
				EXPECT_EQ(j.rot, test_node.rot) << error_message << "�p���͕ω����܂���D\n_";

			}
		}
	}

	TEST_F(LegHierarchyNodeCreatorTest, CreateTestOtherValueCheckCaseOfNoLegLifted)
	{
		// �S�Ă̋r���ڒn���Ă���ꍇ�C���̑��̕ύX����Ă͂����Ȃ��l���ύX����Ă��Ȃ����̃e�X�g

		std::array<bool, HexapodConst::kLegNum> is_ground = { true, true, true, true, true, true };

		const RobotStateNode test_node = MakeTestNode(is_ground);

		std::vector<RobotStateNode> output_nodes;

		// �e�X�g�Ώۂ̊֐������s
		creator_ptr_->Create(test_node, next_node_index_, &output_nodes);

		for (const auto& j : output_nodes)
		{
			// �o�͂��ꂽ�m�[�h���m�F����D
			std::string error_message = ToString(is_ground) + "\n" +
				"�������ꂽ�m�[�h" + j.ToString() + "\n_";

			EXPECT_EQ(j.leg_pos, test_node.leg_pos) << error_message << "�r�ʒu�͕ω����܂���D\n_";
			EXPECT_EQ(j.leg_reference_pos, test_node.leg_reference_pos) << error_message << "�r�̊�ʒu�͕ω����܂���D\n_";
			EXPECT_EQ(j.global_center_of_mass, test_node.global_center_of_mass) << error_message << "�d�S�ʒu�͕ω����܂���D\n_";
			EXPECT_EQ(j.rot, test_node.rot) << error_message << "�p���͕ω����܂���D\n_";

		}
	}
}