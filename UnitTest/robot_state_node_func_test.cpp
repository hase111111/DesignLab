#include "pch.h"

#include "../DesignLab/robot_state_node.h"


namespace dl = ::designlab;
namespace dllf = ::designlab::leg_func;


namespace designlab::test::node
{
	TEST(RobotStateNodeTest, ChangeParentNodeTest)
	{
		const RobotStateNode node(
			dllf::MakeLegStateBit(DiscreteComPos::kCenterFront, { true, true, false, true, false, true }, {}),
			dl::make_array<dl::Vector3>(dl::Vector3{ 10,10,10 }, dl::Vector3{ 10,10,10 }, dl::Vector3{ 10,10,10 }, dl::Vector3{ 10,10,10 }, dl::Vector3{ 10,10,10 }, dl::Vector3{ 10,10,10 }),
			dl::make_array<dl::Vector3>(dl::Vector3{ 10,10,10 }, dl::Vector3{ 10,10,10 }, dl::Vector3{ 10,10,10 }, dl::Vector3{ 10,10,10 }, dl::Vector3{ 10,10,10 }, dl::Vector3{ 10,10,10 }),
			dl::Vector3{ 300, 50, -10 },
			dl::EulerXYZ{ 0.1f, 1.2f, 3.14f },
			HexapodMove::kLegHierarchyChange,
			315,
			5
		);

		EXPECT_EQ(node.parent_num, 315);
		EXPECT_EQ(node.depth, 5);

		RobotStateNode changeed = node;

		changeed.ChangeParentNode();

		EXPECT_EQ(changeed.parent_num, -1) << "�e�����Ȃ��Ȃ�΁Cindex��-1�ł��D";
		EXPECT_EQ(changeed.depth, 0) << "�e�m�[�h�̐[����0�ł��D";

		const std::string error_mes = "���̑��̒l�͕ύX����܂���D";
		EXPECT_EQ(node.leg_state, changeed.leg_state) << error_mes;
		EXPECT_EQ(node.leg_pos, changeed.leg_pos) << error_mes;
		EXPECT_EQ(node.leg_reference_pos, changeed.leg_reference_pos) << error_mes;
		EXPECT_EQ(node.global_center_of_mass, changeed.global_center_of_mass) << error_mes;
		EXPECT_EQ(node.rot, changeed.rot) << error_mes;
		EXPECT_EQ(node.next_move, changeed.next_move) << error_mes;
	}

	TEST(RobotStateNodeTest, ChangeToNextNodeTest)
	{
		const RobotStateNode node(
			dllf::MakeLegStateBit(DiscreteComPos::kCenterFront, { true, true, false, true, false, true }, {}),
			dl::make_array<dl::Vector3>(dl::Vector3{ 10,10,10 }, dl::Vector3{ 10,10,10 }, dl::Vector3{ 10,10,10 }, dl::Vector3{ 10,10,10 }, dl::Vector3{ 10,10,10 }, dl::Vector3{ 10,10,10 }),
			dl::make_array<dl::Vector3>(dl::Vector3{ 10,10,10 }, dl::Vector3{ 10,10,10 }, dl::Vector3{ 10,10,10 }, dl::Vector3{ 10,10,10 }, dl::Vector3{ 10,10,10 }, dl::Vector3{ 10,10,10 }),
			dl::Vector3{ 300, 50, -10 },
			dl::EulerXYZ{ 0.1f, 1.2f, 3.14f },
			HexapodMove::kLegHierarchyChange,
			315,
			5
		);

		EXPECT_EQ(node.next_move, HexapodMove::kLegHierarchyChange);
		EXPECT_EQ(node.parent_num, 315);
		EXPECT_EQ(node.depth, 5);

		RobotStateNode changeed = node;
		const int parent_index = 1000;
		const HexapodMove next_move = HexapodMove::kComMove;

		changeed.ChangeToNextNode(parent_index, next_move);

		EXPECT_EQ(changeed.next_move, next_move) << "���̓���͕ύX����܂���D";
		EXPECT_EQ(changeed.parent_num, parent_index) << "�e�͕ύX����܂���D";
		EXPECT_EQ(changeed.depth, node.depth + 1) << "�[���͕ύX����܂���D";

		const std::string error_mes = "���̑��̒l�͕ύX����܂��D";
		EXPECT_EQ(node.leg_state, changeed.leg_state) << error_mes;
		EXPECT_EQ(node.leg_pos, changeed.leg_pos) << error_mes;
		EXPECT_EQ(node.leg_reference_pos, changeed.leg_reference_pos) << error_mes;
		EXPECT_EQ(node.global_center_of_mass, changeed.global_center_of_mass) << error_mes;
		EXPECT_EQ(node.rot, changeed.rot) << error_mes;
	}
}