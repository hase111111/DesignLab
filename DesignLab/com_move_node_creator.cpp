//#include "com_move_node_creator.h"
//
//#include <iostream>
//
//#include "com_candidate_polygon_maker.h"
//#include "com_selecter.h"
//#include "graph_search_const.h"
//#include "leg_state.h"
//
//
//// @todo ���󓮂��Ȃ��̂ŁC�����悤�ɂ���
//
//ComMoveNodeCreator::ComMoveNodeCreator(const MapState_Old* const p_map, const std::shared_ptr<const AbstractHexapodStateCalculator>& calc, const EHexapodMove next_move)
//	: INodeCreator(p_map, calc, next_move), mp_map(p_map)
//{
//	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "ComMoveNodeCreator : �R���X�g���N�^���Ă΂ꂽ�D\n"; }
//};
//
//
//ComMoveNodeCreator::~ComMoveNodeCreator()
//{
//	if (GraphSearchConst::DO_DEBUG_PRINT) { std::cout << "ComMoveNodeCreator : �f�X�g���N�^���Ă΂ꂽ�D\n"; }
//};
//
//
//void ComMoveNodeCreator::create(const SNode& current_node, const int current_num, std::vector<SNode>* output_graph)
//{
//	//std::vector<std::pair<designlab::SPolygon2, EDiscreteComPos>> candidate_polygons;
//
//	////�d�S�ړ���̌��n�_�͈̔͂��������p�`���쐬����
//	//ComCandidatePolygonMaker polygon_maker;
//	////polygon_maker.makeCandidatePolygon(current_node, candidate_polygons);
//
//	////���͈͂�����ۂɈړ������̍��W��I������
//	//ComSelecter com_selecter;
//	//com_selecter.setCurrentNode(current_node);
//
//	//for (const auto& i : candidate_polygons)
//	//{
//	//	designlab::Vector3 res_pos;
//
//	//	if (com_selecter.getComFromPolygon(i.first, i.second, res_pos) == true)
//	//	{
//	//		SNode next_node = current_node;
//
//	//		next_node.changeGlobalCenterOfMass(res_pos, false);					//�d�S�ʒu��ύX���C����ɔ����ڒn�r�̈ʒu���ύX����
//
//	//		dl_leg::changeComPattern(next_node.leg_state, i.second);		//leg_state��com_pattern��ύX����
//
//	//		for (int i = 0; i < HexapodConst::LEG_NUM; ++i) { dl_leg::changeLegStateKeepTopBit(next_node.leg_state, i, 4); }
//
//	//		next_node.changeNextNode(current_num, m_next_move);	//�[����e�m�[�h��ύX����
//
//	//		(*output_graph).push_back(next_node);
//	//	}
//	//}
//
//	//if (DO_DEBUG_PRINT)std::cout << "ComMoveNodeCreator::create() : " << (*output_graph).size() << std::endl;
//}