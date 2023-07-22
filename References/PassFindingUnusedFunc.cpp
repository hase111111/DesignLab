#include "PassFinding.h"
#include "function_walking_pattern_generator.h"
#include "vectorFunc.h"
#include "Define.h"
#include "LegState.h"

//PassFinding.cpp �̒����猻�ݎg�p����Ă��Ȃ��֐��𐮗����邽�߂ɂ܂Ƃ߂Ď����Ă�������
//2023/04/28���� PassFinding.cpp�̒����]�T��3000�s���炢����̂ŁC�����S���ǂ߂Ȃ�
//�v�����Ȃ��C�g�p���Ă��Ȃ��֐��͑S���������Ɏ����Ă��Đ������Ă������Ǝv��
//���g�p������ƌ����ĕs�K�v�ł͂Ȃ��C��y���̍��܂ł̌������ʂ̃\�[�X�R�[�h�̉�Ȃ̂�
//�����Ƃ��đ�ϋM�d�ł���Ƃ����邾�낤

//�m�[�h�̋r�ʒu����Ccomtype�̏d�S�ʒu���\�ȏꍇ1
bool PassFinding::isAbleLegPosComType(SNode* node)
{
	//myvector::SVector ret_4_GCOM[35];
	//int ret_4_COMType[35] = { 0 }, ret_4_transition_kaisou[35] = { 0 }, passnum = 0;

	//myvector::SVector legPosi[6];//�d�S�ړ���̋r�ʒu
	//int groundLegInd[6];		//�ڒn���Ă���r��1

	//for (int le = 0; le < 6; le++) {
	//	//groundLegInd[le] = HX2[node->v][le];
	//	groundLegInd[le] = LegState::isGrounded(node->leg_state, le);
	//}

	//myvector::SVector ret_Leg[6][20];
	//searchTransitionCoM(node, ret_Leg, ret_4_GCOM, ret_4_COMType, passnum);	//���ꂾ�߂ȋC������180823

	//for (int i = 0; i < passnum; i++) {
	//	//if(ret_4_COMType[i] == node->COM_type){
	//	if (ret_4_COMType[i] == numOfCOMType(node->leg_state)) {
	//		for (int le = 0; le < 6; le++) {
	//			legPosi[le] = myvector::subVec(node->Leg[le], ret_4_GCOM[i]);			//�d�S�ړ���̋r�ʒu
	//		}
	//		return 1;
	//	}
	//}
	////if(node->COM_type == 0){
	//if (numOfCOMType(node->leg_state) == 0) {
	//	return 1;
	//}
	//return 0;
	return false;
}

//�񎟕����ړ�(����)
void PassFinding::calculatePassLegRotation() {

	//int ret_2_transition_v[36][6]={};												//�֐��ɓn��2�������ړ���ۑ�����z��@36�ʂ肠��@6�͓K���ɂ��߂� ret_2_transition_v[][0]�͊i�[����Ă��郋�[�g�̒���
	//int LegGroundablePointNum[HexapodConst::LEG_NUM][3]={};										//�r�̑O��̋r�ڒn�\�_ LegGroundablePointNum[HexapodConst::LEG_NUM][0] ���P�̈ʒu��\���B
	//myvector::VECTOR LegGroundablePoint[HexapodConst::LEG_NUM][3][100]={};									//��������r�ʒu1�C2�C3�Ɉʒu����_�Q�̒��̑�\����1�_
	//int i_F[6]={};																	//i_F[6]�͑������邩�ǂ���
	//int passnum_2zi;															//�֐�����A���Ă���2�������ړ��̐�
	//for(int to = LastNodeNum; to >= 0 ;to--){
	//	if(route[to].debug == 22) continue;	//���ł�2��ȏ�A���ŋr���㉺�^�������Ă���ꍇ
	//	if(route[to].node_height <= 0)continue;
	//	if(route[to].node_height < 2){
	//		//std::cout<<"calculatePassLegRotation\n";
	//		SelectLegPoint_Rotation(&route[to], LegGroundablePointNum, LegGroundablePoint);									//�r�ʒu�Z�o//�r�ڈړ��_������

	//		////////////////�グ���ςȂ��̋r�@�̏�r//////////////////////////
	//		//LegGroundablePointNum[4][0] = 0;//4�ԋr�͉��낹�Ȃ�
	//		//LegGroundablePointNum[4][1] = 0;//4�ԋr�͉��낹�Ȃ�
	//		//LegGroundablePointNum[4][2] = 0;//4�ԋr�͉��낹�Ȃ�
	//		//////////////////////////////////////////////////////////////////

	//		////////////////�r�ڒn�_�����@�����p�@�r�ʒu�P��F�߂Ȃ�//////////
	//		//LegGroundablePointNum[0][0] = 0;
	//		//LegGroundablePointNum[1][0] = 0;
	//		//LegGroundablePointNum[2][0] = 0;
	//		//LegGroundablePointNum[3][0] = 0;
	//		//LegGroundablePointNum[4][0] = 0;
	//		//LegGroundablePointNum[5][0] = 0;
	//		//////////////////////////////////////////////////////////////////

	//		bool visited[36] = { 0 };	//�T���ς݂������͒T���ł��Ȃ��m�[�h��1�ɂȂ�
	//		getGraph(route[to].leg_state, i_F, LegGroundablePointNum, visited);	//�}�b�v�̃f�[�^����O���t�𓾂�//�J�ڕs�\�ȃm�[�h��visited[i]��1�ɂȂ�

	//		for(int ileg = 0; ileg < 6; ileg ++){
	//			//if (HX2[route[to].v][ileg] == 1 && i_F[ileg] == 0) {//�ڒn�r�̋r�ڒn�_����0
	//			if (isGrounded(route[to].leg_state, ileg) && i_F[ileg] == 0) {//�ڒn�r�̋r�ڒn�_����0
	//				route[to].node_height = -1;
	//				break;
	//			}
	//		}
	//		//std::cout<<"route[to].v = "<<route[to].v<<"\n";
	//		pass_transitions_2zi(route[to].leg_state, visited, ret_2_transition_v, &passnum_2zi);	//i_F[6]�͑������邩�ǂ����@ret_2_transition_v[36][6]�Ɍ���//�K�w���ŒZ�o�H�T��
	//		//passnum_2zi�͈ړ��\�ȃm�[�h�̐�
	//		//ret_2_transition_v[a][b],a�͈ړ��\�ȃm�[�h�̃m�[�h�ԍ��̏�������,b�̓��[�gb=0���[�g����b=1���̃m�[�hb=2�ЂƂ�̃m�[�hb=3���̃m�[�h�c
	//																											
	//		for(int ii = 0; ii < passnum_2zi; ii++){
	//			//std::cout<<"���[�g����="<<ret_2_transition_v[ii][0]<<std::endl;
	//			//std::string stop; std::cin>>stop;
	//			for(int iii=0; iii<1/*ret_2_transition_v[ii][0]*/; iii++){			//�i�[����Ă��郋�[�g�̒��������J��Ԃ�
	//				(LastNodeNum)++;
	//				if(LastNodeNum >= m_route_max){std::cout<<"Error Overflow LastNodeNum = "<<LastNodeNum;std::string r;std::cin>>r;}			//�I�[�o�[�t���[����O�ɋً}��~
	//				//route[LastNodeNum].v = ret_2_transition_v[ii][iii+2];					//v�̒l���X�V
	//				route[LastNodeNum].leg_state = ret_2_transition_v[ii][iii+2];					//v�̒l���X�V
	//				//route[LastNodeNum].kaisou = route[to].kaisou;							//�K�w�͕ς��Ȃ�
	//				route[LastNodeNum].center_of_mass = route[to].center_of_mass;									//�d�S�ʒu�͐e�m�[�h�Ɠ���
	//				route[LastNodeNum].global_center_of_mass = route[to].global_center_of_mass;								//�d�S�ʒu�͐e�m�[�h�Ɠ���
	//				//route[LastNodeNum].COM_type = route[to].COM_type;						//�d�S�ʒu�^�C�v�͐e�m�[�h�Ɠ���
	//				route[LastNodeNum].pitch = route[to].pitch;//�����Ő���͍s��Ȃ�
	//				route[LastNodeNum].roll = route[to].roll;
	//				route[LastNodeNum].yaw = route[to].yaw;
	//				route[LastNodeNum].debug = route[to].debug / 10 + 20;
	//				int legPosiInd[6] = {0};												//LegGroundablePoint[HexapodConst::LEG_NUM][3][100]�@�Ɋi�[����Ă���r�ʒu�̃C���f�b�N�X[100]��{0�����s�\�ȋr�ʒu�̏ꍇ�͏���++1
	//				myvector::VECTOR myCom;
	//				//std::cout<<"v="<<route[LastNodeNum].v<<std::endl;
	//				//					for(;;){
	//				for(int iLeg = 0; iLeg < 6; iLeg++){
	//					int legv = isGrounded(route[LastNodeNum].leg_state, iLeg);
	//					int legkaisou = numOfLegPosi(route[LastNodeNum].leg_state, iLeg);
	//					//std::cout<<"route[LastNodeNum].v = "<<route[LastNodeNum].v<<"\n";
	//					if(legv == 0 ){								//0�͗V�r
	//						route[LastNodeNum].Leg[iLeg] = route[to].Leg[iLeg];
	//						route[LastNodeNum].Leg[iLeg].z = -route[LastNodeNum].global_center_of_mass.z + LegHeight;
	//						//if(route[LastNodeNum].Leg[iLeg].z == -100){
	//						//	std::cout<<"ERROR1 route[LastNodeNum].Leg["<<iLeg<<"].z != -100\n";
	//						//std::cout<<"route[LastNodeNum].Leg["<<iLeg<<"].z = "<<route[LastNodeNum].Leg[iLeg].z<<"\n";
	//						//std::string stop;
	//						//std::cin>>stop;
	//						//}

	//					}else if(legkaisou == 2){					//�r�ʒu��2�Ȃ�Leg2����
	//						route[LastNodeNum].Leg[iLeg] = route[to].Leg2[iLeg];					//�r�ʒu���X�V
	//						//route[LastNodeNum].Leg[iLeg] = LegGroundablePoint[iLeg][HX4[route[LastNodeNum].kaisou][iLeg] - 1][legPosiInd[iLeg]];
	//						route[LastNodeNum].Leg[iLeg] = LegGroundablePoint[iLeg][legkaisou - 1][legPosiInd[iLeg]];
	//						//if(LegGroundablePointNum[iLeg][HX4[route[LastNodeNum].kaisou][iLeg] - 1]==0)std::cout<<"����!!\n";
	//						if(LegGroundablePointNum[iLeg][legkaisou - 1]==0)std::cout<<"����!!\n";
	//							if(route[LastNodeNum].Leg[iLeg].z != -110){
	//								std::cout<<"ERROR2 route[LastNodeNum].Leg["<<iLeg<<"].z != -100\n";
	//								std::cout<<"route[LastNodeNum].Leg["<<iLeg<<"].z = "<<route[LastNodeNum].Leg[iLeg].z<<"\n";
	//								std::string stop;
	//								std::cin>>stop;
	//						}

	//					}else{
	//						//route[LastNodeNum].Leg[iLeg] = LegGroundablePoint[iLeg][HX4[route[LastNodeNum].kaisou][iLeg] - 1][legPosiInd[iLeg]];//iLeg�Ԗڂ̋r�ʒu�iVECTOR�j����
	//						//if(LegGroundablePointNum[iLeg][HX4[route[LastNodeNum].kaisou][iLeg] - 1]==0)std::cout<<"����!!\n";
	//						route[LastNodeNum].Leg[iLeg] = LegGroundablePoint[iLeg][legkaisou - 1][legPosiInd[iLeg]];//iLeg�Ԗڂ̋r�ʒu�iVECTOR�j����
	//						if (LegGroundablePointNum[iLeg][legkaisou - 1] == 0)std::cout << "����!!\n";
	//							if(route[LastNodeNum].Leg[iLeg].z != -110){
	//								std::cout<<"ERROR3 route[LastNodeNum].Leg["<<iLeg<<"].z != -100\n";
	//								std::cout<<"route[LastNodeNum].Leg["<<iLeg<<"].z = "<<route[LastNodeNum].Leg[iLeg].z<<"\n";
	//								std::string stop;
	//								std::cin>>stop;
	//						}
	//					}
	//					route[LastNodeNum].Leg2[iLeg] = route[to].Leg2[iLeg];						//�r�ʒu�Q�͕ύX���Ȃ�

	//				}//�r�ʒu����I��

	//				 //						if(isAbleLegPosComType(&route[LastNodeNum],myCom) )break;						//�\�ȑ̐��Ȃ甲����

	//				 //					}
	//				if(iii == 0){
	//					route[LastNodeNum].parent = &route[to];								//�e�m�[�h�Ǝq�m�[�h��ڑ�

	//					if(route[LastNodeNum].parent->node_height <= 0){
	//						route[LastNodeNum].node_height = route[LastNodeNum].parent->node_height - 1;
	//						route[LastNodeNum].debug = -1;
	//						std::cout<<"�eNoodeHeight_Error"<<std::endl;std::string stop;std::cin>>stop;
	//					}else{
	//						route[LastNodeNum].node_height = 1;								//�Ƃ肠�����X�V��
	//					}

	//				}else{
	//					route[LastNodeNum].parent = &route[LastNodeNum - 1];				//�e�m�[�h�Ǝq�m�[�h��ڑ�

	//					if(route[LastNodeNum].parent->node_height <= 0){
	//						route[LastNodeNum].node_height = route[LastNodeNum].parent->node_height - 1;
	//						route[LastNodeNum].debug = -1;
	//						std::cout<<"�eNoodeHeight_Error"<<std::endl;std::string stop;std::cin>>stop;
	//					}else{
	//						route[LastNodeNum].node_height = 1;								//�Ƃ肠�����X�V��
	//						route[LastNodeNum - 1].node_height = 3;								//2���ړ��̓r���̃m�[�h�Ȃ̂ōX�V�s��
	//					}

	//				}

	//				////////////////////////////////////////////////////////

	//				////������C_C_C���Ăԁ@�r�ʒu�n���ďd�S�^�C�v���邩�ǂ������f�@route[LastNodeNum].COM_type
	//				////����������r�ʒu�ς���@�ǂ�����āH�@1�{�����r�ʒu��ύX���Ă��Ȃ��Ȃ�y
	//				////�܂��͏d�S�^�C�v�Ȃ�������m�[�h������
	//				//myvector::VECTOR myComPosi, beforeLeg2Posi[6];
	//				//if( isAbleLegPosComType(&route[LastNodeNum], &myComPosi) && route[LastNodeNum].parent->node_height > 0){
	//				//	//route[LastNodeNum].global_center_of_mass = myvector::addVec(myComPosi, route[LastNodeNum].global_center_of_mass);
	//				//	//for(int le = 0;le < 6; le++){
	//				//	//	route[LastNodeNum].Leg[le] = myvector::subVec(route[LastNodeNum].Leg[le], myComPosi) ;			//�r�ʒu���X�V
	//				//	//	//beforeLeg2Posi[le] = route[LastNodeNum].Leg2[le];
	//				//	//	route[LastNodeNum].Leg2[le] = myvector::subVec(route[LastNodeNum].Leg2[le], myComPosi) ;			//�r�ʒu���X�V
	//				//	//	//route[LastNodeNum].Leg2[le].z = beforeLeg2Posi[le].z;
	//				//	//}
	//				//	route[LastNodeNum].debug = 20;
	//				//}else if( !isAbleLegPosComType(&route[LastNodeNum], &myComPosi) && route[LastNodeNum].parent->node_height > 0){
	//				//	//std::cout<<"/////////////////////////////////////////////////////";
	//				//	route[LastNodeNum].node_height = route[LastNodeNum].node_height -100;		//����ȍ~�̍X�V�s��
	//				//	route[LastNodeNum].debug = -1;
	//				//	
	//				//}else{
	//				//	route[LastNodeNum].node_height = route[LastNodeNum].node_height - 1000;		//����ȍ~�̍X�V�s��
	//				//	route[LastNodeNum].debug = -1;

	//				//}
	//				////////////////////////////////////////////////////////

	//			}
	//		}
	//		route[to].node_height ++;																//�e�m�[�h�̃^�C�v���v���X1
	//	}
	//	//std::cout<<to<<",,"<<route[to].node_height<<std::endl;
	//	//std::string stop;std::cin>>stop;
	//}

}

//���̉�]�̒T��
void PassFinding::pass_body_rotation(SNode* node, double thPRY[3][20], myvector::SVector ret_leg_move[HexapodConst::LEG_NUM][20], int* Rotation_passnum)
{
	//int i, Rotation_num;
	//int groundLeg[6];
	//for (i = 0; i < HexapodConst::LEG_NUM; i++) {
	//	//groundLeg[i] = HX2[node->v][i];	//�ڒn���V�r��Ԃ����R�s�[
	//	groundLeg[i] = LegState::isGrounded(node->leg_state, i);	//�ڒn���V�r��Ԃ����R�s�[
	//}
	//_SearchPossibleBodyRotation.phantomX.setMyPosition(node->global_center_of_mass);					//�d�S�ʒu
	////_SearchPossibleBodyRotation.phantomX.setMyPosition(myvector::VGet(0,0,100));	
	//_SearchPossibleBodyRotation.phantomX.setTarget(m_target);	//�i�s����
	//_SearchPossibleBodyRotation.phantomX.setMyDirection(node->pitch, node->roll, node->yaw);	//�p��
	//_SearchPossibleBodyRotation.phantomX.setLocalLeg2Pos(node->Leg2);					//�r�ʒuz=�ڒn
	//_SearchPossibleBodyRotation.phantomX.setLocalLegPos(node->Leg);					//�r�ʒu
	//myvector::SVector Rotation_Center = node->global_center_of_mass;	//��]���S
	////double leg_th[6] = {45,0,-45,-135,180,135};

	//Rotation_num = _SearchPossibleBodyRotation.pass_body_rotation(groundLeg);	//���̉�]�̒T��
	//for (int i = 0; i < Rotation_num; i++) {		//�T���������̉�]�ʂƃ��{�b�g���W�ł̋r�ʒu���L�^
	//	for (int ii = 0; ii < HexapodConst::LEG_NUM; ii++) {
	//		ret_leg_move[ii][i] = _SearchPossibleBodyRotation.Leg_move[ii][i];
	//	}
	//	thPRY[0][i] = _SearchPossibleBodyRotation.C_thPRY[0][i];
	//	thPRY[1][i] = _SearchPossibleBodyRotation.C_thPRY[1][i];
	//	thPRY[2][i] = _SearchPossibleBodyRotation.C_thPRY[2][i];
	//}
	//*Rotation_passnum = Rotation_num;
}
