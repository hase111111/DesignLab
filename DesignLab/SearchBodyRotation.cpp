#include "SearchBodyRotation.h"
#define _USE_MATH_DEFINES


SearchPossibleBodyRotation::SearchPossibleBodyRotation(void)
{
}


SearchPossibleBodyRotation::~SearchPossibleBodyRotation(void)
{
}

int SearchPossibleBodyRotation::pass_body_rotation(int groundLeg[6]){//ヨー軸の胴体回転探索
	int DIV = 20;
	double kaitennhankei=0.0;	//重心経路の回転半径
	double step;//刻み幅
	double Target_thY = 0.0;
	double thP,thR,thY;
	int i, ii, nnn = 0;
	myvector::SVector c_leg[6],c_leg2[6],r_leg[6],r_leg2[6],map[6];
	for(i = 0; i < LEGCOUNT; i++){//現在の脚位置
		c_leg[i] = r_leg[i] = phantomX.showLocalLegPosition(i);
		c_leg2[i] = r_leg2[i] = phantomX.showLocalPosition_of_2(i);
		map[i] = phantomX.showGlobalLegPosition(i);
	}
	myvector::SVector Rotation_Center=phantomX.showGlobalMyPosition();	//旋回中心=重心
	//現在の角度をコピー
	thP=phantomX.showGlobalMyDirectionthP();
	thR=phantomX.showGlobalMyDirectionthR();
	thY=phantomX.showGlobalMyDirectionthY();

	//double leg_th[6] = {45,0,-45,-135,180,135};//脚の基準角度[deg]
	//目標角度
	switch(phantomX.showTargetType())
	{
	case ETargetMode::STRAIGHT_VECOTR:
	case ETargetMode::STRAIGHT_POSITION:
		Target_thY = phantomX.showGlobalMyDirectionthY();//目標角度
		break;

	case ETargetMode::TURN_ON_SPOT_DIRECTION:
	case ETargetMode::TURN_DIRECTION:
	/*case 7:*/
		Target_thY = phantomX.showTurningDirection().z * 100;//方向×100
		break;

	case ETargetMode::TURN_ON_SPOT_ANGLE:
		Target_thY = phantomX.showTurningAngle().z;//目標角度
		break;
	
	case ETargetMode::TURN_ANGLE:
	/*case 8:*/
		myvector::SVector Vtemp;
		Vtemp = myvector::VSub(Rotation_Center, phantomX.showTurningCenter());

		if(myvector::V2Mag(Vtemp) <= phantomX.showTurningRadius())
		{
			Target_thY = atan2(Vtemp.y, Vtemp.x) + M_PI/2.0*phantomX.showTurningDirection().z - M_PI/2.0;//円の中心からの法線方向
		}
		else
		{//この部分は修正してないので間違っている
			myvector::SVector a;
			Vtemp.x = Rotation_Center.x - phantomX.showTurningCenter().x;
			Vtemp.y = Rotation_Center.y - phantomX.showTurningCenter().y;
			if(phantomX.showTurningDirection().z > 0){
				//左回転方向接点
				a.x = (Vtemp.x * phantomX.showTurningRadius() - Vtemp.y * sqrt(pow(Vtemp.x,2.0) + pow(Vtemp.y,2.0) - pow(phantomX.showTurningRadius(),2.0))) * phantomX.showTurningRadius() / (pow(Vtemp.x,2.0) + pow(Vtemp.y,2.0)) + phantomX.showTurningCenter().x;
				a.y = (Vtemp.y * phantomX.showTurningRadius() + Vtemp.x * sqrt(pow(Vtemp.x,2.0) + pow(Vtemp.y,2.0) - pow(phantomX.showTurningRadius(),2.0))) * phantomX.showTurningRadius() / (pow(Vtemp.x,2.0) + pow(Vtemp.y,2.0)) + phantomX.showTurningCenter().y;
				Target_thY = atan2(a.y - Rotation_Center.y, a.x -Rotation_Center.x);
			}else if(phantomX.showTurningDirection().z < 0){
				//右回転方向接点
				a.x = (Vtemp.x * phantomX.showTurningRadius() + Vtemp.y * sqrt(pow(Vtemp.x,2.0) + pow(Vtemp.y,2.0) - pow(phantomX.showTurningRadius(),2.0))) * phantomX.showTurningRadius() / (pow(Vtemp.x,2.0) + pow(Vtemp.y,2.0)) + phantomX.showTurningCenter().x;
				a.y = (Vtemp.y * phantomX.showTurningRadius() - Vtemp.x * sqrt(pow(Vtemp.x,2.0) + pow(Vtemp.y,2.0) - pow(phantomX.showTurningRadius(),2.0))) * phantomX.showTurningRadius() / (pow(Vtemp.x,2.0) + pow(Vtemp.y,2.0)) + phantomX.showTurningCenter().y;
				Target_thY = atan2(a.y - Rotation_Center.y, a.x -Rotation_Center.x);
			}
		}
		if(thY - M_PI > Target_thY) Target_thY += M_PI * 2.0;
		break;
	default:
		break;
	}

	//回転方向,回転角度単位(0.1は適当)
	if(thY < Target_thY) step = 0.1;
	else if(thY > Target_thY) step = -0.1;

	for(ii = 0; ii < LEGCOUNT; ii++)Leg_move[ii][0] = myvector::VGet(0.0,0.0,0.0);//旋回によるロボットから見た脚の移動量

	for(i = 0; i < 20; i++){//step=0.05なら1.0[rad]≒57.3[deg]まで
		for(ii = 0; ii < LEGCOUNT; ii++){
			if(phantomX.check_touchdown_point(ii, VCangeBodyToLeg(r_leg[ii],0.0,0.0,0.0))/* || groundLeg[ii] == 0*/){//胴体が回転しても接地脚が可動範囲内にあるかどうか
				//if(Target_thY>0.5){
				//std::cout<<"Leg="<<ii+1<<",thY="<<phantomX.showGlobalMyDirectionthY()<<std::endl;
				//myvector::VectorOutPut(phantomX.showGlobalCoxaJointPosi(ii));
				//myvector::VectorOutPut(phantomX.showPosition_of_2(ii));
				//myvector::VectorOutPut(r_leg[ii]);
				//double dth;
				//double leg_th[6] = {45,0,-45,-135,-180,135};
				//dth = atan2(r_leg[ii].y,r_leg[ii].x);
				//dth = dth * 180.0 / M_PI - leg_th[ii];
				//std::cout<<"dth="<<dth<<std::endl;
				//}
				//std::string stop;std::cin>>stop;
			}else{
				//std::cout<<phantomX.check_touchdown_point(ii, VCangeBodyToLeg(r_leg2[ii],0.0,0.0,0.0))<<"失敗"<<std::endl;
				//std::cout<<"Leg="<<ii+1<<",thY="<<phantomX.showGlobalMyDirectionthY()<<std::endl;
				//myvector::VectorOutPut(phantomX.showGlobalCoxaJointPosi(ii));
				//myvector::VectorOutPut(phantomX.showPosition_of_2(ii));
				//myvector::VectorOutPut(VCangeBodyToLeg(r_leg2[ii],0.0,0.0,0.0));
				//myvector::VectorOutPut(r_leg[ii]);
				//std::string stop;std::cin>>stop;
				break;
			}
		}
		if(ii != LEGCOUNT) {
			//if((thY > Target_thY && step > 0) || (thY < Target_thY && step < 0)){//目標角度を超えた角度も探索したとき
			//	//角度が目標値で探索
			//	i--;
			//	thY = Target_thY;
			//	phantomX.setMyDirection(thP,thR,thY);
			//	for(ii = 0; ii < LEGCOUNT; ii++){
			//		if(groundLeg[ii] == 1){//接地脚
			//			Leg_move[ii][i+1] = myvector::VSub(myvector::VRot( myvector::VSub(map[ii], phantomX.showGlobalMyPosition()), thP, thR, thY), phantomX.showLocalCoxaJointPosi(ii));
			//			r_leg2[ii] = Leg_move[ii][i+1];
			//			r_leg2[ii].z = -110;
			//			Leg_move[ii][i+1].z = c_leg[ii].z;
			//			r_leg[ii] = Leg_move[ii][i+1];
			//		}else{//遊脚はロボット座標系は変わらない（胴体と一緒に回転）
			//			Leg_move[ii][i+1] = phantomX.showLocalLegPosition(ii);
			//			r_leg[ii] = phantomX.showLocalLegPosition(ii);
			//			r_leg2[ii] = phantomX.showLocalPosition_of_2(ii);
			//		}
			//	}
			//	phantomX.setPosition_of_2(r_leg2);//脚位置更新
			//	phantomX.setMyLegPosition(r_leg);
			//}
			///*std::cout<<step*i<<"deg回転失敗"<<std::endl;*/
			//else break;
			if((int)(phantomX.showTargetType())>=3 && nnn == 0)
			{
				if(step > 0)
				{
					//反対方向側も0.1radだけ探索
					i--;
					thY = C_thPRY[2][0] - 0.1;
					nnn++;
					phantomX.setMyDirection(thP,thR,thY);
					for(ii = 0; ii < LEGCOUNT; ii++){
						if(groundLeg[ii] == 1){//接地脚
							Leg_move[ii][i+1] = myvector::VSub(myvector::VRot( myvector::VSub(map[ii], phantomX.showGlobalMyPosition()), thP, thR, thY), phantomX.showLocalCoxaJointPosi(ii));
							r_leg2[ii] = Leg_move[ii][i+1];
							r_leg2[ii].z = -110;
							Leg_move[ii][i+1].z = c_leg[ii].z;
							r_leg[ii] = Leg_move[ii][i+1];
						}else{//遊脚はロボット座標系は変わらない（胴体と一緒に回転）
							Leg_move[ii][i+1] = phantomX.showLocalLegPosition(ii);
							r_leg[ii] = phantomX.showLocalLegPosition(ii);
							r_leg2[ii] = phantomX.showLocalPosition_of_2(ii);
						}
					}
					phantomX.setPosition_of_2(r_leg2);//脚位置更新
					phantomX.setMyLegPosition(r_leg);
				}
				else if(step < 0){//反対方向側も0.1radだけ探索
					i--;
					thY = C_thPRY[2][0] + 0.1;
					nnn++;
					phantomX.setMyDirection(thP,thR,thY);
					for(ii = 0; ii < LEGCOUNT; ii++){
						if(groundLeg[ii] == 1){//接地脚
							Leg_move[ii][i+1] = myvector::VSub(myvector::VRot( myvector::VSub(map[ii], phantomX.showGlobalMyPosition()), thP, thR, thY), phantomX.showLocalCoxaJointPosi(ii));
							r_leg2[ii] = Leg_move[ii][i+1];
							r_leg2[ii].z = -110;
							Leg_move[ii][i+1].z = c_leg[ii].z;
							r_leg[ii] = Leg_move[ii][i+1];
						}else{//遊脚はロボット座標系は変わらない（胴体と一緒に回転）
							Leg_move[ii][i+1] = phantomX.showLocalLegPosition(ii);
							r_leg[ii] = phantomX.showLocalLegPosition(ii);
							r_leg2[ii] = phantomX.showLocalPosition_of_2(ii);
						}
					}
					phantomX.setPosition_of_2(r_leg2);//脚位置更新
					phantomX.setMyLegPosition(r_leg);
				}
				//if(step > 0 && C_thPRY[2][0] - 0.1 > Target_thY - 0.2 && nnn == 0){//目標角度+0.2を超えたとき
				//	i--;
				//	thY = C_thPRY[2][0] - 0.1;
				//	step = -0.1;
				//	nnn++;
				//	phantomX.setMyDirection(thP,thR,thY);
				//	for(ii = 0; ii < LEGCOUNT; ii++){
				//		if(groundLeg[ii] == 1){//接地脚
				//			Leg_move[ii][i+1] = myvector::VSub(myvector::VRot( myvector::VSub(map[ii], phantomX.showGlobalMyPosition()), thP, thR, thY), phantomX.showLocalCoxaJointPosi(ii));
				//			r_leg2[ii] = Leg_move[ii][i+1];
				//			r_leg2[ii].z = -110;
				//			Leg_move[ii][i+1].z = c_leg[ii].z;
				//			r_leg[ii] = Leg_move[ii][i+1];
				//		}else{//遊脚はロボット座標系は変わらない（胴体と一緒に回転）
				//			Leg_move[ii][i+1] = phantomX.showLocalLegPosition(ii);
				//			r_leg[ii] = phantomX.showLocalLegPosition(ii);
				//			r_leg2[ii] = phantomX.showLocalPosition_of_2(ii);
				//		}
				//	}
				//	phantomX.setPosition_of_2(r_leg2);//脚位置更新
				//	phantomX.setMyLegPosition(r_leg);
				//}
				//else if(step < 0 && C_thPRY[2][0] + 0.1 < Target_thY + 0.2 && nnn == 0){//目標角度-0.2より小さいとき
				//	i--;
				//	thY = C_thPRY[2][0] + 0.1;
				//	step = 0.1;
				//	nnn++;
				//	phantomX.setMyDirection(thP,thR,thY);
				//	for(ii = 0; ii < LEGCOUNT; ii++){
				//		if(groundLeg[ii] == 1){//接地脚
				//			Leg_move[ii][i+1] = myvector::VSub(myvector::VRot( myvector::VSub(map[ii], phantomX.showGlobalMyPosition()), thP, thR, thY), phantomX.showLocalCoxaJointPosi(ii));
				//			r_leg2[ii] = Leg_move[ii][i+1];
				//			r_leg2[ii].z = -110;
				//			Leg_move[ii][i+1].z = c_leg[ii].z;
				//			r_leg[ii] = Leg_move[ii][i+1];
				//		}else{//遊脚はロボット座標系は変わらない（胴体と一緒に回転）
				//			Leg_move[ii][i+1] = phantomX.showLocalLegPosition(ii);
				//			r_leg[ii] = phantomX.showLocalLegPosition(ii);
				//			r_leg2[ii] = phantomX.showLocalPosition_of_2(ii);
				//		}
				//	}
				//	phantomX.setPosition_of_2(r_leg2);//脚位置更新
				//	phantomX.setMyLegPosition(r_leg);
				//}
				else break;
				
			}
			else break;
		}
		else{
			C_thPRY[0][i] = thP;//実行可能なら角度保存
			C_thPRY[1][i] = thR;
			C_thPRY[2][i] = thY;
			//if(thY == Target_thY){
			//	//std::cout<<"目標角度\n";
			//	i++;
			//	break;
			//}
			thY+=step;//角度更新
			//if(phantomX.showTargetType()>=5){
			//	if(thY > Target_thY + 0.2 && step > 0 && C_thPRY[2][0] - 0.1 > Target_thY - 0.2 && nnn == 0){//目標角度+0.2を超えたとき
			//		thY = C_thPRY[2][0] - 0.1;
			//		step = -0.1;
			//		nnn++;
			//	}
			//	if(thY < Target_thY - 0.2 && step < 0 && C_thPRY[2][0] + 0.1 < Target_thY + 0.2 && nnn == 0){//目標角度-0.2より小さいとき
			//		thY = C_thPRY[2][0] + 0.1;
			//		step = 0.1;
			//		nnn++;
			//	}
			//}
			if(nnn == 1)break;

			//for(ii = 0; ii < LEGCOUNT; ii++)sub[ii] = phantomX.showPosition_of_2(ii);
			//myvector::VectorOutPut(phantomX.showPosition_of_2(0));
			//myvector::VectorOutPut(phantomX.showGlobalCoxaJointPosi(0));
			phantomX.setMyDirection(thP,thR,thY);//胴体回転
			//for(ii = 0; ii < LEGCOUNT; ii++){//胴体回転による脚先移動計算
			//	sub[ii] = myvector::VSub(sub[ii],phantomX.showPosition_of_2(ii));
			//	sub[ii] = myvector::VRot(sub[ii],thP-C_thPRY[0][0],thR-C_thPRY[1][0],thY-C_thPRY[2][0]);
			//	VCangeBodyToLeg( myvector::VSub(map[ii], phantomX.showGlobalCoxaJointPosi(ii)), thP, thR, thY);
			//}
			//myvector::VectorOutPut(phantomX.showPosition_of_2(0));
			//myvector::VectorOutPut(phantomX.showGlobalCoxaJointPosi(0));
			for(ii = 0; ii < LEGCOUNT; ii++){
				if(groundLeg[ii] == 1){//接地脚
					Leg_move[ii][i+1] = myvector::VSub(myvector::VRot( myvector::VSub(map[ii], phantomX.showGlobalMyPosition()), thP, thR, thY), phantomX.showLocalCoxaJointPosi(ii));
					r_leg2[ii] = Leg_move[ii][i+1];
					r_leg2[ii].z = -110;
					Leg_move[ii][i+1].z = c_leg[ii].z;
					r_leg[ii] = Leg_move[ii][i+1];
				}else{//遊脚はロボット座標系は変わらない（胴体と一緒に回転）
					Leg_move[ii][i+1] = phantomX.showLocalLegPosition(ii);
					r_leg[ii] = phantomX.showLocalLegPosition(ii);
					r_leg2[ii] = phantomX.showLocalPosition_of_2(ii);
				}
			}
			//std::cout<<"脚位置更新"<<std::endl;
			//if(Target_thY > 0.5){
			//std::cout<<"回転"<<std::endl;
			//std::cout<<"thY="<<thY<<std::endl;
			//for(ii = 0; ii < LEGCOUNT; ii++){if(groundLeg[ii]==1){std::cout<<"Leg="<<ii+1<<std::endl;myvector::VectorOutPut(r_leg2[ii]);myvector::VectorOutPut(r_leg[ii]);myvector::VectorOutPut(phantomX.showPosition_of_2(ii));/*myvector::VectorOutPut(phantomX.showGlobalCoxaJointPosi(ii));std::string stop;std::cin>>stop;*/}}
			//}
			phantomX.setPosition_of_2(r_leg2);//脚位置更新
			phantomX.setMyLegPosition(r_leg);
		}
	}

	return i;
}

myvector::SVector SearchPossibleBodyRotation::VCangeBodyToLeg(myvector::SVector& Vin, double thP, double thR, double thY){
	return myvector::VRot(myvector::VGet(Vin.x, -Vin.y, -Vin.z), thP, thR, -thY - M_PI/2.0);  // - 3.14/2はどうにかならないかな
}