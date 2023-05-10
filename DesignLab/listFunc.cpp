#include "listFunc.h"
#include <iostream>

//ルートの深さを返す関数
int GetRouteDepth(LNODE* My_p) {
	int Depth = 0;
	while (My_p->parent != NULL) {
		Depth++;
		My_p = My_p->parent;
	}
	return Depth;
}


//ルートの経路を返す再帰的関数
int GetRoute(LNODE* My_p, LNODE** Root2My){
	int Depth; //深さ
	if (My_p -> parent != NULL){					//parentがNULLでない∴根ノードでなかったら
		Depth = 1+GetRoute(My_p -> parent, Root2My);//深さに1を足す
		Root2My[Depth] = My_p;						//自身のポインタを格納 Root2My[自身の深さ] 
		return Depth;
	}else{											//parentがNULL∴根ノードだったら
		Depth = 0;
		Root2My[Depth] = My_p;						//Root2My[0]
		return Depth;
	}
}



double AverageLegHeight(LNODE node, myvector::SVector *mapData_G, int mapDatanum) {
	// 接地脚の平均高さ
	double ret = 0.0;
	{
		bool Grounded[6] = {}; // 0 : 遊脚 1:接地
		double Grounded_z = 0.0;  // 接地している脚高さの平均を取る
		int Grounded_Leg_Count = 0;
		Hexapod t;
		//STarget target; // 使わないのでそのままぶち込んでる
		t.setMyDirection(node.pitch, node.roll, node.yaw);
		t.setMyPosition(node.global_center_of_mass);
		//t.setTarget(target);
		t.setMyLegPosition(node.Leg);
		for (int i = 0; i < 6; ++i) {
			myvector::SVector a = t.getGlobalLegPos(i);
			for (int j = 0; j < mapDatanum; ++j) {//mapDataBackUp=グローバル座標,mapData=原点ロボットの中心、方向グローバルと同じ
				myvector::SVector target_local_map_point = mapData_G[j];
				//std::cerr <<"!" << abs(a.x - node.Leg[i].x) << i << std::endl;
				if (j == 0) {
					/*std::cout << "脚は＝";
					myvector::VectorOutPut(t.getGlobalLegPos(i));
					std::cout << "マップは＝";
					myvector::VectorOutPut(mapData_G[j]);
					std::string wait;
					std::cin >> wait;*/
					//std::cout << (a.x - node.Leg[i].x) << i << std::endl;
				}
				if (abs(a.x - target_local_map_point.x) < 0.01 && abs(a.y - target_local_map_point.y) < 0.01 && abs(a.z - target_local_map_point.z) < 0.01) {
					//std::cerr << "Grounded Leg number : " << i + 1 << std::endl;
					ret += a.z;
					Grounded_Leg_Count++;
				}
			}
			myvector::VectorOutPut(a);
		}

		if (Grounded_Leg_Count == 0) {
			std::cerr << "宙に浮いている判定" << std::endl;
			ret = 0;
		} else if (Grounded_Leg_Count < 3) {
			std::cerr << "ヤバヤバ" << std::endl;
		}
		else {
			std::cerr << "接地判定" << std::endl;
			ret /= 1.0 * Grounded_Leg_Count;
		}
	}
	return ret;
}

bool LNODEEqual(const LNODE &node1, const LNODE &node2)
{
	//if ( (node1.v ==node2.v) && (node1.kaisou == node2.kaisou) && (isEqualVector(node1.global_center_of_mass, node2.global_center_of_mass) ) ){
	if ( (node1.leg_state == node2.leg_state) && (myvector::isEqualVector(node1.global_center_of_mass, node2.global_center_of_mass) ) )
	{
		for (int i = 0; i < 6; ++i) 
		{
			if (!myvector::isEqualVector(node1.Leg[i], node2.Leg[i])) {return 0; }
		}

		return true;
	}

	return false;
}

////leg_conと脚番号からその脚が接地しているか返す
////1:接地　0:遊脚
//inline bool isGrounded(int cond, int lnum) {
//	return ( (cond & (v_bit << shift_leg_bit[lnum])) ? 1 : 0 );
//}
////leg_conと脚番号からその脚位置を返す
//inline int numOfLegPosi(int cond, int lnum) {
//	return ( (cond & (kaisou_bit << shift_leg_bit[lnum])) >> shift_leg_bit[lnum] );
//}
//
//inline int numOfCOMType(int cond) {
//	return ((cond & COM_bit) >> shift_COM);
//}