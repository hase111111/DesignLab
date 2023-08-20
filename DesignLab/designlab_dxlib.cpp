#include "designlab_dxlib.h"

#include "DxLib.h"

#include "graphic_const.h"


void dl_dxlib::initDxlib3D()
{
	//カメラの描画範囲を設定する
	SetCameraNearFar(GraphicConst::CAMERA_NEAR, GraphicConst::CAMERA_FAR);

	SetUseLighting(FALSE);					// ライティングの計算をしないように設定を変更	
	SetUseBackCulling(FALSE);				// ポリゴンの両面を描画する．
	SetFogEnable(FALSE);					// フォグは使用しない．
}


void dl_dxlib::setZBufferEnable()
{
	// Ｚバッファを有効にする
	SetUseZBuffer3D(TRUE);

	// Ｚバッファへの書き込みを有効にする
	SetWriteZBuffer3D(TRUE);
}


void dl_dxlib::drawCube3D(const VECTOR& center_pos, const float side_len, const unsigned int color)
{
	//立方体は8つの頂点を持つので，それらの座標を計算する．
	const VECTOR _vertex[8] =
	{
		VGet(center_pos.x - side_len / 2,center_pos.y - side_len / 2,center_pos.z - side_len / 2),
		VGet(center_pos.x + side_len / 2,center_pos.y - side_len / 2,center_pos.z - side_len / 2),
		VGet(center_pos.x + side_len / 2,center_pos.y - side_len / 2,center_pos.z + side_len / 2),
		VGet(center_pos.x - side_len / 2,center_pos.y - side_len / 2,center_pos.z + side_len / 2),
		VGet(center_pos.x - side_len / 2,center_pos.y + side_len / 2,center_pos.z - side_len / 2),
		VGet(center_pos.x + side_len / 2,center_pos.y + side_len / 2,center_pos.z - side_len / 2),
		VGet(center_pos.x + side_len / 2,center_pos.y + side_len / 2,center_pos.z + side_len / 2),
		VGet(center_pos.x - side_len / 2,center_pos.y + side_len / 2,center_pos.z + side_len / 2)
	};

	// 3D描画の関数は3角形を基本単位とするので，4角形の面を張りたい場合は，2つの三角形を組み合わせる必要がある．つまり，6面×2つ＝12個の三角形で立方体が描画できる．

	DrawTriangle3D(_vertex[0], _vertex[1], _vertex[2], color, TRUE);
	DrawTriangle3D(_vertex[2], _vertex[3], _vertex[0], color, TRUE);

	DrawTriangle3D(_vertex[4], _vertex[5], _vertex[6], color, TRUE);
	DrawTriangle3D(_vertex[6], _vertex[7], _vertex[4], color, TRUE);

	DrawTriangle3D(_vertex[4], _vertex[7], _vertex[0], color, TRUE);
	DrawTriangle3D(_vertex[0], _vertex[7], _vertex[3], color, TRUE);

	DrawTriangle3D(_vertex[1], _vertex[2], _vertex[5], color, TRUE);
	DrawTriangle3D(_vertex[5], _vertex[6], _vertex[2], color, TRUE);

	DrawTriangle3D(_vertex[0], _vertex[1], _vertex[5], color, TRUE);
	DrawTriangle3D(_vertex[5], _vertex[4], _vertex[0], color, TRUE);

	DrawTriangle3D(_vertex[2], _vertex[3], _vertex[7], color, TRUE);
	DrawTriangle3D(_vertex[7], _vertex[6], _vertex[2], color, TRUE);

}


void dl_dxlib::drawCube3DWithTopPos(const VECTOR& top_pos, const float side_len, const unsigned int color)
{
	drawCube3D(VSub(top_pos, VGet(0, 0, side_len / 2)), side_len, color);
}


void dl_dxlib::drawHexagon(const VECTOR vertex[HexapodConst::LEG_NUM], const unsigned int color)
{
	// 3D描画の関数は3角形を基本単位とするので，6角形の面を張りたい場合は，4つの三角形を組み合わせる必要がある．
	DrawTriangle3D(vertex[0], vertex[1], vertex[5], color, TRUE);
	DrawTriangle3D(vertex[1], vertex[2], vertex[4], color, TRUE);
	DrawTriangle3D(vertex[1], vertex[4], vertex[5], color, TRUE);
	DrawTriangle3D(vertex[2], vertex[3], vertex[4], color, TRUE);
}


void dl_dxlib::drawHexagonalPrism(const VECTOR vertex[HexapodConst::LEG_NUM], const float height, const unsigned int color)
{
	//6角形面の法線方向のベクトルを取得する．やっている処理としては，頂点0から1へ行くベクトルをv01，同様に頂点0から2へ行くベクトルをv02とすると，
	// v01とv02の外積(Cross)をとると法線方向のベクトルが取得できるため，これを単位ベクトルに変換(Norm，ノーマライズのこと)し，高さの半分だけ倍にした．
	const VECTOR kCenterToTop = VScale(VNorm(VCross(VSub(vertex[0], vertex[1]), VSub(vertex[0], vertex[2]))), height / 2.0f);

	//上面の頂点．
	const VECTOR kVertexTop[HexapodConst::LEG_NUM] =
	{
		VAdd(vertex[0],kCenterToTop),
		VAdd(vertex[1],kCenterToTop),
		VAdd(vertex[2],kCenterToTop),
		VAdd(vertex[3],kCenterToTop),
		VAdd(vertex[4],kCenterToTop),
		VAdd(vertex[5],kCenterToTop)
	};

	//底面の頂点
	const VECTOR kVertexBottom[HexapodConst::LEG_NUM] =
	{
		VSub(vertex[0],kCenterToTop),
		VSub(vertex[1],kCenterToTop),
		VSub(vertex[2],kCenterToTop),
		VSub(vertex[3],kCenterToTop),
		VSub(vertex[4],kCenterToTop),
		VSub(vertex[5],kCenterToTop)
	};

	drawHexagon(kVertexTop, color);		//上面を描画する．
	drawHexagon(kVertexBottom, color);	//底面を描画する．

	//側面を描画していく．側面は四角形6つで構成されるので，3角形が12こ必要になる．
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		DrawTriangle3D(kVertexTop[i % HexapodConst::LEG_NUM], kVertexTop[(i + 1) % HexapodConst::LEG_NUM], kVertexBottom[i % HexapodConst::LEG_NUM], color, TRUE);
		DrawTriangle3D(kVertexTop[(i + 1) % HexapodConst::LEG_NUM], kVertexBottom[i % HexapodConst::LEG_NUM], kVertexBottom[(i + 1) % HexapodConst::LEG_NUM], color, TRUE);
	}

}
