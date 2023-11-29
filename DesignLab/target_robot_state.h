//! @file target_robot_state.h
//! @brief ロボットの目標地点や目標姿勢を表現する構造体

#ifndef DESIGNLAB_TARGET_ROBOT_STATE_H_
#define DESIGNLAB_TARGET_ROBOT_STATE_H_

#include <optional>

#include "designlab_quaternion.h"
#include "designlab_vector3.h"


//! @enum TargetMode
//! @brief Targetへの移動をどのように評価するか表す列挙体
//! @details [列挙体について]
//! @n 以下で定義されているTargetModeが列挙体に当たる
//! @n イメージとしては以下に宣言された値しかとることのできない新しい変数を定義するような感じ．
//! @n こうすることで関数を特定のモードで動作させたいときなどに，入力されたくない数字が入力されずらくなる．
//! @n また値に名前を付けることで何を想定してこの値が代入されたのかが分かりやすくなるという利点がある．
//! @n C++ には enum と enum class という2通りの列挙体があるが，安全のため enum class の使用をおすすめする．
//! @n よくわからんっていうのであれば TargetMode の記述を真似して書いてみて．
enum class TargetMode : int
{
	kNone,					//!< 評価しない．
	kStraightMoveVector,	//!< 直線移動をさせる（移動したい方向をベクトルで示す）．
	kStraightMovePosition,	//!< 直線移動をさせる（移動したい座標を示す）．
	kSpotTurnLastPosture,	//!< その場で旋回させる（最終的な姿勢Postureを示す）．
	kSpotTurnRotAxis,		//!< その場で旋回させる（回転軸を示し，その軸周りの右ねじの回転）．
};


//! @class TargetRobotState
//! @brief 探索において目標となる座標や角度，評価する値についてまとめた構造体
class TargetRobotState final
{
public:

	//! @brief 進行方向を指定する．正規化されたベクトルを渡すこと．
	//! @param [in] straight_move_vector 進行方向を表すベクトル．
	inline void SetStraightMoveVector(const ::designlab::Vector3& straight_move_vector)
	{
		assert(::designlab::math_util::IsEqual(straight_move_vector.GetLength(), 1.0f));

		ResetAllData();

		straight_move_vector_ = straight_move_vector;
		target_mode = TargetMode::kStraightMoveVector;
	}

	//! @brief 進行方向を取得する．値を設定していない場合はエラー．
	//! @return ::designlab::Vector3 進行方向を表すベクトル．
	[[nodiscard]] constexpr ::designlab::Vector3 GetStraightMoveVector() const
	{
		assert(target_mode == TargetMode::kStraightMoveVector);	//SetStraightMoveVector()が呼ばれていない場合はエラー
		assert(straight_move_vector_.has_value());

		return straight_move_vector_.value();
	}

	//! @brief 目標到達地点を指定する．グローバル座標．
	//! @param [in] straight_move_position 目標到達地点．
	inline void SetStraightMovePosition(const ::designlab::Vector3& straight_move_position)
	{
		ResetAllData();

		straight_move_position_ = straight_move_position;
		target_mode = TargetMode::kStraightMovePosition;
	}

	//! @brief 目標到達地点を取得する．値を設定していない場合はエラー．
	//! @return ::designlab::Vector3 目標到達地点．
	[[nodiscard]] constexpr ::designlab::Vector3 GetStraightMovePosition() const
	{
		assert(target_mode == TargetMode::kStraightMovePosition);	//SetStraightMovePosition()が呼ばれていない場合はエラー
		assert(straight_move_position_.has_value());

		return straight_move_position_.value();
	}

	//! @brief その場旋回時の目標姿勢を指定する．
	//! @param [in] spot_turn_last_posture_ 目標姿勢．
	inline void SetSpotTurnLastPosture(const ::designlab::Quaternion& last_posture)
	{
		ResetAllData();

		spot_turn_last_posture_ = last_posture;
		target_mode = TargetMode::kSpotTurnLastPosture;
	}

	//! @brief その場旋回時の目標姿勢を取得する．値を設定していない場合はエラー．
	//! @return ::designlab::Quaternion 目標姿勢．
	[[nodiscard]] constexpr ::designlab::Quaternion GetSpotTurnLastPosture() const
	{
		assert(target_mode == TargetMode::kSpotTurnLastPosture);	//SetSpotTurnLastPosture()が呼ばれていない場合はエラー
		assert(spot_turn_last_posture_.has_value());

		return spot_turn_last_posture_.value();
	}

	//! @brief その場旋回時の回転軸を指定する．
	//! @param [in] spot_turn_rot_axis 回転軸．
	inline void SetSpotTurnRotAxis(const ::designlab::Vector3& rot_axis)
	{
		ResetAllData();

		spot_turn_rot_axis = rot_axis;
		target_mode = TargetMode::kSpotTurnRotAxis;
	}

	//! @brief その場旋回時の回転軸を取得する．値を設定していない場合はエラー．
	//! @return ::designlab::Vector3 回転軸．
	[[nodiscard]] constexpr ::designlab::Vector3 GetSpotTurnRotAxis() const
	{
		assert(target_mode == TargetMode::kSpotTurnRotAxis);	//SetSpotTurnRotAxis()が呼ばれていない場合はエラー
		assert(spot_turn_rot_axis.has_value());

		return spot_turn_rot_axis.value();
	}

	[[nodiscard]] constexpr TargetMode GetTargetMode() const
	{
		return target_mode;
	}

private:

	//! @brief この構造体のメンバ変数をすべてnullopt(値なし)にする．
	inline void ResetAllData()
	{
		straight_move_vector_ = std::nullopt;
		straight_move_position_ = std::nullopt;
		spot_turn_last_posture_ = std::nullopt;
		spot_turn_rot_axis = std::nullopt;
	}

	std::optional<::designlab::Vector3> straight_move_vector_{ std::nullopt };		//!< 目標方向．正規化されたベクトル．
	std::optional<::designlab::Vector3> straight_move_position_{ std::nullopt };	//!< 目標位置（グローバル座標）．
	std::optional<::designlab::Quaternion> spot_turn_last_posture_{ std::nullopt };	//!< 目標姿勢(posture)．
	std::optional<::designlab::Vector3> spot_turn_rot_axis{ std::nullopt };			//!< 旋回時の回転軸．右ねじの回転．

	TargetMode target_mode{ TargetMode::kNone };	//!< どうやって目標を評価するか
};


#endif	// DESIGNLAB_TARGET_ROBOT_STATE_H_
