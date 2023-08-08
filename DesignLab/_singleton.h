#pragma once

//! @class Singleton
//! @date 2023/08/07
//! @author 長谷川
//! @brief このクラスを継承するとSingletonクラスになる．グラフ探索では絶対に使わないこと!
//! @details Singletonクラスとは，C言語でいうところのグローバル変数（どこからでも値を変更できる変数）である．
//! @n Singletonパターンは悪しき構築の一つなのでなるべく使わないほうがよい．
//! @n このプロジェクトでは画像表示クラスでキーボードとマウスの入力を管理するために使用している．@n 
//! @n グラフ探索では""絶対""に使わないこと!! @n 絶対だよ!!!! @n
//! @n 使い方については 継承先の keyboard.h を参照してみてほしい．
template <typename _T>
class Singleton
{
protected:
	Singleton() = default;
	virtual ~Singleton() = default;
	Singleton(const Singleton& r) = default;
	Singleton& operator=(const Singleton& r) = default;

public:

	//! @brief インスタンスを取得する．@n
	//! このクラスを継承したクラスは クラス名::getIns()-> の形式でメンバ関数を呼び出す．
	//! @return インスタンスのポインタ
	static _T* getIns()
	{
		static _T inst;
		return &inst;
	};

};

//! @file singleton.h
//! @date 2023/08/07
//! @author 長谷川
//! @brief Singletonクラスのためのテンプレートクラス
//! @n 行数 : @lineinfo
