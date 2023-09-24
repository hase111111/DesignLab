//! @file cassert_define.h
//! @brief デバッグモードとリリースモードでアサートを有効化・無効化するためのヘッダファイル
//! 
//! @details Visual Studioでは，デバッグモードではアサートを有効化し，リリースモードではアサートを無効化する．
//! @n しかし，このプログラムは処理が重いせいで，環境によってはそもそもデバッグモードで実行することが難しい
//! @n なので，リリースモードでアサートを出したい場合は，以下のリリースモードのところで
//! @n #undef NDEBUG でアサートを有効化すること．
//! @n NDEBUGがdefineされているときは，assertを無効化する．
//! @n #undef はdefineされているものを無効化する．
//! @n つまり，#undef NDEBUG は，assertを有効化する．
//! @n 逆に，#define NDEBUG は，assertを無効化する．
//! @n これらの処理は，cassertのインクルードより前に行う必要がある．
//! @n よってこのようなヘッダファイルを作成した．


#ifndef DESIGNLAB_CASSERT_DEFINE_H_
#define DESIGNLAB_CASSERT_DEFINE_H_


#ifndef _DEBUG	// if not define _DEBUG つまり，リリースモードの場合，

// アサートを有効化する場合は，以下の行のコメント( // )をはずすこと
#undef NDEBUG	

#endif 


#include <cassert>


#endif // !DESIGNLAB_CASSERT_DEFINE_H_