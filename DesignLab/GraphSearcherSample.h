#pragma once
#include "IGraphSearcher.h"

// IGraphSearcher を継承したい場合．以下のように
// 
//	class 好きな名前 final : public IGraphSearcher
//	{
//	}
// 
// と宣言します．finalはもうこれ以上継承しないよという意味．public IGraphSearcherはこのクラスを継承したよという意味です．
// IGraphSearcherを継承したクラスに課せられる制約はただ一つ，IGraphSearcherの純粋仮想関数，searchGraphTreeを実装することです．
// 以下のようにsearchGraphTreeをpublicなところに宣言して，後ろにoverrideとつけてください．



// GraphSearcherSampleクラスはテキトーに次の動作を選んで返します．あくまでこのクラスはIGraphSearcherの説明用です．
class GraphSearcherSample final : public IGraphSearcher
{
public:
	GraphSearcherSample () = default;
	~GraphSearcherSample() = default;

	bool searchGraphTree(const std::vector<SNode>& _graph, const STarget& _target, SNode& _output_result) override;

private:

};
