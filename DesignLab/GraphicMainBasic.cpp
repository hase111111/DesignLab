#include "GraphicMainBasic.h"
#include "Dxlib3DFunction.h"

GraphicMainBasic::GraphicMainBasic(const GraphicDataBroker* _broker)
    : AbstractGraphicMain(_broker), m_Map(mp_Broker->getMapState())
{
    //3D系の処理行う前に初期化する．
    myDxlib3DFunc::initDxlib3D();
}

bool GraphicMainBasic::update()
{
    return false;
}

void GraphicMainBasic::draw() const
{
}
