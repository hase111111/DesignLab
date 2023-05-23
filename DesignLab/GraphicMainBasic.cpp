#include "GraphicMainBasic.h"
#include "Dxlib3DFunction.h"
#include "DxLib.h"
#include "MapRenderer.h"
#include "HexapodRenderer.h"

GraphicMainBasic::GraphicMainBasic(const GraphicDataBroker* _broker)
    : IGraphicMain(_broker), m_Map(mp_Broker->getMapState())
{
    //3D�n�̏����s���O�ɏ���������D
    myDxlib3DFunc::initDxlib3D();

    m_node.clear();
}

bool GraphicMainBasic::update()
{
    if (m_counter % GET_NODE_COUNT == 0) 
    {
        //�m�[�h��ǂݏo�����ԂɂȂ�����C�ǂݏo���D
        mp_Broker->copyOnlyNewNode(m_node);
    }

    if (m_counter % CHANGE_NEXT_NODE == 0) 
    {
        //�\������m�[�h���X�V���鎞�ԂɂȂ�����X�V����D
        m_display_node++;

        //�l���傫��������ۂ߂�D
        if (m_display_node >= (int)m_node.size()) { m_display_node = (int)m_node.size() - 1; }
    }

    if (m_node.empty() == false) 
    {
        //�m�[�h�����݂��Ă���Ȃ�΁C�J�����̏������s���D
        m_Camera.update();
        m_Camera.setTargetPos(myDxlib3DFunc::convertToDxVec(m_node.at(m_display_node).global_center_of_mass));

        //���{�b�g�̏�Ԃ��X�V����D
        m_hexapod.setMyPosition(m_node.at(m_display_node).global_center_of_mass);
        m_hexapod.setLocalLegPos(m_node.at(m_display_node).Leg);
    }

    m_counter++;    //�J�E���^��i�߂�D

    return false;
}

void GraphicMainBasic::draw() const
{
    //�}�b�v��`�悷��D
    MapRenderer _map_render;
    _map_render.draw(m_Map);

    if (m_node.empty() == false) 
    {
        //�m�[�h�����݂��Ă���Ȃ�΁C���{�b�g��`�悷��D
        HexapodRenderer _hexapod_render;
        _hexapod_render.draw(m_hexapod, m_node.at(m_display_node).leg_state);
    }
}
