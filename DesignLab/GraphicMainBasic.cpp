#include "GraphicMainBasic.h"
#include "Dxlib3DFunction.h"
#include "DxLib.h"
#include "MapRenderer.h"

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
        m_Camera.setTargetPos(myDxlib3DFunc::convertToDxVec(m_node.at(m_display_node).global_center_of_mass));

        //���{�b�g�̏�Ԃ��X�V����D
        m_HexapodRender.update(m_node.at(m_display_node));
    }

    m_counter++;    //�J�E���^��i�߂�D

    m_GUI.update(m_Camera); //GUI���X�V����D

    m_Camera.update();  //�J�������X�V����

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
        m_HexapodRender.draw(m_node.at(m_display_node));

        //UI��\������D
        m_GUI.draw(m_node.at(m_display_node));
    }
}
