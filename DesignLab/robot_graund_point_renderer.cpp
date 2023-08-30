#include "robot_graund_point_renderer.h"

#include "DxLib.h"
#include "designlab_dxlib.h"
#include "hexapod_state_calculator.h"
#include "leg_state.h"


RobotGraundPointRenderer::RobotGraundPointRenderer()
	: GRAUND_POINT_COLOR_RIGHT(GetColor(230, 15, 145)), GRAUND_POINT_COLOR_BLACK_RIGHT(GetColor(237, 159, 160)),
	GRAUND_POINT_COLOR_LEFT(GetColor(15, 230, 145)), GRAUND_POINT_COLOR_BLACK_LEFT(GetColor(159, 237, 160))
{
}


void RobotGraundPointRenderer::setNode(const std::vector<SNode>& node, const std::vector<size_t>& simu_end_node_index)
{
	HexapodStateCalclator hexapod_state_calclator;

	while (m_loaded_node_num < node.size())
	{
		int simu_num = 0;	//���̃m�[�h�̃V�~�����[�V�����ԍ�

		for (size_t i = 0; i < simu_end_node_index.size(); i++)
		{
			if (simu_end_node_index[i] >= m_loaded_node_num)
			{
				break;
			}
			++simu_num;
		}


		//���݂̃V�~�����[�V�����ԍ��̃f�[�^���Ȃ��Ȃ�Βǉ�����
		while (simu_num >= m_graund_point.size()) { m_graund_point.push_back({}); }


		for (int i = 0; i < HexapodConst::LEG_NUM; i++)
		{
			if (dl_leg::isGrounded(node[m_loaded_node_num].leg_state, i))
			{
				m_graund_point[simu_num].push_back({ hexapod_state_calclator.getGlobalLegPos(node[m_loaded_node_num], i, false) ,i });
			}
		}

		++m_loaded_node_num;
	}
}


void RobotGraundPointRenderer::draw(const size_t draw_simu_num, const bool draw_all_simulation) const
{
	unsigned int color[6] = { GRAUND_POINT_COLOR_RIGHT,GRAUND_POINT_COLOR_RIGHT,GRAUND_POINT_COLOR_RIGHT,GRAUND_POINT_COLOR_LEFT,GRAUND_POINT_COLOR_LEFT,GRAUND_POINT_COLOR_LEFT };

	unsigned int color_black[6] = { GRAUND_POINT_COLOR_BLACK_RIGHT,GRAUND_POINT_COLOR_BLACK_RIGHT,GRAUND_POINT_COLOR_BLACK_RIGHT,GRAUND_POINT_COLOR_BLACK_LEFT,GRAUND_POINT_COLOR_BLACK_LEFT,GRAUND_POINT_COLOR_BLACK_LEFT };

	for (size_t i = 0; i < m_graund_point.size(); i++)
	{
		for (size_t j = 0; j < m_graund_point[i].size(); j++)
		{
			if (draw_all_simulation || i == draw_simu_num)
			{
				dl_dxlib::drawCube3DWithTopPos(dl_dxlib::convertToDxVec(m_graund_point[i][j].first), 25, color[m_graund_point[i][j].second]);
			}
			else
			{
				SetDrawBlendMode(DX_BLENDMODE_ALPHA, 200);
				dl_dxlib::drawCube3DWithTopPos(dl_dxlib::convertToDxVec(m_graund_point[i][j].first), 25, color_black[m_graund_point[i][j].second]);
				SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);
			}
		}
	}
}
