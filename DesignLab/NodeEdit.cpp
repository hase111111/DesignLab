#include "NodeEdit.h"
#include "MapConst.h"
#include "MyMath.h"
#include "LegState.h"

void node_edit::initNode(SNode& _node, const bool _do_random)
{
	//�r���
	_node.leg_state = 0;

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		// �r�̏�Ԃ͑S�āC�r�ʒu 4 �Őڒn���Ă����� ( �܂� bit�� 1100 ) �ɏ���������D
		leg_state::changeLegState(_node.leg_state, i, 0b1100);
	}

	//�r�t���������_�Ƃ����C�r��̈ʒu������������D
	const float COM_Z = HexapodConst::VERTICAL_MIN_RANGE + MapConst::MAX_Z_BASE;	// ���{�b�g�̏d�S��Z���W

	_node.Leg[0] = my_vec::SVector(100.0f,	-120.0f,	-COM_Z);
	_node.Leg[1] = my_vec::SVector(0.0f,		-130.0f,	-COM_Z);
	_node.Leg[2] = my_vec::SVector(-100.0f,	-120.0f,	-COM_Z);
	_node.Leg[3] = my_vec::SVector(-100.0f,	120.0f,		-COM_Z);
	_node.Leg[4] = my_vec::SVector(0.0f,		130.0f,		-COM_Z);
	_node.Leg[5] = my_vec::SVector(100.0f,	120.0f,		-COM_Z);

	//�r�t���������_�Ƃ����C�r�ʒu4�ɊY��������W������������D
	_node.Leg2[0] = my_vec::SVector(100.0f,	-120.0f,	-COM_Z);
	_node.Leg2[1] = my_vec::SVector(0.0f,		-130.0f,	-COM_Z);
	_node.Leg2[2] = my_vec::SVector(-100.0f,	-120.0f,	-COM_Z);
	_node.Leg2[3] = my_vec::SVector(-100.0f,	120.0f,		-COM_Z);
	_node.Leg2[4] = my_vec::SVector(0.0f,		130.0f,		-COM_Z);
	_node.Leg2[5] = my_vec::SVector(100.0f,	120.0f,		-COM_Z);

	//�O���[�o�����W�̏d�S�ʒu�D�O���[�o�����W(0,0,0)�𒆐S�Ƃ����C���̕ϐ� _x�C_y�𔼌a�Ƃ���ȉ~�`�̂Ȃ��ɏd�S���ړ�����D
	const float _angle = _do_random ? my_math::generateRandomNumber(0.0f, 2.0f * my_math::MY_FLT_PI) : 0;
	const float _ex = _do_random ? my_math::generateRandomNumber(0.0f, 1.0f) : 0;

	const float _x = ((float)MapConst::MAP_START_ROUGH - MapConst::MAP_MIN_FORWARD) * 0.25f;
	const float _y = ((float)MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / 2.0f * 0.8f;

	_node.global_center_of_mass = my_vec::SVector(_ex * _x * cos(_angle), _ex * _y * sin(_angle), COM_Z);


	//���[���s�b�`���[�ŉ�]��\������D���{�b�g�̏d�S�𒆐S�ɂ��ĉ�]����D 
	_node.rot = my_vec::SRotator(0, 0, 0);

	_node.next_move = EHexapodMove::COM_UP_DOWN;
	_node.parent_num = -1;
	_node.depth = 0;


	//�ȉ��C�����g���C���Ȃ��p�����[�^�D

	_node.roll = 0.0f;			// x����]
	_node.pitch = 0.0f;			// y����]
	_node.yaw = 0.0f;			// z����]

	_node.parent = nullptr;		//�e�m�[�h�̃|�C���^
	_node.node_height = 1;		//�m�[�h����
	_node.debug = 24;			//���݉^�������Ƃ��Ďg�p,�O��̋r�㉺�m�[�h(�㉺�^���������ꍇ)2��,�O��̓���1��,�O�X��̓���1��

	_node.last_node_num = 0;
	_node.time = 0;

	_node.delta_comz = 0.0f;
	_node.target_delta_comz = 0;
}

void node_edit::changeParentNode(SNode& _node)
{
	_node.depth = 0;		//�[����0�ɂ���
	_node.parent_num = -1;	//���g���e�̂��߁C���̒l��������D
}

void node_edit::changeNextNode(SNode& _node, const int _parent_num, const EHexapodMove _next_move)
{
	_node.depth += 1;
	_node.parent_num = _parent_num;
	_node.next_move = _next_move;
}

std::string node_edit::getTextHexapodMove(const EHexapodMove _move)
{
	switch (_move)
	{
	case EHexapodMove::COM_MOVE:
		return "�d�S�̕��s�ړ�";

	case EHexapodMove::COM_UP_DOWN:
		return "�d�S�̏㉺�ړ�";

	case EHexapodMove::LEG_HIERARCHY_CHANGE:
		return "�r�̊K�w�ύX";

	case EHexapodMove::LEG_UP_DOWN:
		return "�r�̏㉺�ړ�";

	default:
		return "��`����ĂȂ�����ł��DNodeEdit.h��getTextHexapodMove�֐��ɓ����ǉ����Ă�������";
		break;
	}
}
