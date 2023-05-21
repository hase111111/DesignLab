#pragma once
#include <string>

class GraphicConst final
{
public:
	const static std::string WIN_NAME;	//�E�B���h�E�̖��O�D
	const static int WIN_X;				//�E�B���h�E�̉����Ddxlib�ł̓E�B���h�E�̉������ɁC�E�����𐳂Ƃ��� X �����Ƃ�܂��D
	const static int WIN_Y;				//�E�B���h�E�̏c���Ddxlib�ł̓E�B���h�E�̏c�����ɁC�������𐳂Ƃ��� Y �����Ƃ�܂��D
	const static int COLOR_BIT;			//�F��\������bit���D�ʏ�32�ŗǂ����y������Ȃ�16�ɂ���D
private:

	//�R���X�g���N�^�ƃR�s�[�R���X�g���N�^���폜���Ď��̂𐶐��ł��Ȃ��悤�ɂ���D
	GraphicConst() = delete;
	GraphicConst(GraphicConst& _other) = delete;
};