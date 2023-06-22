#pragma once
#include <map>
#include "MapCreator.h"
#include "MapConst.h"


class MapState final
{
private:

	std::vector<my_vec::SVector> m_map_data;					//���{�b�g�������}�b�v�D�r�ݒu�\�_�̏W���ŕ\�������D

	std::vector<std::vector<my_vec::SVector> > m_devide_map;	//�}�b�v�����݂���̈�𐳕��`�ɐ؂蕪���āC���̒��ɑ��݂���r�ݒu�\�_���W�߂����́D

	std::vector<float> m_devide_map_top_z;						//m_devide_map�̒��̍ł�����z���W���܂Ƃ߂����́C�v�f�����݂��Ȃ��Ȃ�Cfloat�^�̍ŏ��l������D


	// m_map_data �����Ƃ� m_devide_map�ɒl���Z�b�g����
	void makeDevideMap();

	// m_devide_map�ł͗v�f��1�����̔z��Ƃ��ĕ���ł���̂ŁC2�̓��͂���C�ǂ̒l���Q�Ƃ���΂悢�̂����v�Z����D
	inline int getDevideMapNum(const int _x, const int _y) const { return _x * MapConst::LP_DIVIDE_NUM + _y; }

public:
	MapState() = default;

	//! �n�`�f�[�^�̏��������s���D
	//! @param [in] _mode �񋓑� EMapCreateMode�łǂ̂悤�Ȓn�`�𐶐����邩�w�肷��D
	//! @param [in] _option �����o�ϐ��� OPTION_????�Ő�������}�b�v�̃I�v�V�������w�肷��D_mode��ReadFromFile���w�肳��Ă���Ȃ疳�������D
	//! �܂��Cbit���Z�𗘗p���ĕ����w��ł���D�Ⴆ�Ό��������C�K�i��ɂ������Ȃ�΁COPTION_PERFORATED | OPTION_STEP �Ǝw�肷��D
	//! @param [in] _do_output ���������}�b�v���t�@�C���o�͂���Ȃ�true�C���Ȃ��Ȃ��false�D����ꍇ�͐��������u�ԂɃt�@�C���o�͂����D
	void init(const EMapCreateMode _mode, const int _option, const bool _do_output);

	//! �����`��ɐ؂蕪����ꂽ�}�b�v����C�r�ݒu�\�_�̐����擾����D
	//! @param [in] _x X���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @param [in] _y Y���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @return int �r�ݒu�\�_�̐�
	int getPointNumFromDevideMap(const int _x, const int _y) const;

	//! �����`��ɐ؂蕪����ꂽ�}�b�v����C�r�ݒu�\�_�̎��ۂ̍��W���擾����D
	//! @param [in] _x X���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @param [in] _y Y���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @param [in] _num ���Ԗڂ̋r�ݒu�\�_���D 
	//! @return SVector �r�ݒu�\�_�̍��W�D
	my_vec::SVector getPosFromDevideMap(const int _x, const int _y, const int _num) const;

	//! �����`��ɐ؂蕪����ꂽ�}�b�v����C�ł�����Z���W��Ԃ��D
	//! @param [in] _x X���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @param [in] _y Y���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @return float �ł�����Z���W�D
	inline float getTopZFromDevideMap(const int _x, const int _y) const 
	{
		return m_devide_map_top_z.at(getDevideMapNum(_x, _y)); 
	};

	//! �r�ݒu�\�_�̍��W���o�͂���D�����`��ɐ؂蕪����ꂽ�}�b�v����l���擾����킯�ł͂Ȃ��̂ŁC�`��ɂ̂ݗ��p���邱�Ƃ𐄏��D
	//! @param [in] _num 
	//! @return SVector �r�ݒu�\�_�̍��W�D
	my_vec::SVector getPos(const int _num) const;

	//! �r�ݒu�\�_�̍��W�̐����o�͂���DgetPos�֐��ƕ��p���Ďg�p����D
	//! @return int �r�ݒu�\�_�̐�
	inline int getPosNum() const 
	{ 
		return (int)m_map_data.size(); 
	}

	//! �O���[�o����x���W�̒l����Cm_devide_map�̂ǂ��������Ă��邩�v�Z���ĕԂ��D
	//! @param [in] _posx �O���[�o����x���W�C�O���[�o���̓}�b�v�̌��_��0�Ƃ�����W�n�D���W�n�̌�����Svector�\���̂��Q�ƁD
	//! @return int m_devide_map��X���W�̂ǂ��������Ă��邩
	inline static int getDevideMapNumY(const float _posx)
	{
		int tmp = (int)((_posx - MapConst::MAP_MIN_HORIZONTAL) / (((float)MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / MapConst::LP_DIVIDE_NUM));
		if (tmp < 0) { return 0; }
		if (tmp >= MapConst::LP_DIVIDE_NUM) { return MapConst::LP_DIVIDE_NUM - 1; }
		return tmp;
	}

	//! �O���[�o����y���W�̒l����Cm_devide_map�̂ǂ��������Ă��邩�v�Z���ĕԂ��D
	//! @param [in] _posy �O���[�o����y���W�C�O���[�o���̓}�b�v�̌��_��0�Ƃ�����W�n�D���W�n�̌�����Svector�\���̂��Q�ƁD
	//! @return int m_devide_map��Y���W�̂ǂ��������Ă��邩
	inline static int getDevideMapNumX(const float _posy)
	{
		int tmp = (int)( (_posy - MapConst::MAP_MIN_FORWARD) / ( ((float)MapConst::MAP_MAX_FORWARD - MapConst::MAP_MIN_FORWARD) / MapConst::LP_DIVIDE_NUM) );
		if (tmp < 0)return 0;
		if (tmp >= MapConst::LP_DIVIDE_NUM)return MapConst::LP_DIVIDE_NUM - 1;
		return tmp;
	}
};


//! @file MapState.h
//! @brief �}�b�v��\���N���X�̎����D
//! @author ���J��

//! @class MapState
//! @brief �}�b�v��\���N���X�D
//! @details ���̃v���O�����ł́C���{�b�g�������}�b�v�͋r�ݒu�\�_�̏W���ŕ\�������D�ʂł͂Ȃ��_�̏W���D<br>
//! �r�ݒu�\�n�_�̃f�[�^�̓x�N�g���̔z��Ŏ������Ă���D<br>
//! �܂��C�������y�����邽�߂ɁC�}�b�v�����݂���̈�𒷕��`�ɐ؂蕪���āC���̒��ɑ��݂���r�ݒu�\�_���W�߂����̂ł���devide_map�������o�ϐ��ɂ���D<br>
//! devide_map�̗v�f�� https://atcoder.jp/contests/APG4b/tasks/APG4b_t �� �u1�����̔z��𑽎����z��Ƃ��Ďg���v�̗v�̂ŕ���ł���D<br>
//! <br>
//! �����o�ϐ��̃f�[�^�ւ̃A�N�Z�X�́C�����o�֐���get????�n�̊֐��ōs���D<br>
//! ���ڃf�[�^�̂������s��Ȃ��̂́C���f�[�^�ł���ƒl��ύX�\�ɂȂ��Ă��܂�����ł���Dconst�Ȋ֐����g���΁C�Ԉ���Ēl�̕ύX�����Ă��܂��댯����0�ɂȂ�̂ŁC�f�[�^�̂����ɗD���D
//! @author ���J��
