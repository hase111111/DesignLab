#pragma once

#include <map>
#include <vector>

#include "designlab_vector.h"
#include "map_creator.h"
#include "map_const.h"


//! @class MapState
//! @date 2023/08/06
//! @author ���J��
//! @brief �}�b�v��\���N���X�D
//! @details ���̃v���O�����ł́C���{�b�g�������}�b�v�͋r�ݒu�\�_�̏W���ŕ\�������D�ʂł͂Ȃ��_�̏W���D
//! @n �r�ݒu�\�n�_�̃f�[�^�̓x�N�g���̔z��Ŏ������Ă���D
//! @n �܂��C�������y�����邽�߂ɁC�}�b�v�����݂���̈�𒷕��`�ɐ؂蕪���āC���̒��ɑ��݂���r�ݒu�\�_���W�߂����̂ł���devide_map�������o�ϐ��ɂ���D
//! @n devide_map�̗v�f�� https://atcoder.jp/contests/APG4b/tasks/APG4b_t �� �u1�����̔z��𑽎����z��Ƃ��Ďg���v�̗v�̂ŕ���ł���D@n 
//! @n �����o�ϐ��̃f�[�^�ւ̃A�N�Z�X�́C�����o�֐���get????�n�̊֐��ōs���D
//! @n ���ڃf�[�^�̂������s��Ȃ��̂́C���f�[�^�ł���ƒl��ύX�\�ɂȂ��Ă��܂�����ł���Dconst�Ȋ֐����g���΁C�Ԉ���Ă��l�̕ύX���ł��Ȃ��̂ŁC�f�[�^�̂����ɗD���D
class MapState final
{
public:

	MapState() = default;

	MapState& operator = (const MapState& other);

	//! @breif �n�`�f�[�^�̏��������s���D
	//! @param [in] mode �񋓑� EMapCreateMode�łǂ̂悤�Ȓn�`�𐶐����邩�w�肷��D
	//! @param [in] option �����o�ϐ��� OPTION_????�Ő�������}�b�v�̃I�v�V�������w�肷��Dmode��ReadFromFile���w�肳��Ă���Ȃ疳�������D
	//! @n �܂��Cbit���Z�𗘗p���ĕ����w��ł���D�Ⴆ�Ό��������C�K�i��ɂ������Ȃ�΁COPTION_PERFORATED | OPTION_STEP �Ǝw�肷��D
	//! @param [in] do_output ���������}�b�v���t�@�C���o�͂���Ȃ�true�C���Ȃ��Ȃ��false�D����ꍇ�͐��������u�ԂɃt�@�C���o�͂����D
	void init(const EMapCreateMode mode, const int option, const bool do_output);

	//! @brief �O���[�o����x���W�̒l����Cm_devide_map�̂ǂ��������Ă��邩�v�Z���ĕԂ��D
	//! @n �͈͊O�̒l���w�肵���ꍇ�́C0�܂��̓}�b�v�̒[�̍��W��Ԃ��D
	//! @param [in] posx �O���[�o����y���W�C�O���[�o���̓}�b�v�̌��_��0�Ƃ�����W�n�D���W�n�̌�����Svector�\���̂��Q�ƁD
	//! @return int m_devide_map��x���W�̂ǂ��������Ă��邩
	inline static int getDevideMapNumX(const float posx)
	{
		const int res = static_cast<int>((posx - MapConst::MAP_MIN_FORWARD) / (((float)MapConst::MAP_MAX_FORWARD - MapConst::MAP_MIN_FORWARD) / MapConst::LP_DIVIDE_NUM));
		if (res < 0)return 0;
		if (res >= MapConst::LP_DIVIDE_NUM)return MapConst::LP_DIVIDE_NUM - 1;
		return res;
	}

	//! @brief �O���[�o����y���W�̒l����Cm_devide_map�̂ǂ��������Ă��邩�v�Z���ĕԂ��D
	//! @n �͈͊O�̒l���w�肵���ꍇ�́C0�܂��̓}�b�v�̒[�̍��W��Ԃ��D
	//! @param [in] posy �O���[�o����y���W�C�O���[�o���̓}�b�v�̌��_��0�Ƃ�����W�n�D���W�n�̌����͍\����SVector���Q�ƁD
	//! @return int m_devide_map��y���W�̂ǂ��������Ă��邩
	inline static int getDevideMapNumY(const float posy)
	{
		const int res = static_cast<int>((posy - MapConst::MAP_MIN_HORIZONTAL) / (((float)MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / MapConst::LP_DIVIDE_NUM));
		if (res < 0) { return 0; }
		if (res >= MapConst::LP_DIVIDE_NUM) { return MapConst::LP_DIVIDE_NUM - 1; }
		return res;
	}

	//! @brief �����`��ɐ؂蕪����ꂽ�}�b�v����C�r�ݒu�\�_�̐����擾����D
	//! @n �͈͊O�̒l���w�肵���ꍇ�́C0��Ԃ��D
	//! @param [in] x X���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @param [in] y Y���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @return int �r�ݒu�\�_�̐�
	int getPointNumFromDevideMap(const int x, const int y) const;

	//! @brief �����`��ɐ؂蕪����ꂽ�}�b�v����C�r�ݒu�\�_�̎��ۂ̍��W���擾����D
	//! @n �͈͊O�̒l���w�肵���ꍇ�́C(0,0,0)��Ԃ��D
	//! @param [in] x x���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @param [in] y y���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @param [in] num ���Ԗڂ̋r�ݒu�\�_���D 
	//! @return SVector �r�ݒu�\�_�̍��W�D
	dl_vec::SVector getPosFromDevideMap(const int x, const int y, const int num) const;

	//! @brief �����`��ɐ؂蕪����ꂽ�}�b�v����C�ł�����Z���W��Ԃ��D
	//! @param [in] x X���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @param [in] y Y���W�C�؂蕪����ꂽ�^�C���̈ʒu�Ŏw�肷��D
	//! @return float �ł�����Z���W�D
	inline float getTopZFromDevideMap(const int x, const int y) const
	{
		return m_devide_map_top_z.at(getDevideMapNum(x, y));
	};

	//! @brief �r�ݒu�\�_�̍��W���o�͂���D�����`��ɐ؂蕪����ꂽ�}�b�v����l���擾����킯�ł͂Ȃ��̂ŁC�`���f�o�b�O�݂̂ɗ��p���邱�Ƃ𐄏��D
	//! @param [in] num 
	//! @return SVector �r�ݒu�\�_�̍��W�D
	dl_vec::SVector getPos(const int num) const;

	//! @brief �r�ݒu�\�_�̍��W�̐����o�͂���DgetPos�֐��ƕ��p���Ďg�p����D
	//! @return int �r�ݒu�\�_�̑���
	inline int getPosNum() const
	{
		return static_cast<int>(m_map_data.size());
	}


private:

	const float kMapMinZ = -100000.0f;		//�}�b�v�̍Œ��Z���W


	// m_map_data �����Ƃ� m_devide_map�ɒl���Z�b�g����
	void makeDevideMap();

	// m_devide_map�ł͗v�f��1�����̔z��Ƃ��ĕ���ł���̂ŁC2�̓��͂���C�ǂ̒l���Q�Ƃ���΂悢�̂����v�Z����D
	inline int getDevideMapNum(const int x, const int y) const { return x * MapConst::LP_DIVIDE_NUM + y; }


	std::vector<dl_vec::SVector> m_map_data;					//���{�b�g�������}�b�v�D�r�ݒu�\�_�̏W���ŕ\�������D

	std::vector<std::vector<dl_vec::SVector> > m_devide_map;	//�}�b�v�����݂���̈�𐳕��`�ɐ؂蕪���āC���̒��ɑ��݂���r�ݒu�\�_���W�߂����́D

	std::vector<float> m_devide_map_top_z;						//m_devide_map�̒��̍ł�����z���W���܂Ƃ߂����́C�v�f�����݂��Ȃ��Ȃ�Cfloat�^�̍ŏ��l������D

};


//! @file map_state.h
//! @date 2023/08/06
//! @author ���J��
//! @brief �}�b�v�̃f�[�^���Ǘ�����N���X�D
//! @n �s�� : @lineinfo
