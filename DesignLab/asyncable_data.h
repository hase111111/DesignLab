#ifndef DESIGNLAB_ASYNCABLE_DATA_H_
#define DESIGNLAB_ASYNCABLE_DATA_H_

#include <vector>

#include <boost/thread.hpp>


// �e���v���[�g�̎����� .h�ɏ������ق��������炵�� https://qiita.com/i153/items/38f9688a9c80b2cb7da7


//! @class AsyncableData
//! @brief �񓯊��������s���ۂɁC�f�[�^�̍X�V�񐔂ƃf�[�^���܂Ƃ߂Ĉ������߂̍\����
//! @details ���̍\���̂́C�f�[�^�̍X�V�񐔂ƃf�[�^���܂Ƃ߂Ĉ������߂̍\���́D
//! @n �l�̕ύX���s���ۂɁC�f�[�^�̍X�V�񐔂��C���N�������g���邱�ƂŁC�f�[�^�̍X�V���s���D
//! @n �܂��C�l�̎Q�ƂƕύX���s���ۂɃ~���[�e�b�N�X��p���āC�����ɕύX����邱�Ƃ�h���D
template <typename T>
class AsyncableData
{
public:

	AsyncableData() : update_count_(0) {};
	AsyncableData(const T& data) : data_(data), update_count_(0) {};

	//!< �R�s�[�E���[�u�͋֎~
	AsyncableData(const AsyncableData&) = delete;
	AsyncableData& operator=(const AsyncableData&) = delete;
	AsyncableData(AsyncableData&&) = delete;

	//! @breif �l���R�s�[���ĕԂ��D
	//! @n ���̎��Cread lock��������D
	//! @n ���R�C�f�[�^�̍X�V�񐔂̓C���N�������g����Ȃ��D
	//! @return T �l�̃R�s�[
	T data() const
	{
		//�ǂݎ��p�̃��b�N��������D���̃X�R�[�v { } �𔲂���܂Ń��b�N��������D(�܂肱�̊֐����I���܂�)
		boost::shared_lock<boost::shared_mutex> read_lock(mtx_);
		return data_;
	};


	//! @brief �l��ύX����D
	//! @n ���̎��Cwrite lock��������D
	//! @n �f�[�^�̍X�V�񐔂��C���N�������g����D
	//! @param [in] data �l
	void set_data(const T& data)
	{
		//�������ݗp�̃��b�N��������D�܂��́Cupgrade_lock��p�ӂ��āC�����unique_lock�ɕύX����D
		boost::upgrade_lock<boost::shared_mutex> upgrade_lock(mtx_);

		{
			boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

			data_ = data;
			++update_count_;
		}
	};

	//! @brief �f�[�^�̍X�V�񐔂�Ԃ��D
	//! @return int �f�[�^�̍X�V��
	//! @n ���̎��Cread lock��������D
	int update_count() const
	{
		//�ǂݎ��p�̃��b�N��������D���̃X�R�[�v { } �𔲂���܂Ń��b�N��������D(�܂肱�̊֐����I���܂�)
		boost::shared_lock<boost::shared_mutex> read_lock(mtx_);
		return update_count_;
	};

private:

	mutable boost::shared_mutex mtx_;	//!< ���b�N�p��mutex

	T data_;

	int update_count_;
};


template <typename T>
class AsyncableData <std::vector<T>>
{
public:

	AsyncableData() : data_({}), update_count_(0) {};
	AsyncableData(const std::vector<T>& data) : data_(data), update_count_(0) {}

	//!< �R�s�[�E���[�u�͋֎~
	AsyncableData(const AsyncableData&) = delete;
	AsyncableData& operator=(const AsyncableData&) = delete;
	AsyncableData(AsyncableData&&) = delete;

	//! @breif �l���R�s�[���ĕԂ��D
	//! @n ���̎��Cread lock��������D
	//! @n ���R�C�f�[�^�̍X�V�񐔂̓C���N�������g����Ȃ��D
	//!	@return T �l�̃R�s�[
	std::vector<T> data() const
	{
		//�ǂݎ��p�̃��b�N��������D���̃X�R�[�v { } �𔲂���܂Ń��b�N��������D(�܂肱�̊֐����I���܂�)
		boost::shared_lock<boost::shared_mutex> read_lock(mtx_);
		return data_;
	};

	//! @brief �l��ύX����D
	//! @n ���̎��Cwrite lock��������D
	//! @n �f�[�^�̍X�V�񐔂��C���N�������g����D
	//! @param [in] data �l
	void set_data(const std::vector<T>& data)
	{
		//�������ݗp�̃��b�N��������D�܂��́Cupgrade_lock��p�ӂ��āC�����unique_lock�ɕύX����D
		boost::upgrade_lock<boost::shared_mutex> upgrade_lock(mtx_);

		{
			boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

			data_ = data;
			++update_count_;
		}
	};

	//! @brief push_back���s���D
	//! @n ���̎��Cwrite lock��������D
	//! @n �f�[�^�̍X�V�񐔂��C���N�������g����D
	//! @param [in] data �l
	void push_back(const T& data)
	{
		//�������ݗp�̃��b�N��������D�܂��́Cupgrade_lock��p�ӂ��āC�����unique_lock�ɕύX����D
		boost::upgrade_lock<boost::shared_mutex> upgrade_lock(mtx_);

		{
			boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

			data_.push_back(data);
			++update_count_;
		}
	};

	//! @brief clean���s���D
	//! @n ���̎��Cwrite lock��������D
	//! @n �f�[�^�̍X�V�񐔂��C���N�������g����D
	//! @param [in] data �l
	void clean()
	{
		//�������ݗp�̃��b�N��������D�܂��́Cupgrade_lock��p�ӂ��āC�����unique_lock�ɕύX����D
		boost::upgrade_lock<boost::shared_mutex> upgrade_lock(mtx_);

		{
			boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

			data_.clear();
			++update_count_;
		}
	};

	//! @brief size��Ԃ��D
	//! @n ���̎��Cread lock��������D
	//! @return size_t
	size_t size() const
	{
		//�ǂݎ��p�̃��b�N��������D���̃X�R�[�v { } �𔲂���܂Ń��b�N��������D(�܂肱�̊֐����I���܂�)
		boost::shared_lock<boost::shared_mutex> read_lock(mtx_);
		return data_.size();
	};

	int update_count() const
	{
		//�ǂݎ��p�̃��b�N��������D���̃X�R�[�v { } �𔲂���܂Ń��b�N��������D(�܂肱�̊֐����I���܂�)
		boost::shared_lock<boost::shared_mutex> read_lock(mtx_);
		return update_count_;
	};

private:

	mutable boost::shared_mutex mtx_;	//!< ���b�N�p��mutex

	std::vector<T> data_;

	int update_count_;
};
#endif