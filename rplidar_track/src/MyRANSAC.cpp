#include "rplidar_track/MyRANSAC.h"
#include <math.h>
#include <algorithm>
#include <string.h>

MyRANSAC::MyRANSAC()
	:probability_(0.99)
	, max_iterations_ (1000)
{

}

double MyRANSAC::compute(std::vector<double> &model_parameters, 
	ParameterEsitmator<Point2D,double> *param_estimator , 
	std::vector<Point2D> &data, 
	int num_for_estimate	//������ģ�͵��������ݸ���
	)
{

	int num_data = data.size();

	int iterations = 0;	
	double k = max_iterations_;	//�㷨�ĵ�������


	double log_probability  = log (1.0 - probability_);
	double one_over_indices = 1.0 / static_cast<double> (num_data);
	
	

	short *best_votes = new short[num_data]; //���ģ�͵ĵ㼯������־, data[i] �������ģ����1������0�����ڼ�¼����ģ�͵ĵ���������Լ�ͳ�Ƶ���
	short *cur_votes = new short[num_data];  //��ǰģ�͵ĵ㼯������־��data[i] ���ϵ�ǰģ����1������0

	SubSetIndexComparator sub_set_index_comparator(num_for_estimate);		//�Ƚ����������ж���ʼ�����Ƿ�һ��
	std::set<int *, SubSetIndexComparator > chosen_sub_sets(sub_set_index_comparator);	//���ڼ�¼�Ѿ��ù����Ӽ��������ظ�


	int* cur_inti_sub_set_indexs = NULL;	//��ʼѡ�������
	

	int best_model_num = -1;	//���ģ���еĵ���
	int maybe_inliers_num = 0;	//����ģ���еĵ���
	std::vector<double> maybe_model;	//��ǰ�ҵ���ģ��

	std::vector<int> shuffled_indices(num_data);//����ȡ��ʼ����

	while (iterations < k)
	{

		maybe_inliers_num = 0;

		//�����ݼ������ѡ��n����
		cur_inti_sub_set_indexs = new int[num_for_estimate];
					

		//��ǰ�ҵ���ģ�Ͳ���
		maybe_model.clear();

		//����		
		for (int i=0;i < (int)shuffled_indices.size(); i++)
		{
			shuffled_indices[i]=i;
		}

		//���ѡ��������
		int max_index = num_data-1;
		for (int i=0; i<num_for_estimate; i++)
		{
			std::swap(shuffled_indices[i], shuffled_indices[i + rand() % (data.size() - i)]);
		}
		
		memset(cur_votes, 0, num_data*sizeof(short));

		for (int i=0; i<num_for_estimate; i++)
		{
			cur_inti_sub_set_indexs[i] = shuffled_indices[i];
			cur_votes[shuffled_indices[i]] = 1;
		}
		maybe_inliers_num = num_for_estimate;


		//�鿴�Ƿ��Ѿ��ù�����Ӽ�
		std::pair< std::set<int *, SubSetIndexComparator >::iterator, bool > res = chosen_sub_sets.insert(cur_inti_sub_set_indexs);

		if (res.second)//true,��ʾ����ɹ�����һ���õ�����Ӽ�
		{
			vector<Point2D*> exactEstimateData;
			for (int i=0; i<num_for_estimate; i++)
			{
				exactEstimateData.push_back(&(data[cur_inti_sub_set_indexs[i]]));
			}
			//��������õ�ֱ�߷���
			param_estimator->estimate(exactEstimateData,maybe_model);

			//�ж�ʣ��ĵ��Ƿ����ģ��
			for(int i=0; i<num_data; i++)
			{
				if(0 == cur_votes[i] && 
					param_estimator->agree(maybe_model, data[i]))
				{
					cur_votes[i] = 1;
					maybe_inliers_num++;					
				}
			}
			//��֮ǰ���ã�
			if (maybe_inliers_num > best_model_num)
			{
				best_model_num = maybe_inliers_num;
				memcpy(best_votes, cur_votes, num_data*sizeof(short));
			}

			//���¼���k, k=log(1-p)/log(1-pow(w,n))
			double w = static_cast<double> (best_model_num) * one_over_indices;
			double p_no_outliers = 1.0 - std::pow(w, static_cast<double> (maybe_inliers_num));

			p_no_outliers = (std::max) (std::numeric_limits<double>::epsilon (), p_no_outliers);       // Avoid division by -Inf
			p_no_outliers = (std::min) (1.0 - std::numeric_limits<double>::epsilon (), p_no_outliers);   // Avoid division by 0.
			k = log_probability / log(p_no_outliers);
			
		}
		else
		{
			delete [] cur_inti_sub_set_indexs;
			--iterations;	//��ε���������
		}

		++iterations;
	}

	//����
	std::set<int *, SubSetIndexComparator >::iterator it = chosen_sub_sets.begin();
	std::set<int *, SubSetIndexComparator >::iterator chosenSubSetsEnd = chosen_sub_sets.end();
	while(it!=chosenSubSetsEnd) {
		delete [] (*it);
		it++;
	}
	chosen_sub_sets.clear();


	//���ҵ��ĵ㼯������С���˷����¹���ģ�Ͳ���
	std::vector<Point2D*> leastSquaresEstimateData;
	for(int j=0; j<num_data; j++) {
		if(best_votes[j])
			leastSquaresEstimateData.push_back(&(data[j]));
	}
	param_estimator->leastSquaresEstimate(leastSquaresEstimateData,model_parameters);

	delete []best_votes;
	delete []cur_votes;


	return (double)best_model_num/(double)num_data;
}