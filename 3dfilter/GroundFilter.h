#ifndef PCL_FILTERS_GROUNDFILTER_H
#define PCL_FILTERS_GROUNDFILTER_H

#include <pcl/filters/filter.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <math.h>

namespace pcl
{
	template<typename PointT>
	class GroundFilter : public pcl::Filter<PointT>
	{
		using Filter<PointT>::input_;
		typedef typename Filter<PointT>::PointCloud PointCloud;

	public:
		GroundFilter()
		{
		}

		void applyFilter(PointCloud &output);

		void setThreshold(double threshold) { threshold_ = threshold; }
		int getThreshold() const { return threshold_; }
	private:
		double threshold_;
	};

	template<typename PointT>
	void pcl::GroundFilter<PointT>::applyFilter(PointCloud &output)
	{
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<PointT> seg;
		// Optional
		seg.setOptimizeCoefficients(true);
		// Mandatory
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(threshold_);
		seg.setInputCloud(input_);
		seg.segment(*inliers, *coefficients);

		if (inliers->indices.size() == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
		}
		else
		{
			std::vector<int> removed(inliers->indices);
			std::cerr << "Number of ground points: " << removed.size() << std::endl;
			std::cerr << "Model coefficients: " << coefficients->values[0] << " "
				<< coefficients->values[1] << " "
				<< coefficients->values[2] << " "
				<< coefficients->values[3] << std::endl;
			size_t size = input_->points.size();
			std::vector<int> indices;
			for (int i = 0; i < size; ++i)
			{
				if (!binary_search(removed.begin(), removed.end(), i))
				{
					indices.push_back(i);
				}
			}
			copyPointCloud(*input_, indices, output);
		}
	}
}

#endif
