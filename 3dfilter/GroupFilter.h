#pragma once

class Group
{
public:
	int count;

	static bool less(Group *g1, Group *g2) { return g1->count < g2->count; }

	Group(int n) { count = n; }
};

template<typename PointT>
class GroupFilter : public pcl::FilterIndices<PointT>
{
protected:
	typedef typename pcl::FilterIndices<PointT>::PointCloud PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;
	typedef typename pcl::search::Search<PointT>::Ptr SearcherPtr;

private:
	double radius = 0.1;
	int minCount = 200;
	std::vector<Group*> groups;

public:
	GroupFilter(bool extract_removed_indices = false) :
		FilterIndices<PointT>::FilterIndices(extract_removed_indices) {}
	virtual ~GroupFilter()
	{
		clearGroups();
	}

	double getRadius() const { return radius; }
	void setRadius(double newValue) { radius = newValue; }

	int getMinCount() const { return minCount; }
	void setMinCount(int newValue) { minCount = newValue; }

	void perform(std::vector<int> &indices)
	{
		applyFilter(indices);
	}

	std::vector<Group*> getGroups()
	{
		std::vector<Group*> result(groups);
		std::sort(result.begin(), result.end(), Group::less);
		return result;
	}
protected:
	using PCLBase<PointT>::input_;

	virtual void applyFilter(std::vector<int> &indices)
	{
		std::vector<Group*> pointGroups;
		segregate(pointGroups);
	}

	virtual void applyFilter(PointCloud &output)
	{
		std::vector<int> indices;
		applyFilter(indices);
	}

	void segregate(std::vector<Group*> &pointGroups)
	{
		double r2 = radius*radius;
		pointGroups.resize(input_->points.size());
		for (int i = 0; i < input_->points.size(); ++i)
		{
			const PointT &pt = input_->points[i];
			Group *g = newGroup(1);
			pointGroups[i] = g;
			for (int j = 0; j < i; ++j)
			{
				Group *g2 = pointGroups[j];
				if (g2 != g && dist2(pt, input_->points[j]) < r2)
				{
					std::replace(pointGroups.begin(), pointGroups.begin() + i,
						g2, g);
					g->count += g2->count;
					removeGroup(g2);
				}
			}
		}
	}

	int dist2(const PointT &p1, const PointT &p2) const
	{
		double dx = p2.x - p1.x;
		double dy = p2.y - p1.y;
		double dz = p2.z - p1.z;
		return dx*dx + dy*dy + dz*dz;
	}
private:
	void clearGroups()
	{
		for (std::vector<Group*>::iterator it = groups.begin(); it != groups.end(); ++it)
		{
			delete *it;
		}
		groups.clear();
	}

	Group *newGroup(int count)
	{
		Group *result = new Group(count);
		groups.push_back(result);
		return result;
	}

	void removeGroup(Group *g)
	{
		for (std::vector<Group*>::iterator it = groups.begin(); it < groups.end();)
		{
			if (*it == g)
			{
				it = groups.erase(it, it+1);
			}
			else
			{
				++it;
			}
		}
		delete g;
	}
};
