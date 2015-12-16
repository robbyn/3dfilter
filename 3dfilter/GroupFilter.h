#pragma once

class Group
{
public:
	int count;

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

protected:
	virtual void applyFilter(std::vector<int> &indices) {}
	virtual void applyFilter(PointCloud &output) {}

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
		for (std::vector<Group*>::iterator it = groups.begin(); it < groups.end(); ++it)
		{
			if (*it == g)
			{
				groups.erase(it, it.next());
			}
		}
	}
};
