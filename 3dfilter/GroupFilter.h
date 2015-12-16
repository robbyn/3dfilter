#pragma once

class Group
{
public:
	int count;

	Group(int n) { count = n; }
};

template<class PointT>
class GroupFilter
{
private:
	double radius = 0.1;
	int minCount = 200;
	pcl::PointCloud<typename PointT> cloud;
	std::vector<Group*> groups;

public:
	GroupFilter() {}
	~GroupFilter()
	{
		clearGroups();
	}

	double getRadius() const { return radius; }
	void setRadius(double newValue) { radius = newValue; }

	int getMinCount() const { return minCount; }
	void setMinCount(int newValue) { minCount = newValue; }
	void setInputCloud(pcl::PointCloud<PointT>::Ptr newValue) { cloud = newValue; }

private:
	void clearGroups()
	{
		for (std::iterator<Group*> it = groups.begin(); it < groups.end(); ++it)
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
		for (std::iterator<Group*> it = groups.begin(); it < groups.end(); ++it)
		{
			if (*it == g)
			{
				groups.erase(it, it.next());
			}
		}
	}
};
