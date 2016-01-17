#ifndef PCL_FILTERS_GROUPFILTER_H
#define PCL_FILTERS_GROUPFILTER_H

#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace pcl
{
	class Node
	{
	private:
		int size = 1;
		Node *group = this;
		Node *next = this;

		void setGroup(Node *g);
	public:
		int groupSize() const { return group->size; }
		void merge(Node &other);
	};

	template<typename PointT>
	class GroupFilter : public pcl::Filter<PointT>
	{
		using Filter<PointT>::input_;
		typedef typename Filter<PointT>::PointCloud PointCloud;
		typedef typename pcl::KdTree<PointT>::Ptr KdTreePtr;

	public:
		GroupFilter(double radius, int min_size) : radius_(radius),
			min_size_(min_size)
		{
		}

		void applyFilter(PointCloud &output);

		void setRadius(double radius) { radius_ = radius; }
		double getRadius() const { return radius_; }
		void setMinSize(int min_size) { min_size_ = min_size; }
		int getMinSize() const { return min_size_; }
	private:
		double radius_;
		int min_size_;
		KdTreePtr tree_;

		void buildGroups(std::vector<Node> &nodes);
	};

	template<typename PointT>
	void pcl::GroupFilter<PointT>::applyFilter(PointCloud &output)
	{
		if (!tree_)
		{
			tree_.reset(new pcl::KdTreeFLANN<PointT>(false));
		}
		tree_->setInputCloud(input_);
		std::vector<Node> nodes(input_->points.size());
		buildGroups(nodes);
		std::vector<int> indices;
		for (int pid = 0; pid < nodes.size(); ++pid)
		{
			if (nodes[pid].groupSize() >= min_size_)
			{
				indices.push_back(pid);
			}
		}
		copyPointCloud(*input_, indices, output);
	}

	template<typename PointT>
	void GroupFilter<PointT>::buildGroups(std::vector<Node> &nodes)
	{
		std::vector<int> indices;
		std::vector<float> dists;
		for (int pid = 0; pid < nodes.size(); ++pid)
		{
			tree_->radiusSearch(pid, radius_, indices, dists);
			for (int i = 0; i < indices.size(); ++i)
			{
				int id = indices[i];
				nodes[pid].merge(nodes[id]);
			}
		}
	}

	void Node::setGroup(Node *g)
	{
		g->size += group->size;
		Node *n = this;
		do {
			n->group = g;
			n = n->next;
		} while (n != this);
	}

	void Node::merge(Node &other)
	{
		if (group != other.group)
		{
			if (groupSize() > other.groupSize())
			{
				other.setGroup(group);
			}
			else
			{
				setGroup(other.group);
			}
			Node *n = next;
			next = other.next;
			other.next = n;
		}
	}
}

#endif
