#pragma once

#include "bounding_sphere.hpp"
#include "kd_tree.hpp"

namespace Discregrid
{

class TriangleMeshBSH : public KDTree<BoundingSphere>
{

public:

	using super = KDTree<BoundingSphere>;

	TriangleMeshBSH(std::vector<Eigen::Vector3f> const& vertices,
		std::vector<std::array<unsigned int, 3>> const& faces);

	Eigen::Vector3f const& entityPosition(unsigned int i) const final;
	void computeHull(unsigned int b, unsigned int n, BoundingSphere& hull) const final;

private:

	std::vector<Eigen::Vector3f> const& m_vertices;
	std::vector<std::array<unsigned int, 3>> const& m_faces;

	std::vector<Eigen::Vector3f> m_tri_centers;
};

class TriangleMeshBBH : public KDTree<Eigen::AlignedBox3f>
{
public:

	using super = KDTree<Eigen::AlignedBox3f>;

	TriangleMeshBBH(std::vector<Eigen::Vector3f> const& vertices,
		std::vector<std::array<unsigned int, 3>> const& faces);

	Eigen::Vector3f const& entityPosition(unsigned int i) const final;
	void computeHull(unsigned int b, unsigned int n, Eigen::AlignedBox3f& hull) const final;

private:

	std::vector<Eigen::Vector3f> const& m_vertices;
	std::vector<std::array<unsigned int, 3>> const& m_faces;

	std::vector<Eigen::Vector3f> m_tri_centers;


};

class PointCloudBSH : public KDTree<BoundingSphere>
{

public:

	using super = KDTree<BoundingSphere>;

	PointCloudBSH();
	PointCloudBSH(std::vector<Eigen::Vector3f> const& vertices);

	Eigen::Vector3f const& entityPosition(unsigned int i) const final;
	void computeHull(unsigned int b, unsigned int n, BoundingSphere& hull)
		const final;

private:

	std::vector<Eigen::Vector3f> const* m_vertices;
};

}
