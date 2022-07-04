#ifndef ADTREE_CYLINDER_H
#define ADTREE_CYLINDER_H

/*
*	Copyright (C) 2022 by
*       Noortje van der Horst (noortje.v.d.horst1@gmail.com)
*       Liangliang Nan (liangliang.nan@gmail.com)
*       3D Geoinformation, TU Delft, https://3d.bk.tudelft.nl
*
*	This file is part of GTree, which implements the 3D tree
*   reconstruction and growth modelling method described in the following thesis:
*   -------------------------------------------------------------------------------------
*       Noortje van der Horst (2022).
*       Procedural Modelling of Tree Growth Using Multi-temporal Point Clouds.
*       Delft University of Technology.
*       URL: http://resolver.tudelft.nl/uuid:d284c33a-7297-4509-81e1-e183ed6cca3c
*   -------------------------------------------------------------------------------------
*   Please consider citing the above thesis if you use the code/program (or part of it).
*
*   GTree is based on the works of Easy3D and AdTree:
*   - Easy3D: Nan, L. (2021).
*       Easy3D: a lightweight, easy-to-use, and efficient C++ library for processing and rendering 3D data.
*       Journal of Open Source Software, 6(64), 3255.
*   - AdTree: Du, S., Lindenbergh, R., Ledoux, H., Stoter, J., & Nan, L. (2019).
*       AdTree: accurate, detailed, and automatic modelling of laser-scanned trees.
*       Remote Sensing, 11(18), 2074.
*
*	GTree is free software; you can redistribute it and/or modify
*	it under the terms of the GNU General Public License Version 3
*	as published by the Free Software Foundation.
*
*	GTree is distributed in the hope that it will be useful,
*	but WITHOUT ANY WARRANTY; without even the implied warranty of
*	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*	GNU General Public License for more details.
*
*	You should have received a copy of the GNU General Public License
*	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/


#include <stdexcept>
#include <utility>
#include <ostream>
#include <istream>
#include <vector>
#include <3rd_party/kd_tree/Vector3D.h>
#include <stdio.h>
#include <3rd_party/optimizer_lm/optimizer_lm.h>


class Cylinder
{
public:
	inline Cylinder();
	inline ~Cylinder();

	//Initialize the cylinder
	inline Cylinder(const Vector3D &axisPos1, const Vector3D &axisPos2, float radius);

	//Conduct leas squares
	template< class IteratorT >
	bool LeastSquaresFit(IteratorT begin, IteratorT end);

	//Get fitted attributes
	inline Vector3D GetAxisPosition1() { return m_axisPos1; }
	inline Vector3D GetAxisPosition2() { return m_axisPos2; }
	inline double GetRadius() { return m_radius; }
	inline void SetAxisPosition1(Vector3D pos) { m_axisPos1 = pos; }
	inline void SetAxisPosition2(Vector3D pos) { m_axisPos2 = pos; }
	inline void SetRadius(double r) { m_radius = r; }

private:
	Vector3D m_axisPos1;
	Vector3D m_axisPos2;
	double m_radius;
};

Cylinder::Cylinder()
{

};

Cylinder::~Cylinder()
{

};

Cylinder::Cylinder(const Vector3D &axisPos1, const Vector3D &axisPos2, float radius)
{
	m_axisPos1 = axisPos1;
	m_axisPos2 = axisPos2;
	m_radius = radius;
};


inline void evaluate_cylinder(double* data, int m, int n, double* var, double* fvec, int iflag)
{
	//Compute the objective function
	Vector3D p1(var[0], var[1], var[2]);
	Vector3D p2(var[3], var[4], var[5]);
	double r = var[6];
	for (int i = 0; i < m; ++i)
	{
		Vector3D p(data[i], data[i + m], data[i + m * 2]);
		Vector3D ptemp = Vector3D::crossProduct(p - p1, p - p2);
		double d = ptemp.normalize() / ((p2 - p1).normalize());
		fvec[i] = (d - r)*data[i + m * 3];
	}
}



template< class IteratorT >
bool Cylinder::LeastSquaresFit(IteratorT begin, IteratorT end)
{
	int m = end - begin;
	int n = 7;
	// assign value to 7 parameters
	double param[7];
	for (size_t i = 0; i < 3; ++i)
		param[i] = m_axisPos1[i];
	for (size_t i = 0; i < 3; ++i)
		param[i + 3] = m_axisPos2[i];
	param[6] = m_radius;

	//assign value to data
	double *data = nullptr;
	int size = 4 * m;
	data = new double[size];
	int index = 0;
	std::vector<double> ptemp;
	for (IteratorT pIter = begin; pIter != end; pIter++)
	{
		ptemp = *pIter;
		data[index] = ptemp[0];
		data[index + m] = ptemp[1];
		data[index + 2*m] = ptemp[2];
		data[index + 3*m] = ptemp[3];
		index++;
	}
	//Create the optimizer
	Optimizer_LM lmo;
	bool result = lmo.optimize(m, n, param, (Optimizer_LM::lm_evaluate_func*)evaluate_cylinder, data);
	if(!result)
		return false;
	for(size_t i = 0; i < 3; ++i)
		m_axisPos1[i] = param[i];
	for(size_t i = 0; i < 3; ++i)
		m_axisPos2[i] = param[i + 3];
	m_radius = param[6];
	delete[](data);
	return true;
}

#endif
