/* ----------------------------------------------------------------------------
 * Copyright 2022, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include "rmader_types.hpp"

void CPs2TrajAndPwp(std::vector<Eigen::Vector3d> &q, std::vector<mt::state> &traj, mt::PieceWisePol &solution_, int N,
                    int p, int num_pol, Eigen::RowVectorXd &knots, double dc);

Eigen::Spline3d findInterpolatingBsplineNormalized(const std::vector<double> &times,
                                                   const std::vector<Eigen::Vector3d> &positions);