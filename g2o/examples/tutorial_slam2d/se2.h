// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_TUTORIAL_SE2_H
#define G2O_TUTORIAL_SE2_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "g2o/stuff/macros.h"
#include "g2o/stuff/misc.h"
#include "g2o_tutorial_slam2d_api.h"

namespace g2o {

namespace tutorial {

// class that reprepresents elements of the SE2 group
class G2O_TUTORIAL_SLAM2D_API SE2 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // constructor: initialize class as point in origin at 0 deg.
  SE2() : _R(0), _t(0, 0) {}

  SE2(double x, double y, double theta) : _R(theta), _t(x, y) {}

  // get translation vector
  const Eigen::Vector2d& translation() const { return _t; }

  Eigen::Vector2d& translation() { return _t; }

  // get rotation matrix
  const Eigen::Rotation2Dd& rotation() const { return _R; }

  Eigen::Rotation2Dd& rotation() { return _R; }

  // motion composition operator
  SE2 operator*(const SE2& tr2) const {
    SE2 result(*this);
    result._t += _R * tr2._t;
    result._R.angle() += tr2._R.angle();
    result._R.angle() = normalize_theta(result._R.angle());
    return result;
  }

  SE2& operator*=(const SE2& tr2) {
    _t += _R * tr2._t;
    _R.angle() += tr2._R.angle();
    _R.angle() = normalize_theta(_R.angle());
    return *this;
  }

  // transform 2D points
  Eigen::Vector2d operator*(const Eigen::Vector2d& v) const {
    return _t + _R * v;
  }

  // get the inverse of a transformation
  SE2 inverse() const {
    SE2 ret;
    ret._R = _R.inverse();
    ret._R.angle() = normalize_theta(ret._R.angle());
    ret._t = ret._R * (Eigen::Vector2d(-1 * _t));
    return ret;
  }

  double operator[](int i) const {
    assert(i >= 0 && i < 3);
    if (i < 2) return _t(i);
    return _R.angle();
  }

  double& operator[](int i) {
    assert(i >= 0 && i < 3);
    if (i < 2) return _t(i);
    return _R.angle();
  }

  // convert elements from minimal representation (Vector3d) to SE2
  void fromVector(const Eigen::Vector3d& v) { *this = SE2(v[0], v[1], v[2]); }

  // convert elements from SE2 to minimal representation (Vector3d)
  Eigen::Vector3d toVector() const {
    Eigen::Vector3d ret;
    for (int i = 0; i < 3; i++) {
      ret(i) = (*this)[i];
    }
    return ret;
  }

 protected:
  // rotation matrix
  Eigen::Rotation2Dd _R;
  // translation veector
  Eigen::Vector2d _t;
};

}  // namespace tutorial
}  // namespace g2o

#endif
