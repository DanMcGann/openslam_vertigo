/*
 * switchVariableLinear.h
 *
 *  Created on: 02.08.2012
 *      Author: niko
 *
 *  Updated Feb 2022 - Dan McGann
 */
#pragma once
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>

namespace vertigo {

/**
 * SwitchVariableLinear is a wrapper around double to allow it to be a Lie type
 */
struct SwitchVariableLinear {
  /** TYPES **/
  enum { dimension = 1 };

  /** default constructor */
  SwitchVariableLinear() : d_(0.0){};

  /** wrap a double */
  SwitchVariableLinear(double d)
      : d_(d){
            // if (d_ < 0.0) d_=0.0;
            // else if(d_>1.0) d_=1.0;
        };

  /** access the underlying value */
  double value() const { return d_; }

  /** print @param s optional string naming the object */
  inline void print(const std::string& name = "") const { std::cout << name << ": " << d_ << std::endl; }

  /** equality up to tolerance */
  inline bool equals(const SwitchVariableLinear& expected, double tol = 1e-5) const {
    return fabs(expected.d_ - d_) <= tol;
  }

  // Manifold requirements

  /** Returns dimensionality of the tangent space */
  inline size_t dim() const { return 1; }
  inline static size_t Dim() { return 1; }

  /** Update the SwitchVariableLinear with a tangent space update */
  inline SwitchVariableLinear retract(const gtsam::Vector& v, gtsam::OptionalJacobian<1, 1> Horigin = boost::none,
                                      gtsam::OptionalJacobian<1, 1> Hv = boost::none) const {
    double x = value() + v(0);
    if (x > 1.0) {
      x = 1.0;
    } else if (x < 0.0) {
      x = 0.0;
    }

    if (Horigin) *Horigin = gtsam::Matrix::Identity(1, 1);
    if (Hv) *Hv = gtsam::Matrix::Identity(1, 1);

    return SwitchVariableLinear(x);
  }

  /** @return the local coordinates of another object */
  inline gtsam::Vector localCoordinates(const SwitchVariableLinear& t2,
                                        gtsam::OptionalJacobian<1, 1> Horigin = boost::none,
                                        gtsam::OptionalJacobian<1, 1> Hv = boost::none) const {
    if (Horigin) *Horigin = -gtsam::Matrix::Identity(1, 1);
    if (Hv) *Hv = gtsam::Matrix::Identity(1, 1);
    return (gtsam::Vector(1) << (t2.value() - value())).finished();
  }

  // Group requirements

  /** identity */
  inline static SwitchVariableLinear identity() { return SwitchVariableLinear(); }

  /** compose with another object */
  inline SwitchVariableLinear compose(const SwitchVariableLinear& p) const { return SwitchVariableLinear(d_ + p.d_); }

  /** between operation */
  inline SwitchVariableLinear between(const SwitchVariableLinear& l2, gtsam::OptionalJacobian<1, 1> H1 = boost::none,
                                      gtsam::OptionalJacobian<1, 1> H2 = boost::none) const {
    if (H1) *H1 = -gtsam::Matrix::Identity(1, 1);
    if (H2) *H2 = gtsam::Matrix::Identity(1, 1);
    return SwitchVariableLinear(l2.value() - value());
  }

  /** invert the object and yield a new one */
  inline SwitchVariableLinear inverse() const { return SwitchVariableLinear(-1.0 * value()); }

  // Lie functions

  /** Expmap around identity */
  static inline SwitchVariableLinear Expmap(const Vector& v, gtsam::OptionalJacobian<1, 1> H = boost::none) {
    if (H) *H = gtsam::Matrix::Identity(1, 1);  // TODO (dan) is this correct
    return SwitchVariableLinear(v(0));
  }

  /** Logmap around identity - just returns with default cast back */
  static inline Vector Logmap(const SwitchVariableLinear& p, gtsam::OptionalJacobian<1, 1> H = boost::none) {
    if (H) *H = gtsam::Matrix::Identity(1, 1);  // TODO (dan) is this correct
    return (gtsam::Vector(1) << p.value()).finished();
  }

 private:
  double d_;
};
}  // namespace vertigo

namespace gtsam {
// Define GTSAM traits
template <>
struct traits<vertigo::SwitchVariableLinear> : public gtsam::internal::LieGroup<vertigo::SwitchVariableLinear> {};

template <>
struct traits<const vertigo::SwitchVariableLinear> : public gtsam::internal::LieGroup<vertigo::SwitchVariableLinear> {};
}  // namespace gtsam