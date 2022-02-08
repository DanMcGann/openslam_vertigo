/*
 * betweenFactorSwitchable.h
 *
 *  Created on: 02.08.2012
 *      Author: niko
 *
 *  Updated Feb 2022 Dan McGann
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

#include "vertigo/SwitchVariableLinear.h"

namespace vertigo {

template <class VALUE>
class BetweenFactorSwitchableLinear : public gtsam::NoiseModelFactor3<VALUE, VALUE, SwitchVariableLinear> {
 public:
  BetweenFactorSwitchableLinear(){};
  BetweenFactorSwitchableLinear(gtsam::Key key1, gtsam::Key key2, gtsam::Key key3, const VALUE& measured,
                                const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor3<VALUE, VALUE, SwitchVariableLinear>(model, key1, key2, key3),
        betweenFactor(key1, key2, measured, model){};

  gtsam::Vector evaluateError(const VALUE& p1, const VALUE& p2, const SwitchVariableLinear& s,
                              boost::optional<gtsam::Matrix&> H1 = boost::none,
                              boost::optional<gtsam::Matrix&> H2 = boost::none,
                              boost::optional<gtsam::Matrix&> H3 = boost::none) const {
    // calculate error
    gtsam::Vector error = betweenFactor.evaluateError(p1, p2, H1, H2);
    error *= s.value();

    // handle derivatives
    if (H1) *H1 = *H1 * s.value();
    if (H2) *H2 = *H2 * s.value();
    if (H3) *H3 = error;

    return error;
  };

 private:
  gtsam::BetweenFactor<VALUE> betweenFactor;
};

}  // namespace vertigo
