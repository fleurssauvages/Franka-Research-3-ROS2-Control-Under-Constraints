#include <cassert>
#include <cmath>
#include <iostream>

#include "low_level/residual_torque_compensator.hpp"

using Comp = low_level::ResidualTorqueCompensator;
using A = Comp::Joints;

bool near(double a, double b, double tolerance = 1.e-6) {
  return std::abs(a - b) <= tolerance;
}

int main() {
  Comp::Settings settings;
  settings.minimum_sample_s = 0.5;
  settings.stationary_dwell_s = 0.5;
  settings.filter_tau_s = 0.1;
  settings.output_slew_rate = 0.1;
  settings.torque_limits.fill(0.15);
  Comp comp(settings);
  A q{}, dq{}, zero{}, measured{}, gravity{}, last{};
  gravity.fill(4.0);
  measured.fill(4.10); // unloaded, stationary, model residual +0.10 Nm

  // Cannot learn just by being stationary: explicit authorization is mandatory.
  for (int k = 0; k < 2000; ++k)
    comp.update(.001, true, true, zero, q, dq, measured, gravity, last, false);
  assert(!comp.calibrated());
  assert(near(comp.applied()[0], 0.0));

  // Cannot learn while a command is applying torque.
  last.fill(.06);
  for (int k = 0; k < 2000; ++k)
    comp.update(.001, true, true, zero, q, dq, measured, gravity, last, true);
  assert(!comp.collecting());
  last.fill(0.0);
  for (int k = 0; k < 2000; ++k)
    comp.update(.001, true, true, zero, q, dq, measured, gravity, last, true);
  assert(comp.collecting());
  assert(!comp.calibrated()); // not applied during sampling
  assert(near(comp.applied()[0], 0.0));
  comp.update(.001, true, true, zero, q, dq, measured, gravity, last, false);
  assert(comp.calibrated());
  assert(near(comp.estimate()[0], .10, .001));

  for (int k = 0; k < 1100; ++k)
    comp.update(.001, true, true, zero, q, dq, measured, gravity, last, false);
  assert(near(comp.applied()[0], .10, .001));

  // Learned estimate freezes as soon as the human starts moving a joint.
  dq[0] = .2;
  measured.fill(4.13);
  for (int k = 0; k < 500; ++k)
    comp.update(.001, true, true, zero, q, dq, measured, gravity, last, false);
  assert(near(comp.estimate()[0], .10, .001));

  // Fade a local bias out when moving far from the calibration pose.
  q[0] = .375; // halfway through 0.25 to 0.50 rad fade
  dq[0] = 0;
  for (int k = 0; k < 1000; ++k)
    comp.update(.001, true, true, zero, q, dq, measured, gravity, last, false);
  assert(near(comp.applied()[0], .05, .001));
  q[0] = .60;
  for (int k = 0; k < 1000; ++k)
    comp.update(.001, true, true, zero, q, dq, measured, gravity, last, false);
  assert(near(comp.applied()[0], 0., .001));

  // Command timeout ramps output to zero. Never learns on timeout.
  q.fill(0.0);
  for (int k = 0; k < 1000; ++k)
    comp.update(.001, true, true, zero, q, dq, measured, gravity, last, false);
  assert(comp.applied()[0] > 0.09);
  for (int k = 0; k < 2000; ++k)
    comp.update(.001, false, true, zero, q, dq, measured, gravity, last, true);
  assert(near(comp.applied()[0], 0.0));

  // Reject residuals above per-joint torque limits.
  comp.reset();
  measured.fill(4.25);
  for (int k = 0; k < 2000; ++k)
    comp.update(.001, true, true, zero, q, dq, measured, gravity, last, true);
  assert(!comp.collecting());
  comp.update(.001, true, true, zero, q, dq, measured, gravity, last, false);
  assert(!comp.calibrated());

  std::cout << "Residual compensator tests passed\n";
  return 0;
}
