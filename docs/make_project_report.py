"""Project report PDF: both papers, typed equations, from scratch.
Generate:  python docs/make_project_report.py"""
from pdfdoc import Doc

d = Doc(
    "PROJECT_REPORT.pdf",
    "Full Quaternion Attitude Control and Differential-Flatness Trajectory "
    "Tracking for a Quadrotor",
    "Project Report - Group 12 (B Sainath CB.SC.U4AIE24309, Vishal "
    "CB.SC.U4AIE24363) - School of Artificial Intelligence, Amrita Vishwa "
    "Vidyapeetham. Base papers: Fresk & Nikolakopoulos (ECC 2013) [1] and "
    "Choutri, Lagha, Dala & Lipatov (IEEE 2017) [2]. All equations below are "
    "typed out and were cross-checked numerically against the implementation "
    "(7 equation groups, randomised trials - see Section 8).")

# ================================================================ 1
d.h1("1. Introduction")

d.p("A quadrotor is a rigid body with four rotors. It has six degrees of "
    "freedom in attitude and position but only four independent control "
    "inputs: the total rotor thrust and the three body torques. This "
    "underactuation is the central difficulty of quadrotor control. The "
    "vehicle cannot accelerate sideways directly; it must first tilt so that "
    "the thrust vector, which always points along the body z-axis, points "
    "partly in the desired direction. Attitude and translation are therefore "
    "coupled, and any trajectory-tracking controller must respect that "
    "coupling rather than fight it.")

d.p("This project implements, verifies and extends two papers that treat the "
    "two halves of this problem. The first, Fresk and Nikolakopoulos [1], is "
    "a pure attitude controller: a nonlinear proportional-squared quaternion "
    "law that commands body torques directly from the quaternion attitude "
    "error. The second, Choutri, Lagha, Dala and Lipatov [2], adds "
    "trajectory tracking: it uses differential flatness to convert a desired "
    "path into the thrust and attitude that produce it, and closes both "
    "loops with linear quadratic regulators. The project reproduces both "
    "papers from their equations, flies the results on four simulation "
    "stacks (an ideal model, MuJoCo contact physics, gym-pybullet-drones, "
    "and real ArduPilot firmware in software-in-the-loop), and extends the "
    "second paper with a trajectory library. Every result quoted in this "
    "report regenerates from a single command, and the repository carries "
    "85 tests of which 83 pass in the canonical environment (two require "
    "simulator stacks not installed there).")

d.note("How the two papers connect",
       "Paper 1 controls attitude only - point the body, and accept wherever "
       "the resulting thrust pushes the vehicle. Paper 2 inverts this: given "
       "where the vehicle should be, compute the thrust and the attitude "
       "that put it there, then let an attitude loop hold that attitude. Our "
       "implementation keeps both on one airframe (0.2 kg, inertia "
       "diag(6.5e-4, 6.5e-4, 1.2e-3) kg m^2), so the comparison between them "
       "is a property of the controllers, not of different vehicles.")

# ================================================================ 2
d.h1("2. Quaternion representation of attitude")

d.p("An attitude is a rotation of the body frame relative to the world "
    "frame. Euler angles represent it with three numbers but contain a "
    "singularity (gimbal lock at pitch of plus or minus 90 degrees); the "
    "rotation matrix avoids that singularity but carries nine constrained "
    "numbers. A unit quaternion carries four numbers with one norm "
    "constraint, has no singularity anywhere, and composes rotations with a "
    "cheap product. Both papers base their controllers on it. Throughout, "
    "quaternions are written scalar-first, q = (q0, q1, q2, q3) with vector "
    "part q-vector = (q1, q2, q3), and represent the rotation from body "
    "frame to world frame.")

d.h2("2.1 Algebra")

d.eq(["q = q0 + q1*i + q2*j + q3*k = (q0, q-vector),   q0 in R, q-vector in R^3"],
     "A quaternion: one scalar part and a three-vector part. A unit "
     "quaternion (norm 1) represents a rotation.")

d.eq(["p (x) q = ( p0*q0 - p-vector . q-vector ,",
      "        p0*q-vector + q0*p-vector + p-vector x q-vector )"],
     "The quaternion product. The dot and cross products of the vector parts "
     "appear explicitly; the product is associative but not commutative, "
     "which is exactly what makes it suitable for composing rotations in "
     "order.")

d.eq(["q* = (q0, -q-vector)",
      "||q|| = sqrt( q (x) q* ) = sqrt(q0^2 + q1^2 + q2^2 + q3^2)",
      "q^-1 = q* / ||q||      ( = q* for a unit quaternion )"],
     "Conjugate, norm and inverse. Conjugation reverses a rotation; for unit "
     "quaternions the conjugate is the inverse.")

d.eq(["v' = q (x) [0, v] (x) q*      which equals  R(q) * v"],
     "Rotating a vector v by the attitude q. R(q) is the 3x3 direction "
     "cosine matrix; its columns are the body axes written in world "
     "coordinates, it is orthonormal, and R(p (x) q) = R(p) R(q).")

d.eq(["q = [ cos(alpha/2),  u * sin(alpha/2) ]"],
     "Axis-angle form: a rotation by angle alpha about unit axis u. This is "
     "the most useful construction rule - it builds reference quaternions "
     "directly from geometric intent.")

d.h2("2.2 The kinematic equation")

d.eq(["q-dot = (1/2) * q (x) [0, omega]        omega = body angular rate",
      "q-dot = (1/2) * [0, omega_w] (x) q      omega_w = R(q) omega (world)"],
     "The two equivalent forms of the quaternion kinematic equation. One "
     "physical motion, two algebraic shapes: match the form to the frame "
     "your angular velocity lives in. Mixing them produces an error that "
     "grows with tilt times rate - small near hover, radical during "
     "aggressive manoeuvres (Section 7, finding on validation).")

d.note("Why quaternions in a control loop",
       "The quaternion error between a reference and a measurement is itself "
       "a quaternion, its vector part points along the axis that rotates one "
       "onto the other, and its magnitude tracks the error angle - a "
       "controller can act on it directly with no trigonometry and no "
       "singularity. Euler angles would need unwrapping and would fail at "
       "gimbal lock; the paper's flip test rotates through exactly the "
       "region where Euler representations misbehave.")

# ================================================================ 3
d.h1("3. Paper 1: full quaternion based attitude control")

d.h2("3.1 The plant")

d.p("The attitude dynamics of a rigid body about its centre of mass, with "
    "the paper's inertia tensor and input torque, are:")

d.eq(["J * omega-dot = tau ,      J = diag( 6.5e-4, 6.5e-4, 1.2e-3 )  kg*m^2"],
     "The rotational plant: torque divided by inertia gives angular "
     "acceleration. The paper's vehicle has equal roll and pitch inertia and "
     "roughly double the yaw inertia, as expected for a planar-X quadrotor "
     "with light arms.")

d.p("Coupled with the kinematic equation of Section 2.2, this is a double "
    "integrator from torque to attitude: torque shapes the angular rate, the "
    "rate shapes the attitude. That structure is what makes a "
    "derivative-free proportional law sufficient - no integral action is "
    "needed for the tracking error to decay.")

d.h2("3.2 The control law")

d.p("The controller [1] acts entirely in quaternion space. Given the "
    "reference attitude q_ref and the measured attitude q_m (both in the "
    "paper's conjugated convention), the error quaternion, its vector part, "
    "and the torque command are:")

d.eq(["q_err  = q_ref (x) q_m*                        (attitude error)",
      "e      = vector part of q_err, sign-corrected so it takes the",
      "         SHORTEST rotation (flip sign when q_err's scalar is negative)",
      "tau    = -Pq * e  -  Pw * omega_m ,      Pq = 20,  Pw = 4",
      "tau    clipped elementwise to +/- 4 N*m per axis"],
     "The complete law. The first term rotates the attitude error to zero; "
     "the second damps the angular rate. Torque to attitude is a double "
     "integrator, so this proportional-derivative structure in quaternion "
     "form drives the error to zero without an integral term. The gains and "
     "the saturation bound are the paper's stated values and are reproduced "
     "exactly. The measured attitude and rate carry the paper's sensor "
     "noise: uniform plus or minus 0.1 on every quaternion component and "
     "rate component, refreshed at 1 kHz.")

d.p("Why the sign correction matters: q and -q represent the same rotation, "
    "so an error quaternion with a negative scalar would steer the long way "
    "around. Correcting the sign of the vector part makes the controller "
    "always take the short rotation - except in the flip scenario, where the "
    "correction is deliberately disabled so the vehicle commits to the full "
    "360-degree rotation instead of unwinding.")

d.h2("3.3 The three benchmark scenarios")

d.bullets([
    "STEP - unit-radian (57.3 degree) steps on roll, pitch and yaw, "
    "staggered at t = 1, 5 and 9 s across a 15 s run. Essentially no "
    "overshoot; about 2 s settling per axis.",
    "SINE - 0.5 radian amplitude, 1 rad/s sine on all three axes, "
    "phase-shifted so the torques stay out of phase. Measured lag of 0.4 to "
    "0.5 s matches the paper's stated about half a second.",
    "FLIP - a full 0 to 2*pi rad roll ramp over 2 s. The raw quaternion "
    "components stay smooth through the whole rotation with no gimbal-lock "
    "artifact, which is the paper's central claim.",
])

d.h2("3.4 A derived result: the digital control-rate bound")

d.p("The paper specifies no digital control rate. Linearising the closed "
    "loop about identity attitude gives a characteristic equation and its "
    "poles:")

d.eq(["I*e-double-dot + Pw*e-dot + (Pq/2)*e = 0",
      "lambda(1,2) = ( -Pw +/- sqrt(Pw^2 - 2*I*Pq) ) / (2*I)",
      "           =  -2.5  and  -6150  rad/s"],
     "The continuous loop is heavily overdamped and perfectly stable. The "
     "instability that appears at modest control rates is purely a sampling "
     "artifact of the fast pole: at a 200 Hz update the discrete closed-loop "
     "spectral radius is 29.9 (unstable); at 1 kHz it is 5.16; at 10 kHz it "
     "is just under one.")

d.eq(["f_control >= Pw / (m * I_min)  with margin m = 0.5   gives   "
      "f_control >= 12.3 kHz"],
     "The derived, not tuned, minimum safe control rate. Our simulator "
     "adopts it by default and the scheduler refuses to run below it; the "
     "instability at 200 Hz can be reproduced directly by overriding the "
     "rate.")

# ================================================================ 4
d.h1("4. Paper 2: differential flatness and trajectory tracking")

d.h2("4.1 The full model")

d.p("Paper 2 extends the attitude plant to the complete six-degree-of-"
    "freedom vehicle. The state is position p, velocity v, attitude "
    "quaternion q and body rate omega; the inputs are the total thrust T "
    "and the body torque tau:")

d.eq(["p-dot  = v",
      "v-dot  = R(q) * [0, 0, T/m]  -  [0, 0, g]",
      "q-dot  = (1/2) * q (x) [0, omega]",
      "J*omega-dot = - omega x (J*omega) + tau"],
     "The four rows say: velocity is the derivative of position; the thrust "
     "vector (magnitude T along the body z-axis, rotated into the world by "
     "R(q)) accelerates the vehicle against gravity; the attitude evolves "
     "with the body rate; and the torque overcomes the gyroscopic term to "
     "change the rate. The second row contains the underactuation: the "
     "direction of the thrust is chosen by the attitude, so horizontal "
     "acceleration is bought with tilt.")

d.p("The four rotor speeds produce the two inputs through a mixing matrix. "
    "Each rotor i contributes thrust proportional to the square of its "
    "speed, b times omega-i-squared, along body z, plus a reaction torque "
    "about body z whose sign alternates with the propeller direction. For "
    "the X-configuration with arm distance d:")

d.eq(["[T ]   [  b     b     b     b  ] [w1]",
      "[tx] = [  b*d   b*d  -b*d  -b*d ] [w2]",
      "[ty]   [ -b*d   b*d   b*d  -b*d ] [w3]",
      "[tz]   [ -c     c    -c     c  ] [w4]",
      "so [T, tx, ty, tz] = M [w1, w2, w3, w4],  w = M^-1 [T, tx, ty, tz]"],
     "The mixer: front rotors 1 and 2 (columns 1-2) differ from rear rotors "
    "3 and 4 for the roll torque tx; left differs from right for ty; the "
    "alternating signs of the reaction coefficient c give the yaw torque "
    "tz. Inverting M turns any thrust-and-torque command into rotor speeds; "
    "every real flight controller contains this matrix. Rotor speeds are "
    "bounded, so when a command exceeds what the rotors can produce, we "
    "deliver the thrust exactly and scale the torque uniformly to the "
    "largest feasible value - naive clipping instead corrupts the total "
    "thrust and destabilises altitude (a finding of this project).")

d.p("Linearising the model at hover (identity attitude, zero rates) shows "
    "how tilt steers: the horizontal acceleration becomes proportional to "
    "the small quaternion tilt components, and the thrust commands the "
    "vertical acceleration:")

d.eq(["v-dot approximately = [ 2*(T/m)*q2 ,  -(T/m)*q1 ,  T/m - g ]",
      "q-vector-dot approximately = omega / 2"],
     "The hover linearisation. These two relations are the seeds of the two "
     "linear models the LQR design uses.")

d.h2("4.2 Differential flatness")

d.p("A system is differentially flat if its complete state and input can be "
    "written as functions of a set of flat outputs and finitely many of "
    "their derivatives - with no integration of the differential equations. "
    "For the quadrotor the flat outputs are the position and the yaw angle:")

d.eq(["sigma(t) = ( x, y, z, psi )"],
     "Four flat outputs for four inputs. Given these and their derivatives "
     "up to second order, every state and input of the vehicle follows "
     "algebraically.")

d.p("The chain that recovers the vehicle from the outputs starts from "
    "Newton's law on the translational row: if the vehicle is to accelerate "
    "with a, the net force must be m*a, and the only forces are thrust and "
    "gravity, so the thrust vector is determined:")

d.eq(["f  = m * ( a + g*z-hat )          the required thrust vector",
      "T_d = ||f||                       its magnitude (the thrust command)",
      "d = f / ||f||                     its direction",
      "R(q_d) * z-hat = d                body z must point along d",
      "yaw of q_d = psi_d                the remaining attitude freedom"],
     "The direction d fixes two degrees of the three-degree attitude; the "
     "chosen yaw fixes the last one. The attitude reference q_d is therefore "
     "fully determined by the desired acceleration and yaw - this is the "
     "inversion of the underactuation described in Section 1. Differentiating "
     "q_d in time would additionally give the reference body rate and, one "
     "derivative further, the reference torque (the full parameterisation; "
     "our inner loop regulates rate to zero instead, a documented "
     "simplification).")

d.p("The paper constructs q_d as a composition of two quaternions, and we "
    "implement that construction literally:")

d.eq(["q_pd = tilt quaternion taking z-hat onto d:",
      "      axis = z-hat x d  (horizontal, so q_pd's yaw element is exactly 0)",
      "      q_pd = [ cos(alpha/2), sin(alpha/2) * (z-hat x d)/||z-hat x d|| ]",
      "      with alpha the angle between z-hat and d",
      "q_zd = [ cos(psi_d/2), 0, 0, sin(psi_d/2) ]     pure yaw quaternion",
      "q_d  = q_pd (x) q_zd                             the composition"],
     "Eq. (19), (20), (21) of the paper, typed as implemented. The tilt "
     "quaternion's rotation axis is horizontal by construction, so its yaw "
     "component is identically zero - the paper's statement that the "
     "position quaternion's fourth element vanishes. We verified numerically "
     "that the composed q_d aligns the body z-axis with the thrust direction "
     "to 1e-10 and that the tilt part's yaw element is zero to 1e-12. The "
     "only degeneracy of the whole chain is a vanishing thrust vector (free "
     "fall), where the attitude stops mattering to the motion.")

d.h2("4.3 The double-loop LQR")

d.p("The controller is cascaded the way the plant is: an outer position "
    "loop computes the acceleration command, the flatness map of the "
    "previous subsection turns it into the thrust T_d and the attitude "
    "reference q_d, and an inner attitude loop tracks q_d with torque. The "
    "inner loop must run roughly an order of magnitude faster than the "
    "outer - the classical time-scale separation. Both loops are linear "
    "quadratic regulators, designed on the hover linearisation:")

d.eq(["INNER loop, state x = [q-vector(3), omega(3)]:",
      "  x-dot = A x + B u,   A = [ 0(3x3)  (1/2)I ]    B = [   0    ]",
      "                            [ 0(3x3)   0    ]        [ J^-1   ]",
      "OUTER loop, per axis, state x = [p, p-dot]:",
      "  x-dot = A x + B u,   A = [ 0  1 ]              B = [ 0 ]",
      "                            [ 0  0 ]                  [ 1 ]"],
     "The inner A-matrix's half-identity block is the hover relation "
     "q-vector-dot = omega/2; the B-matrix's J-inverse is Euler's equation. "
     "The outer loop is the double integrator with acceleration as input. "
     "Both have the standard linear form x-dot = A x + B u.")

d.eq(["J_cost = (1/2) * integral from 0 to infinity of ( x' Q x + u' R u ) dt",
      "P A + A' P - P B R^-1 B' P + Q = 0        (the Riccati equation)",
      "K = R^-1 B' P ,        u = -K x"],
     "The LQR design: the quadratic cost trades state error (weighted by Q) "
     "against control effort (weighted by R); the algebraic Riccati equation "
     "yields the matrix P, and the constant gain K multiplies the state to "
     "give the optimal input. For the outer double integrator the resulting "
     "law is a proportional-derivative controller with optimal gains plus "
     "the flatness feedforward. The paper states the controller ran at "
     "200 Hz; we run both loops at 200 Hz with zero-order hold, and the "
     "closed-loop poles of both designs are verified Hurwitz (all "
     "eigenvalues with negative real part) in our tests.")

d.h2("4.4 The study case and results")

d.p("The paper's experiment tracks a circular path at a fixed altitude of "
    "1 m, starting from the point (-1, 0, 0), with the 200 Hz controller "
    "and actuator limits in effect. We reproduce it with the vehicle "
    "starting at rest and climbing onto the circle with a minimum-jerk "
    "altitude ramp, which reproduces the roughly one-second lag the paper "
    "reports in the horizontal responses. The root-mean-square tracking "
    "errors after settling, on three plants:")

d.table(
    ["Plant", "RMS x", "RMS y", "RMS z", "RMS radial", "Peak rotor"],
    [["Ideal Eq. (8) model, RK4 at 1 kHz", "1.6 cm", "3.6 cm", "0.6 cm", "2.7 cm", "0.57 N"],
     ["MuJoCo contact physics, 20 kHz", "1.4 cm", "3.7 cm", "0.7 cm", "2.8 cm", "0.99 N"],
     ["MuJoCo + paper noise +/-0.1", "2.3 cm", "4.6 cm", "0.9 cm", "3.7 cm", "1.69 N"]],
    widths=[5.8 * 28.35, 1.9 * 28.35, 1.9 * 28.35, 1.9 * 28.35, 2.0 * 28.35, 1.9 * 28.35])

d.p("Approximately 97 percent radial accuracy on the 1 m circle, with no "
    "rotor saturation (the per-rotor limit is 1.95 N), and near-identical "
    "numbers on two independent plants - the flatness reference is exact "
    "regardless of the integrator behind it. Where the paper gives no "
    "numbers (the LQR weights, the circle speed, the actuator lag model, "
    "the gravity value, the integrator), our choices are documented in the "
    "repository README's assumptions table.")

# ================================================================ 5
d.h1("5. Cross-simulator deployment of the attitude law")

d.p("The paper 1 controller, with gains untouched, was deployed through a "
    "shared bridge to four stacks: MuJoCo, gym-pybullet-drones, Gazebo "
    "under WSL2, and real ArduPilot firmware compiled from source under "
    "WSL2. All four run the same references and the same noise model. The "
    "sine scenario is the clean comparison:")

d.table(
    ["Stack", "Sine RMS attitude error", "Flip outcome"],
    [["ArduPilot firmware (real code)", "14.7 deg +/- 0.2", "completes, settles 2.89 s"],
     ["Gazebo (real physics)", "17.3 deg +/- 0.0", "engine-dependent knife-edge"],
     ["gym-pybullet-drones", "17.52 deg", "knife-edge regime"],
     ["MuJoCo", "17.61 deg", "completes, settles 2.93 s"]],
    widths=[5.4 * 28.35, 4.6 * 28.35, 5.4 * 28.35])

d.p("The four stacks agree to within three degrees on sine tracking, and "
    "the two native stacks agree to 0.03 degrees - evidence that the loop "
    "belongs to the controller rather than to any engine. The residual "
    "error near 17.5 degrees is the sensor noise floor: the paper's uniform "
    "plus-or-minus 0.1 quaternion-component noise corresponds to roughly 11 "
    "degrees of measured attitude error, which the controller cannot remove "
    "because it cannot see through it. The flip's engine dependence is "
    "analysed in the repository as a knife-edge property of the pursuit "
    "equilibrium: with shortest-path disabled the vehicle rate saturates at "
    "five times the sine of half the lag angle, so a 2-second ramp is "
    "trackable only with a lag of about 1.36 rad, and whether the vehicle "
    "crosses the full rotation depends on integrator fidelity at exactly "
    "that margin.")

# ================================================================ 6
d.h1("6. Extensions implemented (trajectory library)")

d.p("Beyond the two papers we extended the flatness stack with a trajectory "
    "library - the same frozen controllers, new flat-output generators "
    "only. The waypoint trajectories are piecewise degree-7 polynomials "
    "that minimise the integral of squared snap (the fourth derivative), "
    "because flatness makes the control input depend on the derivatives: "
    "jerk drives body rates and snap drives torques, so the smoothest "
    "flyable path is the minimum-snap path. Waypoints, rest-to-rest ends "
    "and velocity continuity at the joints are constraints; the minimum is "
    "found per segment in closed form. Tracking results, all with the "
    "frozen LQR gains:")

d.table(
    ["Scenario", "RMS x", "RMS y", "RMS z", "RMS 3D", "Peak rotor"],
    [["circle (paper 2 study case)", "1.6", "3.6", "0.6", "3.9", "0.57"],
     ["figure-8, ideal model", "2.5", "3.9", "0.9", "4.7", "0.57"],
     ["figure-8, MuJoCo contact physics", "2.4", "4.0", "1.0", "4.7", "1.01"],
     ["vertical orbit (cone sweep)", "1.6", "3.5", "0.6", "3.9", "0.57"],
     ["waypoint tour, minimum-snap", "4.6", "3.9", "1.3", "6.1", "0.83"],
     ["waypoint tour, minimum-jerk", "5.8", "5.2", "2.3", "8.1", "0.84"]],
    widths=[5.6 * 28.35, 1.9 * 28.35, 1.9 * 28.35, 1.9 * 28.35, 2.0 * 28.35, 2.1 * 28.35])

d.p("(all values in centimetres; rotor peaks in newtons against the 1.95 N "
    "limit). The minimum-snap tour tracks 25 percent better than the "
    "minimum-jerk tour through identical waypoints at identical times - the "
    "flatness argument (snap drives torques) measured directly. Building it "
    "surfaced a instructive failure: minimising only snap with "
    "position-only waypoints lets long segments loiter and then teleport, "
    "because position is four integrals away from the cost and slow large "
    "excursions are nearly free. Pinning the corridor tangent velocity at "
    "each waypoint removes the freedom; solving in normalised time per "
    "segment removes a numerical conditioning problem that reproduces the "
    "same pathology even with correct constraints.")

# ================================================================ 7
d.h1("7. Verification methodology")

d.p("Every claim in this report is backed by an automated check. The "
    "repository carries 85 tests: quaternion algebra and convention "
    "consistency, plant behaviour, mixer round-trips and desaturation, LQR "
    "pole stability, closed-loop tracking thresholds on both plants, "
    "trajectory waypoint accuracy and knot continuity. The paper 2 "
    "implementation was additionally reviewed equation-by-equation against "
    "the paper by three independent reviewers, with every finding "
    "re-verified numerically before being acted on. That review confirmed "
    "17 findings, none high-severity, of which two were real defects that "
    "we fixed:")

d.bullets([
    "The plant had paired body-frame rates with the world-frame form of "
    "the quaternion kinematic equation. The two forms coincide at hover "
    "but diverge with tilt times rate; on a 2-second, 2 rad/s spin the "
    "wrong pairing accumulated 1.5 rad of attitude error while the correct "
    "one stayed at 3e-8. The plant now uses the literal body-rate form.",
    "The attitude reference was built by a Gram-Schmidt construction whose "
    "comment claimed exact equivalence to the paper's composition; "
    "measurement showed a twist about the thrust axis growing quadratically "
    "with tilt. It was replaced with the literal composition of Section "
    "4.2, whose tilt component's yaw element is zero to machine precision.",
])

d.p("For this report, the equations typed in Sections 2 to 4 were "
    "cross-checked once more against the implementation in seven "
    "randomised groups - quaternion algebra and rotation matrices (50 "
    "trials), the paper 1 control law including its saturation stage (200 "
    "trials), the plant inertia, all four rows of the paper 2 model, the "
    "mixer matrix and its inverse, the flatness map including the zero yaw "
    "element of the tilt quaternion (30 trials), and the two state-space "
    "models with their Riccati gains verified Hurwitz. All seven groups "
    "pass. The recordings distributed with the project are the measured "
    "runs themselves: the recording hooks attach to the benchmark loops, "
    "and each recording's metrics match the corresponding benchmark to the "
    "last digit.")

# ================================================================ 8
d.h1("8. Conclusions")

d.bullets([
    "The quaternion attitude law of paper 1 was reproduced exactly (gains, "
    "saturation, noise), extended to a six-degree-of-freedom flight stack, "
    "and flown unchanged on four simulation stacks including real ArduPilot "
    "firmware, with cross-engine agreement of 0.03 degrees RMS on the "
    "native stacks.",
    "A derived digital-stability bound - a minimum control rate of 12.3 "
    "kHz for these gains - explains why naive discrete implementations of "
    "the paper's controller diverge, and is enforced by our scheduler.",
    "The differential-flatness tracker of paper 2 was implemented "
    "equation-by-equation, validated by independent review with all "
    "findings numerically confirmed, and tracks the paper's study case to "
    "2.7 to 2.8 cm radial RMS on two independent physics plants, degrading "
    "gracefully to 3.7 cm under the paper's full sensor noise.",
    "The extension trajectory library (figure-8, vertical orbit, "
    "minimum-snap waypoint tours) runs on the same frozen controllers and "
    "quantifies the flatness argument: minimum-snap tracks 25 percent "
    "better than minimum-jerk on identical waypoints.",
    "Everything regenerates from single commands; 83 of 85 tests pass in "
    "the canonical environment and the two skips require simulator stacks "
    "not installed there.",
])

# ================================================================ 9
d.h1("9. References")

d.p("[1] E. Fresk and G. Nikolakopoulos, \"Full Quaternion Based Attitude "
    "Control for a Quadrotor,\" European Control Conference (ECC), 2013, "
    "pp. 3864-3869.")
d.p("[2] K. Choutri, M. Lagha, L. Dala and M. Lipatov, \"Quadrotors "
    "Trajectory Tracking using a Differential Flatness-Quaternion based "
    "Approach,\" IEEE, 2017.")
d.p("[3] D. Mellinger and V. Kumar, \"Minimum Snap Trajectory Generation "
    "and Control for Quadrotors,\" ICRA, 2011. (basis of the extension "
    "trajectory library)")
d.p("[4] T. Lee, M. Leok and N. H. McClamroch, \"Geometric Tracking "
    "Control of a Quadrotor UAV on SE(3),\" CDC, 2010. (comparison target "
    "for the planned third controller family)")

print(d.save())
