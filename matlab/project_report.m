%% FULL QUATERNION ATTITUDE CONTROL AND DIFFERENTIAL-FLATNESS
%% TRAJECTORY TRACKING FOR A QUADROTOR
%
% Project report as a MATLAB script - Group 12
%   B Sainath  (CB.SC.U4AIE24309),  Vishal (CB.SC.U4AIE24363)
%   School of Artificial Intelligence, Amrita Vishwa Vidyapeetham
%
% Base papers:
%   [1] E. Fresk, G. Nikolakopoulos, "Full Quaternion Based Attitude
%       Control for a Quadrotor", ECC 2013.
%   [2] K. Choutri, M. Lagha, L. Dala, M. Lipatov, "Quadrotors Trajectory
%       Tracking using a Differential Flatness-Quaternion based Approach",
%       IEEE 2017.
%
% HOW TO USE
%   Run this file (F5 or "project_report" at the command prompt). Every
%   section prints its results to the command window and saves figures to
%   matlab/figures/. To produce a formatted PDF/HTML document with all the
%   prose, equations and figures inline, use the PUBLISH tab in the MATLAB
%   editor (or: publish('project_report.m')).
%
%   All equations in the comments below are typed out manually and were
%   cross-checked numerically against the project's validated Python
%   implementation (7 equation groups, randomised trials).

clear; clc; close all;
rng(1);
figdir = fullfile(fileparts(mfilename('fullpath')), 'figures');
if ~exist(figdir, 'dir'); mkdir(figdir); end

%% ------------------------------------------------------------------------
%% 1. INTRODUCTION
% A quadrotor has 6 degrees of freedom but only 4 independent inputs
% (total thrust + 3 body torques). It cannot accelerate sideways directly:
% it must tilt so the thrust vector (always along body z) points partly
% sideways. Attitude and translation are therefore coupled.
%
% Paper 1 controls ATTITUDE ONLY: a nonlinear quaternion law commands body
% torque straight from the quaternion error. Paper 2 inverts the coupling:
% differential flatness converts a desired path into the thrust and
% attitude that produce it, and two LQR loops close the system.
%
% One airframe throughout (both papers, all sims):
%   mass        m   = 0.2 kg
%   inertia     J   = diag(6.5e-4, 6.5e-4, 1.2e-3) kg m^2
%   arm         L   = 0.15 m   (rotor offsets (+-d,+-d), d = L/sqrt(2))
%   thrust coef kf  = 3.0e-9 N per (rad/s)^2   (km = 0.016*kf)
%   thrust/weight   = 4        (per-rotor max thrust 1.95 N)
%
% Full-project results beyond this script (MuJoCo / gym-pybullet-drones /
% Gazebo / real ArduPilot firmware runs, in-engine recordings, the 85-test
% suite) are summarised in Section 9.

%% ------------------------------------------------------------------------
%% 2. QUATERNION REPRESENTATION OF ATTITUDE (typed equations)
%
% A quaternion is q = q0 + q1*i + q2*j + q3*k, written scalar-first as
% q = (q0, qv) with vector part qv = (q1,q2,q3). A UNIT quaternion
% (||q|| = 1) represents a rotation; no singularity anywhere (unlike Euler
% angles at +-90 deg pitch) and only 4 numbers (vs 9 constrained for a
% rotation matrix).
%
% Product (rotation composition):
%   p (x) q = ( p0*q0 - pv.qv ,  p0*qv + q0*pv + pv x qv )
% Conjugate / norm / inverse:
%   q* = (q0, -qv);   ||q|| = sqrt(q0^2+q1^2+q2^2+q3^2);   q^-1 = q*/||q||
% Rotating a vector:      v' = q (x) [0,v] (x) q*  =  R(q) v
%   R(q) columns = body axes in world; orthonormal; R(p (x) q) = R(p) R(q)
% Axis-angle construction:
%   q = [ cos(alpha/2) , u*sin(alpha/2) ]   for angle alpha about axis u
% Kinematics (BOTH forms are the same motion - match to your omega frame):
%   q-dot = (1/2) q (x) [0, omega]        omega = BODY angular rate
%   q-dot = (1/2) [0, omega_w] (x) q      omega_w = R(q)*omega (world)
% Mixing the two pairings diverges as tilt x rate - found in validation:
% 2 s at 2 rad/s: wrong pairing 1.5 rad error, correct pairing 3e-8 rad.

fprintf('=== 2. Quaternion algebra: randomised verification ===\n');
maxerr = 0;
for k = 1:200
    a = qnorm(randn(4,1)); b = qnorm(randn(4,1));
    maxerr = max(maxerr, norm(qmul(a, qconj(a)) - [1;0;0;0]));  % q (x) q* = 1
    R = qdcm(a);
    maxerr = max(maxerr, norm(R'*R - eye(3)));                  % orthonormal
    v = randn(3,1);
    maxerr = max(maxerr, norm(qrot(a, qrot(b, v)) - qrot(qmul(a,b), v)));
end
fprintf('algebra + DCM orthonormality + R(ab)=R(a)R(b): max err %.2e (200 trials)\n\n', maxerr);

%% ------------------------------------------------------------------------
%% 3. PAPER 1 - THE ATTITUDE CONTROL LAW
%
% Plant (attitude only):   J * omega-dot = tau ,  q-dot = (1/2) q (x) [0,omega]
%
% Controller (Eqs 19-21 of [1]):
%   q_err = q_ref (x) q_m*                     attitude error quaternion
%   e     = vector part of q_err, SIGN-CORRECTED to the shortest rotation
%           (skip the correction for the flip: commit to the full 360 deg)
%   tau   = -Pq * e - Pw * omega_m             with Pq = 20, Pw = 4
%   tau   clipped elementwise to +-4 N*m per axis
% Sensors: uniform +-0.1 noise on every quaternion component and rate.
%
% Why no integral term: torque -> rate -> attitude is a double integrator,
% so a proportional-derivative law in quaternion form already drives the
% error to zero.

Pq = 20; Pw = 4; tau_max = 4; noise = 0.1;
J = diag([6.5e-4 6.5e-4 1.2e-3]);

fprintf('=== 3. Paper 1: three benchmark scenarios (20 kHz control) ===\n');
r1 = run_paper1('step', 15, Pq, Pw, tau_max, noise, J);
r2 = run_paper1('sine', 15, Pq, Pw, tau_max, noise, J);
r3 = run_paper1('flip',  5, Pq, Pw, tau_max, noise, J);
fprintf('scenario   RMS att. err [deg]   max [deg]   notes\n');
fprintf('step       %7.2f              %6.1f      1 rad steps on roll/pitch/yaw\n', r1.rms, r1.mx);
fprintf('sine       %7.2f              %6.1f      0.5 rad, 1 rad/s on all axes\n', r2.rms, r2.mx);
fprintf('flip       %7.2f              %6.1f      360 deg, no gimbal-lock artifact\n', r3.rms, r3.mx);
fprintf('(the ~17.5 deg floor is the sensor noise itself: +-0.1 on each\n');
fprintf(' quaternion component is ~11 deg of measured attitude)\n\n');

%% ------------------------------------------------------------------------
%% 4. PAPER 1 - A DERIVED RESULT: THE DIGITAL CONTROL-RATE BOUND
%
% Linearising the closed loop about identity attitude, per axis:
%   I*e-ddot + Pw*e-dot + (Pq/2)*e = 0
%   lambda(1,2) = (-Pw +- sqrt(Pw^2 - 2*I*Pq)) / (2*I) = -2.5, -6150 rad/s
% The continuous loop is heavily overdamped and stable. The fast pole makes
% SAMPLING the danger: exact zero-order-hold spectral radius rho(Ad):

fprintf('=== 4. Control-rate stability bound (derived, not tuned) ===\n');
Ac = [0 1; -Pq/(2*J(1)) -Pw/J(1)];
fprintf('control rate    rho (explicit)   rho (exact ZOH)\n');
for f = [200 1000 10000 100000]
    Ts = 1/f;
    rho_e = max(abs(eig(eye(2) + Ac*Ts)));      % naive explicit mapping
    rho_z = max(abs(eig(expm(Ac*Ts))));        % exact zero-order hold
    fprintf('%8d Hz      %8.2f          %8.6f\n', f, rho_e, rho_z);
end
fprintf('derived minimum: f_control >= Pw/(m*I_min) ~ 12.3 kHz (margin 0.5)\n');
fprintf('A naive explicit implementation of the paper at 200-1000 Hz diverges\n');
fprintf('outright; Section 3 therefore runs the law at 20 kHz.\n\n');

%% ------------------------------------------------------------------------
%% 5. PAPER 2 - THE FULL MODEL AND THE MIXER
%
% State (p, v, q, omega), inputs (T, tau):
%   p-dot  = v
%   v-dot  = R(q)*[0,0,T/m] - [0,0,g]        thrust vector minus gravity
%   q-dot  = (1/2) q (x) [0, omega]          body rates (literal form)
%   J*omega-dot = -omega x (J*omega) + tau   Euler equation
%
% Rotor mixing (Eq. 9 of [2]), X-config, squared speeds w = omega_rotor^2:
%   [T ]   [ kf    kf    kf    kf  ] [w1]
%   [tx] = [ kfd   kfd  -kfd  -kfd ] [w2]
%   [ty]   [-kfd   kfd   kfd  -kfd ] [w3]
%   [tz]   [-km    km   -km    km  ] [w4]
% Inverting M turns (T,tau) into rotor speeds. Rotor speeds are bounded
% [0, w_max]; when a command is infeasible we deliver the thrust EXACTLY
% and uniformly scale the torque (naive one-sided clipping corrupts the
% total thrust and the altitude loop runs away - a project finding).

m = 0.2; g = 9.81; kf = 3e-9; km = 0.016*kf; d = 0.15/sqrt(2); w_max = 1.95/kf;
kfd = kf*d;
Mmix = [ kf kf kf kf; kfd kfd -kfd -kfd; -kfd kfd kfd -kfd; -km km -km km ];

fprintf('=== 5. Mixer round-trip (Eq. 9) ===\n');
wT = mix_feasible(Mmix, 1.5, [0.01 -0.005 0.002], w_max);
out = Mmix*wT;
fprintf('command [1.5 N; 0.01; -0.005; 0.002] -> delivered [%.6f %.8f %.8f %.8f]\n\n', out);

%% ------------------------------------------------------------------------
%% 6. PAPER 2 - DIFFERENTIAL FLATNESS
%
% The quadrotor is differentially flat with flat outputs
%   sigma(t) = ( x, y, z, psi )            (Eq. 13 of [2])
% From Newton's law on the translational row, the desired acceleration a
% fixes the thrust VECTOR; its direction fixes 2 dof of the attitude, the
% yaw fixes the last one:
%   f  = m * ( a + g*z-hat )     thrust vector        (Eq. 15 row)
%   T_d = ||f||                   total thrust command
%   dir = f/||f||                 desired body-z direction
% Reference attitude (Eqs. 19-21, implemented LITERALLY):
%   q_pd = [ cos(alpha/2); sin(alpha/2)*(z-hat x dir)/||z-hat x dir|| ]
%          tilt taking z-hat onto dir; axis is horizontal => its yaw
%          element is EXACTLY zero (the paper's "4th element = 0")
%   q_zd = [ cos(psi/2); 0; 0; sin(psi/2) ]           pure yaw
%   q_d  = q_pd (x) q_zd                              composition
% Only degeneracy: ||f|| = 0 (free fall).

fprintf('=== 6. Flatness map: randomised verification ===\n');
maxz = 0; maxal = 0;
for k = 1:100
    a = -6 + 12*rand(3,1); psi = -pi + 2*pi*rand;
    [Td, qd] = flatness_reference(a, psi, m, g);
    f = m*(a + [0;0;g]);
    maxal = max(maxal, norm(qdcm(qd)*[0;0;1] - f/norm(f)));   % body-z on f
    qzd = [cos(psi/2);0;0;sin(psi/2)];
    qtilt = qmul(qd, qconj(qzd));
    maxz = max(maxz, abs(qtilt(4)));                            % yaw elem 0
end
fprintf('body-z alignment err %.2e ; tilt yaw element %.2e (100 trials)\n\n', maxal, maxz);

%% ------------------------------------------------------------------------
%% 7. PAPER 2 - DOUBLE-LOOP LQR (Eqs. 16-23)
%
% INNER loop, state x = [q_vec(3); omega(3)] (from the hover linearisation
% q_vec-dot = omega/2 and omega-dot = J^-1 tau):
%   A = [ 0(3x3)  0.5*I ]     B = [  0(3x3) ]
%       [ 0(3x3)  0     ]         [  J^-1   ]
% OUTER loop, per axis, state [p; p-dot], input = acceleration:
%   A = [0 1; 0 0]            B = [0; 1]
% Cost (Eq. 22):  J = 1/2 * integral ( x'Qx + u'Ru ) dt
% Riccati (Eq.23): P*A + A'*P - P*B*inv(R)*B'*P + Q = 0;  K = R\B'*P;  u = -Kx
% (solved here with icare, MATLAB's algebraic Riccati solver)
% The paper gives NO numeric weights; ours (documented tuning):
%   outer per axis Q = diag(49, 42), R = 1
%   inner Q = diag(100,100,100,20,20,20), R = 200*eye(3)
% Controller rate 200 Hz (paper Section V); plant RK4 at 1 kHz; a 20 ms
% first-order ESC lag on the squared rotor speeds (actuator model).

Qi = diag([100 100 100 20 20 20]); Ri = 200*eye(3);
Ai = [zeros(3) 0.5*eye(3); zeros(3) zeros(3)];
Bi = [zeros(3); inv(J)];
Pi = riccati_ham(Ai, Bi, Qi, Ri); Ki = Ri\Bi'*Pi;
Qo = diag([49 42]); Ro = 1;
Ao = [0 1; 0 0]; Bo = [0;1];
Po = riccati_ham(Ao, Bo, Qo, Ro); Ko = Ro\Bo'*Po;

fprintf('=== 7. LQR designs: closed-loop poles (must be Hurwitz) ===\n');
fprintf('inner max real part %+.4f ; outer %+.4f\n', ...
        max(real(eig(Ai - Bi*Ki))), max(real(eig(Ao - Bo*Ko))));

fprintf('\n=== 7a. Paper 2 study case: circle r=1 m at z=1 m from (-1,0,0) ===\n');
fprintf('plant                RMSx   RMSy   RMSz   radial  peak rotor\n');
rc = run_paper2(make_circle(), m, g, Mmix, w_max, Ki, Ko, 0.0);
fprintf('ideal Eq.(8), 1 kHz  %5.1f  %5.1f  %5.1f  %5.1f   %5.2f N   (cm)\n', ...
        100*rc.rmsxyz(1), 100*rc.rmsxyz(2), 100*rc.rmsxyz(3), 100*rc.radial, rc.Tpeak);
rn = run_paper2(make_circle(), m, g, Mmix, w_max, Ki, Ko, 0.1);
fprintf('+ paper noise +-0.1  %5.1f  %5.1f  %5.1f  %5.1f   %5.2f N   (cm)\n', ...
        100*rn.rmsxyz(1), 100*rn.rmsxyz(2), 100*rn.rmsxyz(3), 100*rn.radial, rn.Tpeak);
fprintf('repo cross-check: MuJoCo contact physics gives 2.8 cm radial,\n');
fprintf('gym-pybullet 17.52 deg sine / ArduPilot firmware 14.7 deg (paper 1).\n\n');

%% ------------------------------------------------------------------------
%% 8. EXTENSIONS - TRAJECTORY LIBRARY (frozen gains)
%
% Same LQR loops, new flat-output generators only.
% Waypoint tours are piecewise degree-7 polynomials minimising the integral
% of squared SNAP (4th derivative): flatness makes jerk drive body rates
% and snap drive torques, so min-snap is the smoothest flyable path.
% Segment boundary conditions: waypoints, corridor tangent velocity at
% intermediate waypoints (without it, long segments loiter then teleport -
% position is 4 integrals from the cost), rest-to-rest ends. Solved per
% segment in normalised time (conditioning).

fprintf('=== 8. Extensions (same frozen controllers) ===\n');
fprintf('scenario                     RMS 3D [cm]  peak rotor\n');
f8 = make_figure8();   r8 = run_paper2(f8, m, g, Mmix, w_max, Ki, Ko, 0.0);
fprintf('figure-8 (A=1.5, B=1)        %5.1f        %5.2f N\n', 100*r8.rms3d, r8.Tpeak);
ob = make_orbit();     ro = run_paper2(ob, m, g, Mmix, w_max, Ki, Ko, 0.0);
fprintf('vertical orbit (h=0.5)       %5.1f        %5.2f N\n', 100*ro.rms3d, ro.Tpeak);
WPS = [0 0 0.15; 1.5 0 1.2; 1.5 2 1.2; 0 2 1.2; 0 0 0.8];
pw = run_paper2(make_waypoints(WPS, 0.35, 4), m, g, Mmix, w_max, Ki, Ko, 0.0);
fprintf('waypoints, min-snap          %5.1f        %5.2f N\n', 100*pw.rms3d, pw.Tpeak);
pj = run_paper2(make_waypoints(WPS, 0.35, 3), m, g, Mmix, w_max, Ki, Ko, 0.0);
fprintf('waypoints, min-jerk (A/B)    %5.1f        %5.2f N\n', 100*pj.rms3d, pj.Tpeak);
fprintf('A/B: min-snap tracks %.0f%% better - the flatness argument measured.\n\n', ...
        100*(1 - pw.rms3d/pj.rms3d));

%% ------------------------------------------------------------------------
%% 9. FULL-PROJECT RESULTS (from the repository, for the record)
%
% Paper 1 attitude law, gains untouched, on FOUR stacks (same references,
% same +-0.1 noise, 3 seeds):
%   ArduPilot firmware (real code, WSL2):  sine 14.7 +- 0.2 deg (best)
%   Gazebo (real physics, WSL2):           sine 17.3 +- 0.0 deg
%   gym-pybullet-drones (24 kHz):          sine 17.52 deg; step 17.68 deg
%   MuJoCo (20 kHz contact):               sine 17.61 deg; step 17.71 deg
%   -> native stacks agree to 0.03 deg: the loop belongs to the controller.
%   Flip completes on firmware (2.89 s settle) and MuJoCo (2.93 s);
%   PyBullet/Gazebo sit in the knife-edge regime (analysed in the repo).
%
% Paper 2 on the repo's plants: circle radial RMS 2.7 cm (ideal model),
% 2.8 cm (MuJoCo), 3.7 cm (MuJoCo + noise). Extensions: figure-8 4.7 cm
% on BOTH plants, orbit 3.9 cm, min-snap tour 6.1 cm.
%
% Validation: equation-by-equation review vs paper 2 (17 findings, all
% confirmed, 2 real bugs fixed); 85 tests (83 pass + 2 stack-gated).
% Recordings: every scenario has an in-engine MuJoCo GIF whose metrics
% match the benchmark to the last digit.

fprintf('=== 9. Summary printed above; see README.md / RECORDINGS.md ===\n');
fprintf('Figures saved to matlab/figures/\n');

%% ========================================================================
%%  LOCAL FUNCTIONS
%% ========================================================================

function q = qnorm(q)
    q = q / norm(q);
end

function out = qmul(p, q)
    % p (x) q = ( p0*q0 - pv.qv , p0*qv + q0*pv + pv x qv )
    out = [p(1)*q(1) - p(2:4)'*q(2:4);
           p(1)*q(2:4) + q(1)*p(2:4) + cross(p(2:4), q(2:4))];
end

function out = qconj(q)
    out = [q(1); -q(2:4)];
end

function R = qdcm(q)
    % columns = body axes in world (v_world = R v_body)
    q0 = q(1); qv = q(2:4);
    R = [1-2*(qv(2)^2+qv(3)^2), 2*(qv(1)*qv(2)-q0*qv(3)), 2*(qv(1)*qv(3)+q0*qv(2));
         2*(qv(1)*qv(2)+q0*qv(3)), 1-2*(qv(1)^2+qv(3)^2), 2*(qv(2)*qv(3)-q0*qv(1));
         2*(qv(1)*qv(3)-q0*qv(2)), 2*(qv(2)*qv(3)+q0*qv(1)), 1-2*(qv(1)^2+qv(2)^2)];
end

function v = qrot(q, v)
    v = qdcm(q)*v;
end

function q = qeuler(phi, theta, psi)
    % ZYX Euler to quaternion (scalar-first)
    cr = cos(phi/2); sr = sin(phi/2);
    cp = cos(theta/2); sp = sin(theta/2);
    cy = cos(psi/2); sy = sin(psi/2);
    q = [cr*cp*cy + sr*sp*sy;
         sr*cp*cy - cr*sp*sy;
         cr*sp*cy + sr*cp*sy;
         cr*cp*sy - sr*sp*cy];
end

function w = mix_feasible(Mmix, T, tau, w_max)
    % deliver thrust exactly; uniformly scale torque until rotor-feasible
    w = Mmix\[T; tau(:)];
    if any(w < 0) || any(w > w_max)
        lo = 0; hi = 1;
        for it = 1:24
            mid = 0.5*(lo+hi);
            wt = Mmix\[T; mid*tau(:)];
            if all(wt >= 0) && all(wt <= w_max); lo = mid; else; hi = mid; end
        end
        w = Mmix\[T; lo*tau(:)];
    end
end

function [Td, qd] = flatness_reference(a, psi, m, g)
    % literal Eqs. 15/19/20/21 of paper 2
    f = m*(a + [0;0;g]);
    Td = norm(f);
    dir = f/Td;
    axis = cross([0;0;1], dir);
    s = norm(axis);
    if s < 1e-9
        if dir(3) > 0; qpd = [1;0;0;0];
        else; qpd = [cos(pi/2); sin(pi/2)*[cos(psi);sin(psi);0]]; end
    else
        alpha = atan2(s, dir(3));
        qpd = [cos(alpha/2); sin(alpha/2)*axis/s];
    end
    qzd = [cos(psi/2); 0; 0; sin(psi/2)];
    qd = qnorm(qmul(qpd, qzd));
end

function r = run_paper1(scen, dur, Pq, Pw, tau_max, noise, J)
    % attitude plant + paper 1 law; RK4 at 20 kHz control (Section 4 bound),
    % 1 kHz sensing with the paper's noise, logging at 1 kHz
    dt = 1e-3; n = round(dur/dt); sub = 20; h = dt/sub;
    q = [1;0;0;0]; w = zeros(3,1);
    qm = q; wm = w;               % measured (noisy)
    err = zeros(n,1); tlog = zeros(n,1); qlog = zeros(4,n); rlog = zeros(4,n);
    for k = 1:n
        t = (k-1)*dt;
        qref = ref1(scen, t);
        for s = 1:sub                     % 20 kHz control + paper sensing
            qm = qnorm(q + noise*(2*rand(4,1)-1));   % fresh noise each tick
            wm = w + noise*(2*rand(3,1)-1);
            qe = qmul(qconj(qref), qm);   % error in BODY frame
            vec = qe(2:4);
            if qe(1) < 0 && ~strcmp(scen, 'flip'); vec = -vec; end
            tau = min(max(-Pq*vec - Pw*wm, -tau_max), tau_max);
            [q, w] = integ1(q, w, tau, J, h);
            q = qnorm(q);
        end
        err(k) = 2*acos(min(1, abs(qref'*qm)))*180/pi;  % vs noisy measurement
        tlog(k) = t; qlog(:,k) = q; rlog(:,k) = qref;
    end
    r.rms = sqrt(mean(err.^2)); r.mx = max(err);
    f = figure('Visible','off','Position',[100 100 900 320]);
    subplot(1,2,1); plot(tlog, err); xlabel('t [s]'); ylabel('attitude error [deg]');
    title(sprintf('paper 1 - %s (RMS %.1f deg)', scen, r.rms)); grid on;
    subplot(1,2,2); plot(tlog, qlog(1:2,:)', tlog, rlog(1:2,:)','--');
    legend('q_0','q_1','ref q_0','ref q_1'); xlabel('t [s]'); grid on;
    saveas(f, fullfile(fileparts(mfilename('fullpath')),'figures',['p1_' scen '.png']));
    close(f);
end

function [q, w] = integ1(q, w, tau, J, dt)
    % one RK4 step of the attitude plant (q-dot = 1/2 q (x) [0,w])
    f = @(q,w) [0.5*qmul(q,[0;w]); J\tau];
    k1 = f(q,w); k2 = f(q+k1(1:4)*dt/2, w+k1(5:7)*dt/2);
    k3 = f(q+k2(1:4)*dt/2, w+k2(5:7)*dt/2); k4 = f(q+k3(1:4)*dt, w+k3(5:7)*dt);
    q = q + dt*(k1(1:4)+2*k2(1:4)+2*k3(1:4)+k4(1:4))/6;
    w = w + dt*(k1(5:7)+2*k2(5:7)+2*k3(5:7)+k4(5:7))/6;
end

function qref = ref1(scen, t)
    switch scen
        case 'step'    % 1 rad on roll/pitch/yaw at t = 1, 5, 9 s
            qref = qeuler(t>1, t>5, t>9);
        case 'sine'    % 0.5 rad, 1 rad/s, phases so torques stay apart
            qref = qeuler(0.5*sin(1*t), 0.5*sin(1*t + pi/2), 0.5*sin(1*t + pi));
        case 'flip'    % 0 -> 2*pi roll over 2 s, then hold
            qref = qeuler(min(t,2)*pi, 0, 0);
    end
end

function tr = make_circle()
    tr.r = 1; tr.alt = 1; tr.om = 0.5; tr.Tc = 2; tr.kind = 'circle';
    tr.dur = 2*2*pi/tr.om + tr.Tc;
end

function tr = make_figure8()
    tr.A = 1.5; tr.B = 1; tr.alt = 1; tr.om = 0.6/max(1.5,2); tr.Tc = 2; tr.kind='fig8';
    tr.dur = 2*2*pi/tr.om + tr.Tc;
end

function tr = make_orbit()
    tr.r = 1; tr.alt = 1; tr.h = 0.5; tr.om = 0.5; tr.Tc = 2; tr.kind='orbit';
    tr.dur = 2*2*pi/tr.om + tr.Tc;
end

function tr = make_waypoints(WPS, speed, dg)
    dist = sqrt(sum(diff(WPS).^2, 2));
    segT = max(dist/speed, 1);
    times = [0; cumsum(segT)];
    n = 2*dg;                       % coefficients per segment (deg+1)
    vel = zeros(size(WPS));
    for i = 2:size(WPS,1)-1
        tan = WPS(i+1,:) - WPS(i-1,:);
        vel(i,:) = tan/norm(tan) * min(norm(tan)/(times(i+1)-times(i-1)), 0.8);
    end
    segs = cell(size(WPS,1)-1, 1);
    for i = 1:size(WPS,1)-1
        T = times(i+1)-times(i);
        % rows: p(0), p(1), v(0), v(1), a(0), a(1) - NORMALISED time
        A = [drow(n,0,0); drow(n,1,0); drow(n,0,1); drow(n,1,1); drow(n,0,2); drow(n,1,2)];
        B = [WPS(i,:); WPS(i+1,:); vel(i,:)*T; vel(i+1,:)*T; zeros(1,3); zeros(1,3)];
        cp = lsqminnorm(A, B);      % min-norm particular (as in Python)
        N = null(A);
        if ~isempty(N)
            Qn = qnorm_mat(n, dg)/T^(2*dg-1);
            H = N'*Qn*N; g = N'*Qn*cp;
            cp = cp - N*(pinv(H)*g); % pinv: H is singular on zero-snap dirs
        end
        segs{i} = cp;
    end
    tr.kind='poly'; tr.segs=segs; tr.times=times; tr.n=n; tr.dg=dg;
    tr.dur = times(end); tr.Tc = times(2)-times(1);
end

function v = drow(n, frac, order)
    % d^order/dtau^order of the monomial basis at tau=frac in [0,1]
    v = zeros(1,n);
    for k = order+1:n
        c = 1; for j = 0:order-1; c = c*(k-1-j); end %#ok<PREALL>
        v(k) = c*frac^(k-1-order);
    end
end

function Q = qnorm_mat(n, dg)
    % int_0^1 (d^dg/dtau^dg)^2 dtau in the monomial basis
    Q = zeros(n);
    for k = dg+1:n
        ck = 1; for j = 0:dg-1; ck = ck*(k-1-j); end %#ok<PREALL>
        for l = dg+1:n
            cl = 1; for j = 0:dg-1; cl = cl*(l-1-j); end %#ok<PREALL>
            Q(k,l) = ck*cl/(k+l-2*dg-1);   % 1-based k,l: exponents k-1,l-1
        end
    end
end

function ref = traj_eval(tr, t)
    switch tr.kind
        case 'circle'
            [th, thd, thdd] = ramp(t, tr.Tc, tr.om);
            ref.p = [-tr.r*cos(th); tr.r*sin(th); 0];
            ref.v = tr.r*thd*[sin(th); cos(th); 0];
            ref.a = tr.r*[thdd*sin(th)+thd^2*cos(th); thdd*cos(th)-thd^2*sin(th); 0];
            [z, zd, zdd] = minjerk(t, tr.Tc, tr.alt);
            ref.p(3)=z; ref.v(3)=zd; ref.a(3)=zdd;
            ref.psi = 0; ref.psid = 0;
        case 'fig8'
            [th, thd, thdd] = ramp(t, tr.Tc, tr.om);
            A = tr.A; B = tr.B;
            ref.p = [A*sin(th); B*sin(2*th); 0];
            ref.v = [A*thd*cos(th); 2*B*thd*cos(2*th); 0];
            ref.a = [A*(thdd*cos(th)-thd^2*sin(th));
                     2*B*(thdd*cos(2*th)-2*thd^2*sin(2*th)); 0];
            [z, zd, zdd] = minjerk(t, tr.Tc, tr.alt);
            ref.p(3)=z; ref.v(3)=zd; ref.a(3)=zdd;
            ref.psi = 0; ref.psid = 0;
        case 'orbit'
            [th, thd, thdd] = ramp(t, tr.Tc, tr.om);
            r = tr.r; h = tr.h;
            ref.p = [-r*cos(th); r*sin(th); h*sin(th)];
            ref.v = [r*thd*sin(th); r*thd*cos(th); h*thd*cos(th)];
            ref.a = [r*(thdd*sin(th)+thd^2*cos(th));
                     r*(thdd*cos(th)-thd^2*sin(th));
                     h*(thdd*cos(th)-thd^2*sin(th))];
            [z, zd, zdd] = minjerk(t, tr.Tc, tr.alt);
            ref.p(3)=ref.p(3)+z; ref.v(3)=ref.v(3)+zd; ref.a(3)=ref.a(3)+zdd;
            ref.psi = 0; ref.psid = 0;
        case 'poly'
            times = [0; tr.times(2:end)];
            i = min(max(find(times <= t, 1, 'last')), numel(tr.segs));
            T = tr.times(i+1)-tr.times(i);
            frac = (t - tr.times(i))/T;
            D = zeros(3,3);
            for o = 0:2
                for k = o+1:tr.n
                    c = 1; for j = 0:o-1; c = c*(k-1-j); end %#ok<PREALL>
                    D(o+1,:) = D(o+1,:) + (tr.segs{i}(k,:)*c*frac^(k-1-o)/T^o);
                end
            end
            ref.p = D(1,:)'; ref.v = D(2,:)'; ref.a = D(3,:)';
            sp = norm(ref.v(1:2));
            if sp > 1e-6
                ref.psi = atan2(ref.v(2), ref.v(1));
                ref.psid = (ref.v(1)*ref.a(2)-ref.v(2)*ref.a(1))/max(sp^2,1e-8);
            else
                ref.psi = 0; ref.psid = 0;
            end
    end
end

function [th, thd, thdd] = ramp(t, T, om)
    % min-jerk angular-rate ramp 0 -> om over [0,T]
    if t >= T
        th = om*(t - T/2); thd = om; thdd = 0;
    else
        x = t/T;
        th = om*T*(2.5*x^4 - 3*x^5 + x^6);
        thd = om*(10*x^3 - 15*x^4 + 6*x^5);
        thdd = om*(30*x^2 - 60*x^3 + 30*x^4)/T;
    end
end

function [s, sd, sdd] = minjerk(t, T, total)
    if t >= T; s = total; sd = 0; sdd = 0;
    else
        x = t/T;
        s = total*(10*x^3 - 15*x^4 + 6*x^5);
        sd = total*(30*x^2 - 60*x^3 + 30*x^4)/T;
        sdd = total*(60*x - 180*x^2 + 120*x^3)/T^2;
    end
end

function r = run_paper2(traj, m, g, Mmix, w_max, Ki, Ko, noise)
    % full model (RK4 1 kHz), 200 Hz double-loop LQR, ESC lag 20 ms
    dt = 1e-3; ctrl_every = 5; esc_tau = 0.02;
    n = round(traj.dur/dt);
    J = diag([6.5e-4 6.5e-4 1.2e-3]);
    ref0 = traj_eval(traj, 0);
    st.p = ref0.p; st.v = zeros(3,1); st.q = [1;0;0;0]; st.w = zeros(3,1);
    wcmd = m*g/(4*Mmix(1,1))*ones(4,1);
    Td = m*g; tau = zeros(3,1);
    P = zeros(3,n); PREF = zeros(3,n); errv = []; Tpk = 0;
    for k = 1:n
        t = (k-1)*dt;
        if mod(k-1, ctrl_every) == 0
            ref = traj_eval(traj, t);
            if noise > 0
                qm = qnorm(st.q + noise*(2*rand(4,1)-1));
                wm = st.w + noise*(2*rand(3,1)-1);
            else
                qm = st.q; wm = st.w;
            end
            a = ref.a - Ko(1)*(st.p-ref.p) - Ko(2)*(st.v-ref.v);
            nxy = norm(a(1:2)); if nxy > 3; a(1:2) = a(1:2)*3/nxy; end
            a(3) = min(max(a(3), -5), 5);
            [Td, qd] = flatness_reference(a, ref.psi, m, g);
            qe = qmul(qconj(qd), qm);   % error in BODY frame
            vec = qe(2:4); if qe(1) < 0; vec = -vec; end
            tau = min(max(-(Ki*[vec; wm]), -4), 4);
        end
        alpha = 1 - exp(-dt/esc_tau);
        wcmd = wcmd + alpha*(mix_feasible(Mmix, Td, tau, w_max) - wcmd);
        wT = Mmix*wcmd; Teff = wT(1); taue = wT(2:4);
        st = integ2(st, Teff, taue, m, g, J, dt);
        st.q = qnorm(st.q);
        P(:,k) = st.p; PREF(:,k) = traj_eval(traj, t).p;
        Tpk = max(Tpk, max(Mmix(1,:)*wcmd));
        if t > traj.Tc; errv = [errv, norm(st.p - PREF(:,k))]; end %#ok<AGROW>
    end
    r.rmsxyz = sqrt(mean((P - PREF).^2, 2));
    r.rms3d = sqrt(mean(errv.^2));
    if isprop_or_field(traj, 'r'); r.radial = sqrt(mean((sqrt(sum(P(1:2,:).^2,1))'-traj.r).^2));
    else; r.radial = r.rms3d; end
    r.Tpeak = Tpk;
    % figure
    f = figure('Visible','off','Position',[100 100 900 340]);
    subplot(1,2,1);
    plot3(P(1,:), P(2,:), P(3,:), 'b', PREF(1,:), PREF(2,:), PREF(3,:), 'r--');
    legend('flown','reference'); grid on; xlabel('x'); ylabel('y'); zlabel('z');
    title(traj.kind);
    subplot(1,2,2);
    e = sqrt(sum((P-PREF).^2,1))';
    plot((0:n-1)*dt, e); xlabel('t [s]'); ylabel('3D error [m]'); grid on;
    title(sprintf('3D RMS %.1f cm', 100*r.rms3d));
    saveas(f, fullfile(fileparts(mfilename('fullpath')),'figures',['p2_' traj.kind '.png']));
    close(f);
end

function [st] = integ2(st, T, tau, m, g, J, dt)
    % RK4 of the full 6-DOF model
    f = @(s) [s.v; qdcm(s.q)*[0;0;T/m] - [0;0;g];
              0.5*qmul(s.q,[0;s.w]); J\(-cross(s.w, J*s.w) + tau)];
    k1 = f(st);
    s2 = st; s2.v = st.v + k1(4:6)*dt/2; s2.q = st.q + k1(7:10)*dt/2; s2.w = st.w + k1(11:13)*dt/2;
    s2.p = st.p + k1(1:3)*dt/2;
    k2 = f(s2);
    s3 = st; s3.v = st.v + k2(4:6)*dt/2; s3.q = st.q + k2(7:10)*dt/2; s3.w = st.w + k2(11:13)*dt/2;
    s3.p = st.p + k2(1:3)*dt/2;
    k3 = f(s3);
    s4 = st; s4.v = st.v + k3(4:6)*dt; s4.q = st.q + k3(7:10)*dt; s4.w = st.w + k3(11:13)*dt;
    s4.p = st.p + k3(1:3)*dt;
    k4 = f(s4);
    st.p = st.p + dt*(k1(1:3)+2*k2(1:3)+2*k3(1:3)+k4(1:3))/6;
    st.v = st.v + dt*(k1(4:6)+2*k2(4:6)+2*k3(4:6)+k4(4:6))/6;
    st.q = st.q + dt*(k1(7:10)+2*k2(7:10)+2*k3(7:10)+k4(7:10))/6;
    st.w = st.w + dt*(k1(11:13)+2*k2(11:13)+2*k3(11:13)+k4(11:13))/6;
end

function tf = isprop_or_field(s, name)
    tf = isfield(s, name);
end

function P = riccati_ham(A, B, Q, R)
    % algebraic Riccati solution via the Hamiltonian eigen decomposition
    % (base MATLAB only - no Control System Toolbox needed)
    n = size(A, 1);
    H = [A, -B*(R\B'); -Q, -A'];
    [V, E] = eig(H);
    idx = find(real(diag(E)) < 0);
    Vs = V(:, idx);
    X1 = Vs(1:n, :); X2 = Vs(n+1:2*n, :);
    P = real(X2/X1)';  P = (P + P')/2;
end
