---
title: Quadrotor Trajectory Tracking — Full Math Derivations
subtitle: Choutri, Lagha, Dala, Lipatov (2017), ICMSAO — every equation, where it comes from, what each symbol means
---



# 0. Big Picture — How the Equations Connect

```text
Quaternion math (Eq. 1-7)    : q, q*, q (x) r, R_q, q_dot = 1/2 q (x) w
Dynamics (Eq. 8-10)          : full model (8), motors to T and torques (9), linear model at hover (10)
Flatness (Eq. 11-15)         : outputs x, y, z, psi give the desired angles (15)
State-space (Eq. 16-18)      : A and B matrices from the linear model
Desired attitude (Eq. 19-21) : q_pd and q_zd combine into q_d
LQR (Eq. 22-23)              : cost and Riccati equation give u = -Lx
```

| Eq. | What it is | Where it comes from |
|---|---|---|
| (1) | Quaternion definition | Definition |
| (2) | Quaternion product | Rules $i^2=j^2=k^2=ijk=-1$ |
| (3) | Conjugate, norm, inverse | Eq. (2) applied to $q\otimes q^*$ |
| (4) | Rotation matrix $R_q$ | Eq. (2) and (3) applied to $q\otimes v\otimes q^*$ |
| (5) | Axis–angle form of $q$ | Eq. (4) compared with Rodrigues' rotation formula |
| (6) | $\dot q=\tfrac12 q\otimes\omega$ | Small rotation in time $dt$ |
| (7) | Euler angles to/from quaternion | Multiply three axis quaternions; compare with Eq. (4) |
| (8) | Full nonlinear dynamics | Newton's law + Euler's rotation equation |
| (9) | Motor speeds → thrust & torques | Rotor physics (force × arm) |
| (10) | Linear model at hover | Eq. (8) with small-angle approximation |
| (11),(12) | Definition of differential flatness | Definition |
| (13),(14) | The four flat outputs | Definition + trivial derivatives |
| (15) | Desired pitch/roll from acceleration | Translation part of Eq. (8) written with Euler angles |
| (16) | General linear state-space | Definition |
| (17) | Attitude state-space | Eq. (10) rotation part |
| (18) | Position state-space | Eq. (10) translation part |
| (19)–(21) | Desired quaternion $q_d$ | Eq. (5) and (2) |
| (22) | LQR cost function | Definition (design choice) |
| (23) | Riccati equation | Minimising Eq. (22) subject to Eq. (16) |

---

# 0.1 Symbols Used in the Whole Paper

| Symbol | Meaning |
|---|---|
| $p\in\mathbb{R}^3$ | Position of the drone in the inertial (ground) frame: $[x,y,z]^T$ |
| $\dot p,\ \ddot p$ | Velocity, acceleration |
| $q\in\mathbb{H}$ | Unit quaternion = orientation of the drone body w.r.t. the ground frame |
| $q_0$ | Scalar (real) part of $q$ |
| $\bar q=[q_1,q_2,q_3]^T$ | Vector (imaginary) part of $q$ |
| $\omega\in\mathbb{R}^3$ | Angular velocity in the **body** frame |
| $m$ | Mass of the drone |
| $J$ | Inertia matrix (3×3) in the body frame |
| $g$ | Gravity, $9.81\ \text{m/s}^2$; $\bar g=[0,0,-g]^T$ |
| $T$ | Total thrust (force along the body $z$-axis) |
| $\tau=[\tau_x,\tau_y,\tau_z]^T$ | Torques about the body axes (roll, pitch, yaw) |
| $\phi,\theta,\psi$ | Roll, pitch, yaw (Euler angles) |
| $\omega_i$ | Speed of rotor $i$ ($i=1\ldots4$) |
| $k_i$ | Thrust coefficient of rotor $i$ |
| $l$ | Distance from centre of mass to rotor axis (arm length) |
| $\mathcal{X}=[X_1,X_2,X_3,X_4]^T$ | Flat output $=[x,y,z,\psi]^T$ |
| $A,B,C$ | State-space matrices |
| $Q,R,P,L$ | LQR weights, Riccati solution, feedback gain |

> The paper sometimes numbers quaternion elements 1–4 (e.g. $q_{4zd}$) and sometimes 0–3. In this file I always use $q_0,q_1,q_2,q_3$, with $q_3$ = the $z$-component.

---

# 1. Quaternion Mathematics (Eq. 1 – 7)

**Why quaternions at all?** Euler angles $(\phi,\theta,\psi)$ are easy to imagine but break at $\theta=\pm90^\circ$ (**gimbal lock**: two axes line up and one degree of freedom is lost). A quaternion uses 4 numbers with 1 constraint ($\lVert q\rVert=1$), which gives 3 degrees of freedom, with no singularity.

## Eq. (1) — Quaternion definition

$$
q = q_0 + \bar q = q_0 + \begin{bmatrix} q_1\\ q_2\\ q_3\end{bmatrix},\qquad q\in\mathbb{H},\ \bar q\in\mathbb{R}^3,\ q_0\in\mathbb{R}
\tag{1}
$$

**Meaning of each part**

- $q_0$: scalar part. It is linked to *how much* we rotate.
- $\bar q$: vector part. It is linked to *which axis* we rotate about.
- Written fully: $q=q_0+q_1\,i+q_2\,j+q_3\,k$, where $i,j,k$ are three different "imaginary units".
- Identity quaternion $[1,0,0,0]^T$ means **no rotation** (this is the hover value in the paper).

## Eq. (2) — Quaternion product

$$
q\otimes r = (q_0r_0-\bar q\cdot\bar r)+(r_0\bar q+q_0\bar r+\bar q\times\bar r)
\tag{2}
$$

**Where it comes from (derivation)**

Hamilton's rules for the imaginary units:

$$
i^2=j^2=k^2=ijk=-1\ \Rightarrow\ ij=k,\ jk=i,\ ki=j,\quad ji=-k,\ kj=-i,\ ik=-j
$$

*Step 1.* Multiply term by term:
$(q_0+q_1i+q_2j+q_3k)(r_0+r_1i+r_2j+r_3k)$.

*Step 2.* **Scalar part.** Only $q_0r_0$ and the products $i\cdot i,\ j\cdot j,\ k\cdot k$ (each $=-1$) give scalars:

$$
q_0r_0-q_1r_1-q_2r_2-q_3r_3=q_0r_0-\bar q\cdot\bar r
$$

*Step 3.* **$i$-part.** Collect all terms ending in $i$:
$q_0r_1+q_1r_0+q_2r_3\,(jk=i)-q_3r_2\,(kj=-i)$

$$
= q_0r_1+r_0q_1+(q_2r_3-q_3r_2)
$$

The bracket is exactly the $x$-component of the cross product $\bar q\times\bar r$.

*Step 4.* The $j$ and $k$ parts work the same way, so the full vector part is $r_0\bar q+q_0\bar r+\bar q\times\bar r$.

**Meaning of each part**

- $-\bar q\cdot\bar r$ (dot product): how much the two axes agree.
- $r_0\bar q+q_0\bar r$: each scalar scales the other's axis.
- $\bar q\times\bar r$ (cross product): the "twisting" term. **It changes sign if you swap the order**, so $q\otimes r\neq r\otimes q$.
- Example: $i\otimes j=k$ but $j\otimes i=-k$. **Rotation order matters.**

## Eq. (3) — Conjugate, norm, inverse

$$
q^*=q_0-\bar q,\qquad
\lVert q\rVert=\sqrt{q\otimes q^*}=\sqrt{q_0^2+q_1^2+q_2^2+q_3^2},\qquad
q^{-1}=\frac{q^*}{\lVert q\rVert^2}
\tag{3}
$$

**Derivation**

*Step 1.* Put $r=q^*=(q_0,-\bar q)$ into Eq. (2).

*Step 2.* Scalar part: $q_0q_0-\bar q\cdot(-\bar q)=q_0^2+\lVert\bar q\rVert^2=q_0^2+q_1^2+q_2^2+q_3^2$.

*Step 3.* Vector part: $q_0(-\bar q)+q_0\bar q+\bar q\times(-\bar q)=\mathbf 0$ (because $\bar q\times\bar q=0$).

*Step 4.* So $q\otimes q^*=\lVert q\rVert^2$, a pure real number. Dividing: $q\otimes\dfrac{q^*}{\lVert q\rVert^2}=1$, so $q^{-1}=\dfrac{q^*}{\lVert q\rVert^2}$.

*Step 5.* **Unit quaternion** ($\lVert q\rVert=1$): $q^{-1}=q^*$. This is why the paper only uses unit quaternions: the inverse is free (just flip the sign of the vector part).

**Paper typo:** The paper prints the inverse as $q^*/\lVert q\rVert$ and the norm with indices $q_2^2+q_3^2+q_4^2$. The correct forms are above.

**Meaning:** conjugate = "rotate backwards"; norm = "length" of the quaternion; inverse = the quaternion that undoes $q$.

## Eq. (4) — Rotation matrix $R_q$

$$
R_q=(q_0I+q_{1_{3\times}})^2+\bar q\bar q^T
=(q_0^2-\bar q^T\bar q)I+2q_0q_{1_{3\times}}+2\bar q\bar q^T
\tag{4a}
$$

$$
R_q=\begin{bmatrix}
q_0^2+q_1^2-q_2^2-q_3^2 & 2(q_1q_2-q_3q_0) & 2(q_1q_3+q_2q_0)\\
2(q_1q_2+q_3q_0) & q_0^2-q_1^2+q_2^2-q_3^2 & 2(q_2q_3-q_1q_0)\\
2(q_1q_3-q_2q_0) & 2(q_2q_3+q_1q_0) & q_0^2-q_1^2-q_2^2+q_3^2
\end{bmatrix}
\tag{4b}
$$

**Where it comes from.** A vector $v$ is rotated by $v'=q\otimes v\otimes q^*$, with $v$ treated as the pure quaternion $(0,v)$.

*Step 1.* First product, using Eq. (2) with $r=(0,v)$:

$$
q\otimes v=\big(-\bar q\cdot v,\ \ q_0v+\bar q\times v\big)
$$

*Step 2.* Multiply by $q^*=(q_0,-\bar q)$ on the right. The scalar part cancels to $0$ (so $v'$ is again a pure vector). The vector part is:

$$
q_0(q_0v+\bar q\times v)+(\bar q\cdot v)\bar q-(q_0v+\bar q\times v)\times\bar q
$$

*Step 3.* Expand the last term with $v\times\bar q=-\bar q\times v$ and the identity $(\bar q\times v)\times\bar q=v\,(\bar q\cdot\bar q)-\bar q\,(v\cdot\bar q)$:

$$
v'=(q_0^2-\bar q\cdot\bar q)\,v+2(\bar q\cdot v)\,\bar q+2q_0\,(\bar q\times v)
\tag{4c}
$$

*Step 4.* Write each piece as a matrix times $v$:

- $\bar q\times v=q_{1_{3\times}}v$, where $q_{1_{3\times}}=\begin{bmatrix}0&-q_3&q_2\\ q_3&0&-q_1\\ -q_2&q_1&0\end{bmatrix}$ is the **skew-symmetric matrix** of $\bar q$.
- $(\bar q\cdot v)\bar q=\bar q\bar q^Tv$.

So $v'=R_qv$ with $R_q=(q_0^2-\bar q^T\bar q)I+2q_0q_{1_{3\times}}+2\bar q\bar q^T$. This is the second form in Eq. (4a).

*Step 5.* **Check that the first form equals the second.** $(q_0I+S)^2=q_0^2I+2q_0S+S^2$ with $S=q_{1_{3\times}}$, and $S^2=\bar q\bar q^T-(\bar q^T\bar q)I$. Therefore $(q_0I+S)^2+\bar q\bar q^T=(q_0^2-\bar q^T\bar q)I+2q_0S+2\bar q\bar q^T$.

*Step 6.* **Matrix entries (Eq. 4b).** Example: entry (1,1) $=q_0^2-(q_1^2+q_2^2+q_3^2)+2q_1^2=q_0^2+q_1^2-q_2^2-q_3^2$. Entry (1,2) $=2q_1q_2+2q_0S_{12}=2q_1q_2-2q_0q_3$. All others follow the same way.

**Meaning of each part**

- $I$: 3×3 identity. $S$: skew matrix, which performs the cross product.
- $\bar q\bar q^T$: 3×3 matrix = (column)×(row).
- $R_q$ is **orthogonal**: $R_q^{-1}=R_q^T$ and $\det R_q=1$ (it is a pure rotation, with no stretching). This is the statement under Eq. (4) in the paper.
- **Third column of $R_q$** = where the body $z$-axis (thrust direction) points in the ground frame:

$$
R_q\,e_3=\begin{bmatrix}2(q_1q_3+q_2q_0)\\ 2(q_2q_3-q_1q_0)\\ q_0^2-q_1^2-q_2^2+q_3^2\end{bmatrix}
$$

This column is used in Eq. (10).

## Eq. (5) — Axis–angle form

$$
q=\cos\frac{\alpha}{2}+u\sin\frac{\alpha}{2}
\tag{5}
$$

**Meaning:** $u$ is the unit rotation axis ($\lVert u\rVert=1$) and $\alpha$ is the rotation angle.

**Derivation (it must rotate vectors the right way)**

*Step 1.* Put $q_0=\cos\frac\alpha2$ and $\bar q=u\sin\frac\alpha2$ into Eq. (4c).

*Step 2.* Use $c=\cos\frac\alpha2,\ s=\sin\frac\alpha2$:

$$
v'=(c^2-s^2)v+2s^2(u\cdot v)u+2cs\,(u\times v)
$$

*Step 3.* Double-angle identities: $c^2-s^2=\cos\alpha$, $2cs=\sin\alpha$, $2s^2=1-\cos\alpha$:

$$
v'=\cos\alpha\,v+\sin\alpha\,(u\times v)+(1-\cos\alpha)(u\cdot v)\,u
$$

*Step 4.* This is exactly **Rodrigues' rotation formula**: rotate $v$ by angle $\alpha$ about axis $u$.

**Why half-angles?** Because $q$ appears **twice** in $q\otimes v\otimes q^*$, so each copy carries half of the rotation.

**Where used:** Eq. (20), the yaw quaternion (axis $u=[0,0,1]^T$, angle $\alpha=\psi_d$).

## Eq. (6) — Quaternion kinematics

$$
\dot q=\frac12\,q\otimes\omega
\tag{6}
$$

**Meaning:** how the orientation changes when the body spins with angular velocity $\omega$ (body frame). $\omega$ is used as the pure quaternion $(0,\omega)$.

**Derivation (small rotation)**

*Step 1.* In a tiny time $dt$, the body rotates by the small angle $\alpha=\lVert\omega\rVert dt$ about the axis $u=\omega/\lVert\omega\rVert$.

*Step 2.* By Eq. (5), this small rotation is

$$
\delta q=\Big[\cos\tfrac{\lVert\omega\rVert dt}{2},\ \ u\sin\tfrac{\lVert\omega\rVert dt}{2}\Big]\approx\Big[1,\ \tfrac12\omega\,dt\Big]
$$

(because $\cos(\text{small})\approx1$ and $\sin(\text{small})\approx$ small).

*Step 3.* The new orientation: $q(t+dt)=q(t)\otimes\delta q$ (body-frame rotation multiplies on the right).

*Step 4.* Difference quotient:

$$
\dot q=\lim_{dt\to0}\frac{q\otimes\delta q-q}{dt}=q\otimes\frac{(0,\ \tfrac12\omega\,dt)}{dt}=\frac12\,q\otimes\omega
$$

**Where used:** third row of Eq. (8). At hover, $q\approx[1,0,0,0]$, so $\dot{\bar q}\approx\tfrac12\omega$, which is the "0.5" in Eq. (17).

## Eq. (7) — Euler angles to/from quaternion

**Euler → quaternion** (yaw–pitch–roll, order Z–Y–X, with $c_x=\cos\frac x2,\ s_x=\sin\frac x2$):

$$
q=\begin{bmatrix}
c_\phi c_\theta c_\psi+s_\phi s_\theta s_\psi\\
s_\phi c_\theta c_\psi-c_\phi s_\theta s_\psi\\
c_\phi s_\theta c_\psi+s_\phi c_\theta s_\psi\\
c_\phi c_\theta s_\psi-s_\phi s_\theta c_\psi
\end{bmatrix}
\tag{7a}
$$

**Derivation**

*Step 1.* Each single-axis rotation is a quaternion by Eq. (5):

- Roll: $q_\phi=[c_\phi,\ s_\phi,\ 0,\ 0]$ (axis $x$)
- Pitch: $q_\theta=[c_\theta,\ 0,\ s_\theta,\ 0]$ (axis $y$)
- Yaw: $q_\psi=[c_\psi,\ 0,\ 0,\ s_\psi]$ (axis $z$)

*Step 2.* Total rotation: $q=q_\psi\otimes q_\theta\otimes q_\phi$.

*Step 3.* First $q_\psi\otimes q_\theta$ with Eq. (2): scalar $=c_\psi c_\theta$; vector $=c_\theta(0,0,s_\psi)+c_\psi(0,s_\theta,0)+(0,0,s_\psi)\times(0,s_\theta,0)=(-s_\psi s_\theta,\ c_\psi s_\theta,\ s_\psi c_\theta)$.

*Step 4.* Then multiply by $q_\phi=(c_\phi,(s_\phi,0,0))$. Scalar: $c_\psi c_\theta c_\phi-(-s_\psi s_\theta)s_\phi=c_\phi c_\theta c_\psi+s_\phi s_\theta s_\psi$ (the first row of Eq. 7a). The three vector components give the other three rows in the same way.

**Quaternion → Euler** (from comparing Eq. 4b with the Euler rotation matrix):

$$
\begin{aligned}
\phi&=\operatorname{atan2}\!\big(2(q_0q_1+q_2q_3),\ q_0^2-q_1^2-q_2^2+q_3^2\big)\\
\theta&=\arcsin\!\big(2(q_0q_2-q_3q_1)\big)\\
\psi&=\operatorname{atan2}\!\big(2(q_0q_3+q_1q_2),\ q_0^2+q_1^2-q_2^2-q_3^2\big)
\end{aligned}
\tag{7b}
$$

**Derivation.** For ZYX Euler angles: $R_{31}=-\sin\theta$, $R_{32}=\cos\theta\sin\phi$, $R_{33}=\cos\theta\cos\phi$, $R_{21}=\cos\theta\sin\psi$, $R_{11}=\cos\theta\cos\psi$. Match these to entries of Eq. (4b):

- $R_{31}=2(q_1q_3-q_2q_0)=-\sin\theta$ so $\theta=\arcsin(2(q_0q_2-q_1q_3))$
- $R_{32}/R_{33}=\tan\phi$ so $\phi=\operatorname{atan2}(2(q_2q_3+q_1q_0),\ q_0^2-q_1^2-q_2^2+q_3^2)$
- $R_{21}/R_{11}=\tan\psi$ so $\psi=\operatorname{atan2}(2(q_1q_2+q_3q_0),\ q_0^2+q_1^2-q_2^2-q_3^2)$

**Paper typo:** The paper prints $2(q_0q_1\,\mathbf{-}\,q_2q_3)$ inside the $\phi$ formula. Derivation gives **plus**. This is most likely a typo.

**Why useful:** the controller works in quaternions, but humans (and plots) want angles in degrees.

> **Understanding check:** If $q=[1,0,0,0]$, what are $\phi,\theta,\psi$? *(Answer: all 0. Check: $\theta=\arcsin 0=0$, $\phi=\operatorname{atan2}(0,1)=0$.)*

---

# 2. Dynamic Model (Eq. 8 – 10)

## Eq. (8) — Full nonlinear model (Newton–Euler with quaternions)

$$
\dot x=\frac{d}{dt}\begin{bmatrix}p\\ \dot p\\ q\\ \omega\end{bmatrix}=
\begin{bmatrix}
\dot p\\[2pt]
q\otimes\dfrac{T}{m}\otimes q^*+\bar g\\[6pt]
\dfrac12\,q\otimes\omega\\[6pt]
J^{-1}(\tau-\omega\times J\omega)
\end{bmatrix}
\tag{8}
$$

State: $p$ (3) + $\dot p$ (3) + $q$ (4) + $\omega$ (3) = **13 states**. Inputs: $T,\tau_x,\tau_y,\tau_z$ = **4 inputs**. That is why the system is **underactuated** (6 degrees of freedom, 4 inputs).

**Row-by-row derivation**

**Row 1: $\dot p$.** The derivative of position is velocity. (Definition.)

**Row 2: translation (Newton's second law $F=ma$ in the ground frame).**

*Step 1.* Forces on the drone: thrust and gravity. $m\ddot p=F_{thrust}+m\bar g$.

*Step 2.* The thrust is produced by the rotors **along the body $z$-axis**: in the body frame it is $[0,0,T]^T$.

*Step 3.* To express it in the ground frame, rotate it with $q$ (Eq. 4): $q\otimes[0,0,T]\otimes q^*=R_q[0,0,T]^T$.

*Step 4.* Divide by $m$: $\ddot p=q\otimes\frac Tm\otimes q^*+\bar g$, with $\bar g=[0,0,-g]^T$.

**Row 3: attitude kinematics** is Eq. (6).

**Row 4: rotation (Euler's rotation equation).**

*Step 1.* Angular momentum in the body frame: $H=J\omega$.

*Step 2.* Newton's law for rotation: $\dfrac{dH}{dt}\Big|_{ground}=\tau$.

*Step 3.* Converting the derivative from the rotating body frame to the ground frame adds a term: $\dfrac{dH}{dt}\Big|_{ground}=\dfrac{dH}{dt}\Big|_{body}+\omega\times H$.

*Step 4.* $J$ is constant in the body frame, so $J\dot\omega+\omega\times J\omega=\tau$, and therefore $\dot\omega=J^{-1}(\tau-\omega\times J\omega)$.

**Meaning of each part**

- $T$: total thrust. $m$: mass. $\bar g$: gravity vector. $J^{-1}$: inverse inertia (heavier/larger body so smaller angular acceleration).
- $\omega\times J\omega$: **gyroscopic term**. It appears because the body is rotating. It is **zero when $\omega=0$**, which matters for Eq. (10).
- $\tau=\tau_u+\tau_{ext}$: torque from the motors ($\tau_u$) plus external disturbances such as wind ($\tau_{ext}$).
- Assumptions: rigid body; centre of gravity at the origin of the body frame.

## Eq. (9) — Motors → thrust and torques

$$
\begin{bmatrix}T\\ \tau_{u_x}\\ \tau_{u_y}\\ \tau_{u_z}\end{bmatrix}=
\begin{bmatrix}
\sum_{i=1}^{4}k_i\omega_i^2\\
l\,(k_1\omega_1^2-k_2\omega_2^2-k_3\omega_3^2+k_4\omega_4^2)\\
l\,(k_1\omega_1^2+k_2\omega_2^2-k_3\omega_3^2-k_4\omega_4^2)\\
\sum_{i=1}^{4}\tau_i(-1)^{i}
\end{bmatrix}
\tag{9}
$$

**Paper typo:** The paper prints $(-1)^2$ in the last row, which would always be $+1$. It should be $(-1)^i$, so that the reaction torques alternate in sign.

**Derivation (physical reasoning)**

*Step 1.* A propeller spinning at $\omega_i$ gives thrust $F_i=k_i\omega_i^2$ (thrust grows with the **square** of the speed). Total thrust: $T=\sum F_i$.

*Step 2.* **Roll/pitch torque = force × lever arm.** A rotor at distance $l$ from the centre pushes up with $F_i$ and creates a torque $l\,F_i$ about the axis. Rotors on opposite sides of an axis give opposite signs. That gives the $+$/$-$ patterns in rows 2 and 3.

*Step 3.* **Yaw torque.** A spinning propeller drags air, and the air pushes back on the drone with a reaction torque $\tau_i$. Neighbouring rotors spin in opposite directions, so their reaction torques alternate in sign: $\sum\tau_i(-1)^i$. If all four are equal, yaw torque is zero and the drone does not spin.

**Meaning of each part**

- $k_i$: thrust coefficient (depends on propeller shape). $\omega_i$: rotor speed. $l$: arm length. $\tau_i$: reaction (drag) torque of rotor $i$.
- The map is **linear in $\omega_i^2$**, so with four unknowns $\omega_i^2$ and four equations it can be inverted. We can therefore treat $(T,\tau_x,\tau_y,\tau_z)$ as the control inputs, and a "mixer" converts them into rotor speeds (the PWM signals in the paper's Fig. 8).

## Eq. (10) — Linearisation at hover

**Operating point (hover):** $\dot p=[0,0,0]^T$, $\omega=[0,0,0]^T$, $q=[1,0,0,0]^T$.

$$
J\dot\omega=\tau,\qquad
\dot q=\frac12\begin{bmatrix}0\\ \omega\end{bmatrix},\qquad
m\ddot p=\begin{bmatrix}mg\,(2q_2)\\ -mg\,(2q_1)\\ T\end{bmatrix}-\begin{bmatrix}0\\0\\mg\end{bmatrix}
=\begin{bmatrix}mg\,u_x\\ -mg\,u_y\\ T\end{bmatrix}-\begin{bmatrix}0\\0\\mg\end{bmatrix}
\tag{10}
$$

with $u_x=2q_2$ and $u_y=2q_1$ (small tilt values).

**Paper typo:** The paper prints $2q_0q_2$ in **both** the first and second row of the first vector. From the derivation below, the second row is $-2q_0q_1\approx-2q_1$.

**Derivation**

*Step 1 — rotation part.* Near hover, $\omega\approx0$, so the gyroscopic term $\omega\times J\omega$ is a product of two small numbers so ignore it. Eq. (8) row 4 becomes $J\dot\omega=\tau$.

*Step 2 — kinematics.* With $q\approx[1,0,0,0]$ in Eq. (6): $q\otimes\omega\approx\omega$, so $\dot q\approx\frac12[0;\omega]$.

*Step 3 — translation.* Take the **third column** of $R_q$ from Eq. (4b), which is where the thrust points, and multiply by $T$:

$$
m\ddot p=T\begin{bmatrix}2(q_1q_3+q_2q_0)\\ 2(q_2q_3-q_1q_0)\\ q_0^2-q_1^2-q_2^2+q_3^2\end{bmatrix}-\begin{bmatrix}0\\0\\mg\end{bmatrix}
$$

*Step 4 — small angles.* Near hover $q_0\approx1$ and $q_1,q_2,q_3$ are small, so products like $q_1q_3$ and squares like $q_1^2$ are negligible:

- $x$-row: $2(q_1q_3+q_2q_0)\approx2q_2$
- $y$-row: $2(q_2q_3-q_1q_0)\approx-2q_1$
- $z$-row: $q_0^2-\ldots\approx1$

*Step 5 — thrust near hover.* $T\approx mg$ (the thrust that balances gravity), so in the horizontal rows $T\cdot2q_2\approx mg\cdot2q_2$. The $z$ row stays $T-mg$.

Result: Eq. (10).

**Meaning:** a small **tilt** ($q_2$ about $y$, $q_1$ about $x$) makes the thrust point slightly sideways, giving horizontal acceleration $\ddot x\approx 2g\,q_2$ and $\ddot y\approx-2g\,q_1$ (about $g\times$ tilt angle, since $2q_2\approx\theta$). Thrust above $mg$ makes the drone climb.

> **Understanding check:** At hover, what is $T$? *(Answer: $T=mg$, so $\ddot z=0$.)*

---

# 3. Differential Flatness (Eq. 11 – 15)

## Eq. (11), (12) — Definition

$$
\dot x(t)=f\big(x(t),u(t)\big),\qquad y=h\big(x(t)\big)
\tag{11}
$$

The system is **flat** if there is an output $z(t)$ (the **flat output**) such that **all states and inputs** can be written using only $z$ and a **finite number of its derivatives**:

$$
x(t)=\Xi_1\big(z,\dot z,\ddot z,\ldots,z^{(\alpha)}\big),\qquad
u(t)=\Xi_2\big(z,\dot z,\ddot z,\ldots,z^{(\beta)}\big)
\tag{12}
$$

**Meaning of each part**

- $x$: states, $u$: inputs, $f$: system dynamics, $h$: output map.
- $z$: flat output. $z^{(i)}$: its $i$-th time derivative. $\alpha,\beta$: finite integers (how many derivatives are needed).
- $\Xi_1,\Xi_2$: smooth maps (just algebraic formulas, with **no integration** needed).
- The components of $z$ must be **differentially independent** (none is a function of the others and their derivatives).

**Simple example first** *(extra, not in the paper)*. The double integrator $\ddot x=u$ with state $[x,\dot x]$.

- Choose $z=x$. Then $x=z$, $\dot x=\dot z$, and $u=\ddot z$. Both state and input follow from $z$ and its derivatives, so the system is flat.
- Planning means: draw any smooth path $z(t)$, and the needed input is just $\ddot z(t)$, with no differential equation to solve.

**Why it helps the quadrotor:** plan a smooth path for $x,y,z,\psi$, and the required angles, thrust and torques come out as formulas of the path's derivatives.

## Eq. (13), (14) — Flat outputs of the quadrotor

$$
\mathcal X=\begin{bmatrix}x\\y\\z\\\psi\end{bmatrix}
\tag{13}
$$

$$
\begin{aligned}
[x,\,y,\,z]^T&=[\mathcal X_1,\,\mathcal X_2,\,\mathcal X_3]^T\\
[\dot x,\,\dot y,\,\dot z]^T&=[\dot{\mathcal X}_1,\,\dot{\mathcal X}_2,\,\dot{\mathcal X}_3]^T
\end{aligned}
\tag{14}
$$

- Four flat outputs for four inputs. Position gives velocity and acceleration by differentiating, which is "trivial".
- All 13 states and 4 inputs can be recovered from $x,y,z,\psi$ and their derivatives (up to the 4th derivative for the torques).

## Eq. (15) — Desired pitch and roll from the desired acceleration

$$
\theta_d=\arctan\!\left(\frac{\ddot{\mathcal X}_1\cos\mathcal X_4+\ddot{\mathcal X}_2\sin\mathcal X_4}{\ddot{\mathcal X}_3+g}\right)
\tag{15a}
$$

**Derivation (this is the key flatness step)**

*Step 1.* Take the translation equation of Eq. (8) and write the rotation $R$ with Euler angles ($R=R_z(\psi)R_y(\theta)R_x(\phi)$). The thrust direction (third column of $R$) is:

$$
R\,e_3=\begin{bmatrix}\cos\phi\sin\theta\cos\psi+\sin\phi\sin\psi\\ \cos\phi\sin\theta\sin\psi-\sin\phi\cos\psi\\ \cos\phi\cos\theta\end{bmatrix}
$$

*Step 2.* Newton's law $\ddot p=\frac Tm Re_3+[0,0,-g]^T$ gives three equations:

$$
\ddot x=\tfrac Tm(\cos\phi\sin\theta\cos\psi+\sin\phi\sin\psi)
$$
$$
\ddot y=\tfrac Tm(\cos\phi\sin\theta\sin\psi-\sin\phi\cos\psi)
$$
$$
\ddot z+g=\tfrac Tm\cos\phi\cos\theta
$$

*Step 3 — pitch.* Form $\ddot x\cos\psi+\ddot y\sin\psi$. The $\sin\phi$ terms cancel and $\cos^2\psi+\sin^2\psi=1$:

$$
\ddot x\cos\psi+\ddot y\sin\psi=\tfrac Tm\cos\phi\sin\theta
$$

*Step 4.* Divide by the third equation: $\dfrac{\ddot x\cos\psi+\ddot y\sin\psi}{\ddot z+g}=\dfrac{\sin\theta}{\cos\theta}=\tan\theta$. Hence Eq. (15a), with $\mathcal X_1=x,\ \mathcal X_2=y,\ \mathcal X_3=z,\ \mathcal X_4=\psi$.

*Step 5 — roll.* Form $\ddot x\sin\psi-\ddot y\cos\psi=\tfrac Tm\sin\phi$. Divide by $\tfrac Tm\cos\phi=(\ddot z+g)/\cos\theta$:

$$
\phi_d=\arctan\!\left(\frac{\ddot{\mathcal X}_1\sin\mathcal X_4-\ddot{\mathcal X}_2\cos\mathcal X_4}{\ddot{\mathcal X}_3+g}\cdot\cos\theta_d\right)
\tag{15b}
$$

**Paper typo:** The paper prints $\ddot{\mathcal X}_1\cos\mathcal X_4-\ddot{\mathcal X}_2\sin\mathcal X_4$ for roll. For $\psi=0$ the correct roll should depend on the **$y$-acceleration** (side motion needs roll), which the paper's form does not show. If asked, say: "Eq. (15a) matches my derivation exactly. For (15b) my Z-Y-X derivation gives sin/cos swapped; the paper may use a different sign or frame convention."

*Step 6 — thrust (needed for the altitude controller).* From the third equation:

$$
T=\frac{m(\ddot z+g)}{\cos\phi\cos\theta}
$$

*Step 7 — rates.* $\dot\theta_d$ and $\dot\phi_d$ in Eq. (15) are the **time derivatives** of (15a) and (15b) (the superscript $^{(1)}$ in the paper). This needs the third derivative of position (jerk), which is why the planned path must be smooth.

**Meaning of each part**

- Numerator of (15a): horizontal acceleration component along the heading direction. Denominator: vertical acceleration plus gravity (the thrust needed vertically).
- $\psi$ (yaw) is a free choice, as one of the four flat outputs.
- $\arctan(\text{sideways}/\text{vertical})$: simply the tilt angle needed so the thrust vector has the wanted sideways component.

**Worked example** *(extra, not in the paper)*. Circle of radius $r$, angular rate $\Omega$, constant altitude, $\psi=0$:

- $x_d=r\cos\Omega t,\ y_d=r\sin\Omega t,\ z_d=\text{const}$, so $\ddot x_d=-\Omega^2x_d,\ \ddot y_d=-\Omega^2y_d,\ \ddot z_d=0$.
- Then $\theta_d=\arctan\!\big(-\Omega^2x_d/g\big)$ and $\phi_d=\arctan\!\big(+\Omega^2y_d/g\cdot\cos\theta_d\big)$.
- With $r=1$ m and an assumed period of 8 s ($\Omega=2\pi/8\approx0.785$ rad/s): $\Omega^2\approx0.617$, so max acceleration $\approx0.617$ m/s² and max tilt $\approx\arctan(0.617/9.81)\approx3.6^\circ$. Small tilts, so the linear model of Eq. (10) is a reasonable approximation for this slow circle.

> **Understanding check:** Why does the drone need to tilt to move sideways? *(Answer: thrust only acts along the body $z$-axis; tilting puts part of the thrust in the horizontal direction.)*

---

# 4. Controller Design (Eq. 16 – 21)

## Eq. (16) — General linear state-space

$$
\dot x=Ax+Bu,\qquad y=Cx
\tag{16}
$$

- $x$: state vector, $u$: input, $y$: output.
- $A$: how states evolve by themselves. $B$: how inputs push the states. $C$: which combination of states we measure.

## Eq. (17) — Attitude subsystem

State $x_A=[\bar q;\ \omega]\in\mathbb{R}^6$, input $=\tau_u=[\tau_{u_x},\tau_{u_y},\tau_{u_z}]^T$:

$$
\dot x_A=\begin{bmatrix}0_{3\times3}&0.5\,I_{3\times3}\\ 0_{3\times3}&0_{3\times3}\end{bmatrix}x_A+\begin{bmatrix}0_{3\times3}\\ I_q^{-1}\end{bmatrix}\begin{bmatrix}\tau_{u_x}\\ \tau_{u_y}\\ \tau_{u_z}\end{bmatrix}
\tag{17}
$$

**Derivation from Eq. (10)**

*Step 1.* From $\dot q\approx\tfrac12[0;\omega]$, the vector part gives $\dot{\bar q}=0.5\,\omega$. This is the **top row**: $[\,0\ \ 0.5I\,]\,x_A=0.5\,\omega$.

*Step 2.* From $J\dot\omega=\tau$, $\dot\omega=J^{-1}\tau$. This is the **bottom row**: the state part is $0$ (angular acceleration does not depend on the current state) and the input part is $J^{-1}$.

*Step 3.* In the paper, the inertia matrix is written $I_q$, so the input matrix is $I_q^{-1}$.

**Meaning:** the attitude loop is a chain of two integrators: torque → angular velocity → quaternion vector part. We drive $\bar q\to0$ (identity orientation) or, with an error quaternion, $\bar q_e\to0$ (see below).

## Eq. (18) — Position subsystem

State $x_P=[p;\ \dot p]\in\mathbb{R}^6$:

$$
\dot x_P=\begin{bmatrix}0_{3\times3}&I_{3\times3}\\ 0_{3\times3}&0_{3\times3}\end{bmatrix}x_P+\begin{bmatrix}0_{3\times3}\\ \mathrm{diag}\!\left(g,\,-g,\,\tfrac1m\right)\end{bmatrix}\begin{bmatrix}u_{xd}\\ u_{yd}\\ T_d\end{bmatrix}
\tag{18}
$$

**Derivation from Eq. (10)**

*Step 1.* Top row: $\dot p=\dot p$, so $[\,0\ \ I\,]\,x_P=\dot p$.

*Step 2.* Bottom row from the translation part of Eq. (10):

- $\ddot x=g\,u_x$
- $\ddot y=-g\,u_y$
- $\ddot z=\dfrac{T-mg}{m}=\dfrac{T_d}{m}$, where $T_d$ is the thrust **deviation from hover thrust**.

*Step 3.* Put these into the matrix form: the diagonal $\mathrm{diag}(g,-g,1/m)$ multiplies the inputs $(u_{xd},u_{yd},T_d)$.

**Meaning of each part**

- $u_{xd},u_{yd}$: **desired tilt commands** (they equal $2q_2$ and $2q_1$ in Eq. 10).
- $T_d$: desired extra thrust from the **altitude controller**.
- Each axis is a **double integrator**: acceleration is proportional to the input. LQR is easy to apply to this.

## Controller structure (paper's Fig. 2 and Fig. 3)

1. **Altitude controller** (LQR on the $z$ channel): outputs the desired total thrust $T_d$.
2. **Position controller** (LQR on $x,y$): outputs the desired tilt, i.e. the **desired position quaternion** $q_{pd}$.
3. Combine $q_{pd}$ with the yaw quaternion into the **desired attitude** $q_d$ (Eq. 19–21).
4. **Attitude controller** (inner loop, LQR using Eq. 17): outputs torques $\tau_{u_x},\tau_{u_y},\tau_{u_z}$.
5. Total thrust $T$ and torques are turned into rotor PWM signals via Eq. (9).

**Why two loops?** Position cannot be controlled directly: first the drone must tilt (inner loop), and only then does it accelerate sideways (outer loop). The inner loop must be **faster** than the outer one, so that the tilt is reached before the position controller expects it.

## Eq. (19) — Desired position quaternion

$$
q_{pd}=[q_{0pd},\ q_{1pd},\ q_{2pd},\ 0]^T
\tag{19}
$$

**Meaning and origin**

- It describes only the **tilt** (roll/pitch), with **no yaw**, so the $z$-component is 0. (The paper calls it "4th element" because it counts 1–4.)
- *(Extra reasoning consistent with Eq. 10)* From Eq. (10), $u_x=2q_2$ and $u_y=2q_1$, so $q_{2pd}=u_{xd}/2$, $q_{1pd}=u_{yd}/2$, and since the quaternion must have unit length, $q_{0pd}=\sqrt{1-q_{1pd}^2-q_{2pd}^2}$.

## Eq. (20) — Desired yaw quaternion

$$
q_{zd}=\big[q_{0zd},\ 0,\ 0,\ q_{3zd}\big]^T=\Big[\cos\tfrac{\psi_d}{2},\ 0,\ 0,\ \sin\tfrac{\psi_d}{2}\Big]^T
\tag{20}
$$

**Derivation:** Eq. (5) with rotation axis $u=[0,0,1]^T$ (the $z$-axis) and angle $\alpha=\psi_d$. The second and third elements are zero because the axis has no $x$ or $y$ component. ($\psi_d$ is the 4th flat output.)

## Eq. (21) — Combined desired attitude

$$
q_d=q_{pd}\otimes q_{zd}
\tag{21}
$$

**Meaning:** "do the yaw rotation and the tilt rotation together" by multiplying the two quaternions with Eq. (2). The **order matters** because $\otimes$ is not commutative (the cross-product term in Eq. 2 changes sign when the factors are swapped). The result is again a unit quaternion.

**How $q_d$ is used** *(extra, standard practice from the references the paper cites, e.g. Fresk & Nikolakopoulos; not printed in this paper)*. The attitude loop works on the **error quaternion** $q_e=q_d^{*}\otimes q$. When the vector part $\bar q_e\to0$, the actual orientation $q$ equals the desired $q_d$. That is why $x_A$ in Eq. (17) contains the vector part $\bar q$: driving it to zero means "no attitude error".

---

# 5. LQR (Eq. 22 – 23)

## Eq. (22) — Cost function

$$
J(x,u)=\frac12\int_0^{\infty}\Big(x^T(t)\,Q\,x(t)+u^T(t)\,R\,u(t)\Big)\,dt
\tag{22}
$$

**Meaning of each part**

- $x^TQx$: **penalty for being away from the target** (state error). $Q\succeq0$ (positive semi-definite), usually diagonal. A big entry means "keep this state small".
- $u^TRu$: **penalty for using a lot of control effort**. $R\succ0$ (positive definite). Big $R$ means "be gentle with the motors/battery".
- $\int_0^\infty$: judge the whole future from now to infinity (an *infinite-horizon* problem).
- $\tfrac12$: only for convenience; it cancels the 2 that appears when differentiating.
- Because both terms are $\ge0$ and we need the integral to be finite, minimising $J$ forces $x\to0$, which gives **stability**.

**Trade-off analogy:** driving a car. Big $Q$ = "arrive exactly on the line, even with harsh braking". Big $R$ = "drive smoothly, even if you drift a bit".

> The paper says "$R$ and $Q$ are weight matrices used respectively to increase or diminish the effect of the states and the inputs". The order is swapped in that sentence: $Q$ weights **states** and $R$ weights **inputs**.

## Eq. (23) and the control law — Derivation

**Goal:** find $u$ that minimises Eq. (22) subject to $\dot x=Ax+Bu$ (Eq. 16).

*Step 1 — value function.* Define $V(x)=\tfrac12x^TPx$, the minimum remaining cost from state $x$ ($P$ symmetric, $P\succ0$).

*Step 2 — optimality condition (Hamilton–Jacobi–Bellman).* At the optimum, the running cost plus the rate of change of $V$ is zero:

$$
\min_u\Big[\tfrac12\big(x^TQx+u^TRu\big)+x^TP\,(Ax+Bu)\Big]=0
$$

(because $\dot V=\frac{\partial V}{\partial x}\dot x=x^TP(Ax+Bu)$).

*Step 3 — minimise over $u$.* Differentiate the bracket w.r.t. $u$ and set it to zero: $Ru+B^TPx=0$, so

$$
u=-R^{-1}B^TPx=-Lx,\qquad L=R^{-1}B^TP
$$

This is the paper's **"$u=-Lx$, $L=R^{-1}B^TP$"**.

*Step 4 — plug back.* With $u=-R^{-1}B^TPx$: $u^TRu=x^TPBR^{-1}B^TPx$ and $x^TPBu=-x^TPBR^{-1}B^TPx$. The bracket becomes

$$
\tfrac12x^TQx+\tfrac12x^TPBR^{-1}B^TPx+x^TPAx-x^TPBR^{-1}B^TPx=0
$$

*Step 5 — symmetrise.* $x^TPAx=\tfrac12x^T(PA+A^TP)x$. Collect terms:

$$
\tfrac12x^T\big[A^TP+PA-PBR^{-1}B^TP+Q\big]x=0\quad\text{for all }x
$$

*Step 6.* The matrix in brackets must be zero:

$$
\boxed{A^TP+PA-PBR^{-1}B^TP+Q=0}
\tag{23}
$$

This is the **algebraic Riccati equation (ARE)**.

**Paper typo:** The paper calls it the "Riccati *differential* equation". The finite-horizon problem has $-\dot P=\ldots$; for the infinite horizon used here, $\dot P=0$ and we get the **algebraic** form (23).

**Why it is stable (idea).** With $V=\tfrac12x^TPx$ as a Lyapunov function, $\dot V=-\tfrac12(x^TQx+u^TRu)<0$, so $x\to0$. LQR needs $(A,B)$ controllable (true for Eq. 17 and 18 because they are chains of integrators).

**Meaning of each part**

- $P$: the matrix that measures "how costly is each state error from now on".
- $L$: gain matrix. $u=-Lx$ is **state feedback** (each input is a weighted sum of all state errors).
- $R^{-1}B^TP$: bigger $R$ so smaller gain so gentler control.

**Design recipe (matches the paper's steps)**

1. Get $A,B$ (Eq. 17, 18).
2. Choose $Q,R$.
3. Solve ARE (23) for $P$.
4. Compute $L=R^{-1}B^TP$.
5. Apply $u=-Lx$ with $x$ = state **error**, $e=x_d-x$.

## Worked example 1 — scalar system *(extra, for exam practice)*

$\dot x=u$ so $A=0,\ B=1$. ARE: $0+0-P\cdot\frac1R\cdot P+Q=0\Rightarrow P=\sqrt{QR}$, then $L=P/R=\sqrt{Q/R}$.

| $Q$ | $R$ | $P$ | $L$ | Control law |
|---|---|---|---|---|
| 1 | 1 | 1 | 1 | $u=-x$ |
| 1 | 4 | 2 | 0.5 | $u=-0.5x$ (gentler) |
| 4 | 1 | 2 | 2 | $u=-2x$ (more aggressive) |

## Worked example 2 — double integrator (like each axis of Eq. 18) *(extra)*

$\ddot x=u$: $A=\begin{bmatrix}0&1\\0&0\end{bmatrix},\ B=\begin{bmatrix}0\\1\end{bmatrix}$. Take $Q=I,\ R=1$ and $P=\begin{bmatrix}p_1&p_2\\p_2&p_3\end{bmatrix}$.

*Step 1.* $A^TP=\begin{bmatrix}0&0\\p_1&p_2\end{bmatrix}$, $PA=\begin{bmatrix}0&p_1\\0&p_2\end{bmatrix}$, $PBB^TP=\begin{bmatrix}p_2^2&p_2p_3\\p_2p_3&p_3^2\end{bmatrix}$.

*Step 2.* Eq. (23) entry by entry:

- (1,1): $-p_2^2+1=0\Rightarrow p_2=1$
- (1,2): $p_1-p_2p_3=0\Rightarrow p_1=p_3$
- (2,2): $2p_2-p_3^2+1=0\Rightarrow p_3^2=3\Rightarrow p_3=\sqrt3$

*Step 3.* $P=\begin{bmatrix}\sqrt3&1\\1&\sqrt3\end{bmatrix}$ and $L=R^{-1}B^TP=[\,1\ \ \sqrt3\,]\approx[1,\ 1.73]$.

*Step 4.* Control law: $u=-1\cdot x-1.73\,\dot x$, closed loop $\ddot x+1.73\dot x+x=0$, so $\omega_n=1$, $\zeta=0.866$: stable and well damped.

---

# 6. Simulation and Results (Section "Simulation Results")

| Item | Paper value | Meaning |
|---|---|---|
| Path | Circle, fixed altitude 1 m | Test of $x,y$ tracking while $z$ is held |
| Start point | $p_0=(-1,0,0)$ | On the ground, on the circle's edge |
| Rate | 200 Hz | Controller updates every 5 ms |
| Effects included | Actuator limits, battery model | Closer to a real drone |
| Result | "High accuracy"; ≈1 s delay in X and Y | Delay because the drone must first climb to 1 m (altitude settling) |

- **Figs. 4–6:** X, Y, Z responses (reference vs actual). Fig. 7: the 3D path. Fig. 8: PWM signals of four rotors (ESC range 1000–2000 µs). Fig. 9: quaternion components, with $q_0\approx1$ and $q_1,q_2,q_3\approx0$ meaning the attitude stays near hover.
- The gain matrix $L$ was "tuned to be adaptive with the full non-linear model" (Eq. 8). In other words, the LQR gains came from the **linear** model (Eq. 17–18), and were then tuned in simulation on the **nonlinear** model.

---

# 7. Typos / Inconsistencies in the Printed Paper

Mention these only if asked; they show you checked the maths.

| # | Where | Printed | Correct / note |
|---|---|---|---|
| 1 | Eq. (3) | $q_2^2+q_3^2+q_4^2$ | $q_1^2+q_2^2+q_3^2$ (index slip) |
| 2 | After Eq. (3) | $q^{-1}=q^*/\lVert q\rVert$ | $q^*/\lVert q\rVert^2$ |
| 3 | Eq. (7), $\phi$ | $2(q_0q_1-q_2q_3)$ | $2(q_0q_1+q_2q_3)$ |
| 4 | Eq. (9), yaw | $\sum\tau_i(-1)^2$ | $\sum\tau_i(-1)^i$ |
| 5 | Eq. (10), $y$-row | $-2q_0q_2$ | $-2q_0q_1\approx-2q_1$ |
| 6 | Eq. (15), $\phi_d$ | $\ddot X_1\cos X_4-\ddot X_2\sin X_4$ | Z-Y-X derivation: $\ddot X_1\sin X_4-\ddot X_2\cos X_4$ |
| 7 | Eq. (23) text | "Riccati differential equation" | Eq. (23) is the **algebraic** Riccati equation |
| 8 | Eq. (22) text | "R and Q ... states and inputs" order | $Q$ → states, $R$ → inputs |
| 9 | Simulation | "1 m diameter", start $(-1,0,0)$ | Plots swing between −1 and +1, so the radius looks like 1 m |
| 10 | Fig. 8 text | PWM "1–2 sec" | Axis shows 1000–2000, i.e. 1–2 **ms** (µs units) |

---

# 8. Exam Points

## Key formulas to remember

| Topic | Formula |
|---|---|
| Quaternion | $q=q_0+\bar q$, $\ \lVert q\rVert=1$ |
| Product | $q\otimes r=(q_0r_0-\bar q\cdot\bar r)+(r_0\bar q+q_0\bar r+\bar q\times\bar r)$ |
| Conjugate / inverse | $q^*=q_0-\bar q$, $\ q^{-1}=q^*/\lVert q\rVert^2$ (unit: $q^{-1}=q^*$) |
| Rotate a vector | $v'=q\otimes v\otimes q^*=R_qv$, $\ R_q^{-1}=R_q^T$ |
| Axis–angle | $q=\cos\frac\alpha2+u\sin\frac\alpha2$ |
| Kinematics | $\dot q=\frac12q\otimes\omega$ |
| Translation | $\ddot p=q\otimes\frac Tm\otimes q^*+\bar g$ |
| Rotation | $J\dot\omega=\tau-\omega\times J\omega$ |
| Hover linear model | $\ddot x=g\,u_x,\ \ddot y=-g\,u_y,\ \ddot z=T_d/m,\ J\dot\omega=\tau$ |
| Flat outputs | $[x,y,z,\psi]^T$ |
| Desired pitch | $\theta_d=\arctan\frac{\ddot x\cos\psi+\ddot y\sin\psi}{\ddot z+g}$ |
| Desired attitude | $q_d=q_{pd}\otimes q_{zd}$ |
| LQR cost | $J=\frac12\int_0^\infty(x^TQx+u^TRu)dt$ |
| LQR law | $u=-Lx,\ L=R^{-1}B^TP$ |
| ARE | $A^TP+PA-PBR^{-1}B^TP+Q=0$ |

## Definitions

- **Underactuated:** fewer independent control inputs (4) than degrees of freedom (6).
- **Gimbal lock:** loss of one rotational degree of freedom when two Euler axes align (at $\theta=\pm90^\circ$).
- **Differentially flat:** all states and inputs are algebraic functions of some outputs and a finite number of their derivatives.
- **LQR:** optimal state-feedback controller that minimises a quadratic cost of state error and control effort.

## Common mistakes

1. Using non-unit quaternions with $q^{-1}=q^*$. This only holds when $\lVert q\rVert=1$.
2. Forgetting that $q\otimes r\neq r\otimes q$.
3. Mixing up $Q$ (states) and $R$ (inputs).
4. Forgetting the half-angles in $q=\cos\frac\alpha2+u\sin\frac\alpha2$.
5. Writing the ARE as a differential equation. For infinite horizon it is **algebraic**.
6. Saying "LQR is nonlinear". It is designed on a **linear** model (here, linearised at hover).

## Likely questions with short model answers

1. **Why quaternions?** No gimbal lock, 4 numbers with one constraint, cheap to compose and to invert (conjugate).
2. **How does $q\otimes v\otimes q^*$ rotate a vector?** Expanding gives Rodrigues' formula (Eq. 4c to Eq. 5), the same as multiplying by $R_q$.
3. **Where does $\dot q=\frac12q\otimes\omega$ come from?** A small rotation $\delta q\approx[1,\frac12\omega dt]$ is composed with $q$ (Eq. 6 derivation).
4. **How is the linear model obtained?** Hover point $q=[1,0,0,0],\ \omega=0,\ T=mg$; drop second-order terms in Eq. (8) to get Eq. (10), then Eq. (17), (18).
5. **What makes the quadrotor flat and why does it help?** The four outputs $x,y,z,\psi$ determine every state and input (Eq. 15), so planning happens in output space with simple formulas.
6. **Derive $\theta_d$.** Combine $\ddot x\cos\psi+\ddot y\sin\psi=\frac Tm\cos\phi\sin\theta$ with $\ddot z+g=\frac Tm\cos\phi\cos\theta$ and divide.
7. **Derive the LQR gain.** HJB with $V=\frac12x^TPx$ gives $u=-R^{-1}B^TPx$, and substituting back gives the ARE.
8. **What do $Q$ and $R$ do?** Larger $Q$ = tighter tracking, larger $R$ = gentler control.
9. **Why a double loop?** Position is controlled through tilt; the faster inner attitude loop must achieve the tilt that the outer position loop requests.
10. **Limitations?** Simulation only, one circular trajectory, linear controller designed at hover. The authors suggest nonlinear control such as backstepping.

---

# 9. Reference

Choutri, K., Lagha, M., Dala, L., & Lipatov, I. (2017). *Quadrotors trajectory tracking using a differential flatness-quaternion based approach.* Proceedings of the 2017 7th International Conference on Modeling, Simulation, and Applied Optimization (ICMSAO), IEEE. DOI: <https://doi.org/10.1109/ICMSAO.2017.7934901>
