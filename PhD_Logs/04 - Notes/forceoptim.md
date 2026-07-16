# Grasp Force Distribution

objective: generate a reference for the internal loop force controller of the impedance tasks.

variable to optimize: normal contact forces

$$ x = [F_{n,L} F_{n_R}]^T $$

Motivation: optimizing just the internal forces, can create infinite distributions of $$ x = [F_{n,L} F_{n_R}]^T $$ that can lead to the same internal forces

## Constraints
for each contact:
### Friction
 - $$|| F_{t,L}|| \leq \mu_L F_{t,L}  $$ 
  - $$|| F_{t,R}|| \leq \mu_L F_{t,R}  $$  
tangential forces are already known from the sensors
### Positivity
 - $$F_{t,L} \geq F_{min}  $$ 
 - $$F_{t,R} \geq F_{min}  $$
to avoid the case of one arm pressing all forces and the other dont

## Internal Force
 - $$ P_{int} = I-G^+ G $$
 - $$ \lambda = P_{int}*[w_{L} w_{R}]^T $$
 - Squeeze direction: $$ n_s  $$ 
 - Null Space projection of squeeze: $$ n_squeeze = N * (N.transpose() * n_s); $$
 - Normalize: $$n_squeeze.stableNormalize()$$
 - $$ \lambda_{grasp} = n_squeeze.transpose() * \lambda $$


## Minimal Contact Force

### Static Contact Force
as fallback safety option

### On-Demand Force
since we use the quintic polynomal trajectory for a offline planning, we know beforehand what acceleration the object requires to move from the starting to the final point.
we can create an index of aggresivity of of the trajectory that regulate the minimal grasp

in our case:
$$ s(t) = 10 \tau^3-15\tau^4+6\tau^5  $$
with $$ \tau = t/T $$

the acceleration with respect to $$\tau$$
$$ \frac{d^2s}{d\tau^2} = 60\tau-180\tau^2+120\tau^3 $$
we can use this index $$ a_{demand} =  |60\tau-180\tau^2+120\tau^3| $$
that initially is zero, it grows during the trajectory and return zero when the trajectory is concluded.


$$ F_{demand} = k \cdot a_{demand}  $$

### friction
we take the force $$F_{Friction,L}$$ and $$F_{Friction,R}$$ that respect this constraints
$$ ||F_{t,L}|| \leq \mu_L F_{n,L}  $$
$$  ||F_{t,R}|| \leq \mu_L F_{n,R} $$



### Minimal Force 
$$F_{min} = \max \left( \frac{||F_{t}||}{\mu} , F_{static} + F_{demand}\right)$$


## Cost Function

$$ J =  \underbrace{\alpha (\lambda_{grasp})^2}_{minimum Squeeze force} +
        \underbrace{\beta (F_{n,L}-F_{n,R})^2}_{Balancing}  
         $$

## Final Problem

$$ \min_{w_{n,L},w_{n,R}} J $$
subject to:
$$ F_{n,L} \geq F_{min} $$
$$ F_{n,R} \geq F_{min} $$

 




# Grasp Force Optimization QP

## Overview

This module implements a small standalone quadratic programming (QP) optimizer for regulating the normal forces applied by two robot contacts (e.g. left and right gripper/hand).

The optimizer computes the desired normal contact forces:

\[
x =
\begin{bmatrix}
F_{n,L}\\
F_{n,R}
\end{bmatrix}
\]

while satisfying minimum force requirements and optimizing:

- internal grasp force (squeeze force)
- force balance between contacts

This QP runs independently from the main `mc_rtc` whole-body QP.

---

# Optimization Problem

The optimization variable is:

\[
x =
\begin{bmatrix}
F_{n,L}\\
F_{n,R}
\end{bmatrix}
\]

where:

- \(F_{n,L}\): left contact normal force
- \(F_{n,R}\): right contact normal force

The problem is formulated as:

\[
\min_x J
\]

with:

\[
J =
\alpha(\lambda_{grasp})^2
+
\beta(F_{n,L}-F_{n,R})^2
\]

where:

- \(\alpha\): weight on internal squeeze force
- \(\beta\): weight on force balancing

---

# Internal Force

The internal force component is extracted using the null-space projection:

\[
P_{int}=I-G^+G
\]

where \(G\) is the grasp matrix.

The internal force vector is:

\[
\lambda =
P_{int}
\begin{bmatrix}
w_L\\
w_R
\end{bmatrix}
\]

The squeeze direction is projected into the internal-force space:

\[
n_{squeeze}=N(N^Tn_s)
\]

and normalized:

\[
n_{squeeze}.stableNormalize()
\]

The grasp force is:

\[
\lambda_{grasp}
=
n_{squeeze}^T\lambda
\]

---

# Minimum Contact Force

The minimum required normal force is computed from friction and trajectory demands.

## Friction Constraint

The contact forces must satisfy:

\[
||F_t||\leq\mu F_n
\]

which gives:

\[
F_n\geq\frac{||F_t||}{\mu}
\]

---

## Dynamic Force Demand

For a quintic trajectory:

\[
s(t)=10\tau^3-15\tau^4+6\tau^5
\]

where:

\[
\tau=\frac{t}{T}
\]

the normalized acceleration profile is:

\[
\frac{d^2s}{d\tau^2}
=
60\tau-180\tau^2+120\tau^3
\]

The trajectory aggressiveness index is:

\[
a_{demand}
=
|60\tau-180\tau^2+120\tau^3|
\]

The additional required force is:

\[
F_{demand}=k\,a_{demand}
\]

---

## Final Minimum Force

For each contact:

\[
F_{min}
=
\max
\left(
\frac{||F_t||}{\mu},
F_{static}+F_{demand}
\right)
\]

---

# QP Formulation

The solver uses the standard form:

\[
\min_x
\frac12x^THx+g^Tx
\]

## Hessian Matrix

The balancing term:

\[
\beta(F_{n,L}-F_{n,R})^2
\]

gives:

\[
H_b=
2\beta
\begin{bmatrix}
1&-1\\
-1&1
\end{bmatrix}
\]

The squeeze term:

\[
\alpha(\lambda_{grasp})^2
\]

is converted into:

\[
H_s=2\alpha a^Ta
\]

where:

\[
a=n_{squeeze}^TP_{int}
\]

The final Hessian is:

\[
H=H_s+H_b
\]

The linear term is:

\[
g=0
\]

---

# Constraints

The minimum force constraints are:

\[
F_{n,L}\geq F_{min}
\]

\[
F_{n,R}\geq F_{min}
\]

Converted to QP inequality form:

\[
Ax\leq b
\]

with:

\[
A=
\begin{bmatrix}
-1&0\\
0&-1
\end{bmatrix}
\]

and:

\[
b=
\begin{bmatrix}
-F_{min}\\
-F_{min}
\end{bmatrix}
\]

---

# Implementation

The optimizer uses the standalone Tasks QP backend:

```cpp
#include <Tasks/QPSolver.h>