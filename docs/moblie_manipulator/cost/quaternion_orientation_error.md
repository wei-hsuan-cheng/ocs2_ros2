# Quaternion Orientation Error Used by OCS2

This note derives the quaternion orientation residual used by OCS2 tracking costs. In this codebase it appears in two closely related places:

- [`quaternionDistance()`](../../../robotics/ocs2_robotic_tools/include/ocs2_robotic_tools/common/RotationTransforms.h) defines the 3D quaternion residual.
- [`FrameRelationTrackingCost`](../../../../mobile_manipulator_mpc/src/constraint/FrameRelationTrackingCost.cpp) stacks this residual after the relative-position residual for frame-relation tracking.

The important point is that this residual is a **quaternion-vector error**, not the exact SO(3) logarithm vector. For small attitude errors it is locally proportional to the SO(3) log error, with a factor of approximately $1/2$.

## Notation

Let the current and reference orientations be unit quaternions

$$
\mathbf{q} =
\begin{bmatrix}
\mathbf{q}_v \\
q_w
\end{bmatrix},
\qquad
\mathbf{q}_{\mathrm{ref}} =
\begin{bmatrix}
\mathbf{q}_{\mathrm{ref},v} \\
q_{\mathrm{ref},w}
\end{bmatrix},
$$

where

$$
\mathbf{q}_v, \mathbf{q}_{\mathrm{ref},v} \in \mathbb{R}^3,
\qquad
q_w, q_{\mathrm{ref},w} \in \mathbb{R},
\qquad
\|\mathbf{q}\| = \|\mathbf{q}_{\mathrm{ref}}\| = 1.
$$

The convention in the equations is vector-first:

$$
\mathbf{q} =
\begin{bmatrix}
q_x & q_y & q_z & q_w
\end{bmatrix}^T.
$$

This matches Eigen's coefficient order `q.coeffs() = [x, y, z, w]`.

## Residual Definition

OCS2 defines the orientation residual as

$$
\boxed{\;
\mathbf{e}_q(\mathbf{q}, \mathbf{q}_{\mathrm{ref}}) =
q_w \mathbf{q}_{\mathrm{ref},v} -
q_{\mathrm{ref},w}\mathbf{q}_v +
\mathbf{q}_v \times \mathbf{q}_{\mathrm{ref},v}
\;}
$$

This is exactly the expression implemented by `quaternionDistance(q, qRef)`:

$$
\mathbf{e}_q =
q_w \mathbf{q}_{\mathrm{ref},v} -
q_{\mathrm{ref},w}\mathbf{q}_v +
\mathbf{q}_v \times \mathbf{q}_{\mathrm{ref},v}.
$$

The residual is zero when the two orientations are aligned:

$$
\mathbf{e}_q = \mathbf{0}
\quad \Longleftrightarrow \quad
\mathbf{q} = \pm \mathbf{q}_{\mathrm{ref}},
$$

where the $\pm$ accounts for the usual quaternion double cover of SO(3).

## Derivation From Quaternion Multiplication

For vector-first quaternions, use the Hamilton product

$$
\begin{bmatrix}
\mathbf{a}_v \\
a_w
\end{bmatrix}
\otimes
\begin{bmatrix}
\mathbf{b}_v \\
b_w
\end{bmatrix} =
\begin{bmatrix}
a_w\mathbf{b}_v + b_w\mathbf{a}_v + \mathbf{a}_v \times \mathbf{b}_v \\
a_w b_w - \mathbf{a}_v^T\mathbf{b}_v
\end{bmatrix}.
$$

For a unit quaternion,

$$
\mathbf{q}^{-1} =
\begin{bmatrix} - \mathbf{q}_v \\
q_w
\end{bmatrix}.
$$

The relative quaternion that maps the current orientation to the reference orientation is

$$
\mathbf{q}_{\Delta} =
\mathbf{q}_{\mathrm{ref}} \otimes \mathbf{q}^{-1}.
$$

Substitute

$$
\mathbf{a}_v = \mathbf{q}_{\mathrm{ref},v}, \qquad
a_w = q_{\mathrm{ref},w}, \qquad
\mathbf{b}_v = -\mathbf{q}_v, \qquad
b_w = q_w.
$$

Then

$$
\begin{aligned}
\mathbf{q}_{\Delta,v} &=
q_{\mathrm{ref},w}(-\mathbf{q}_v) + q_w\mathbf{q}_{\mathrm{ref},v} + \mathbf{q}_{\mathrm{ref},v} \times (-\mathbf{q}_v) \\
&= - q_{\mathrm{ref},w}\mathbf{q}_v + q_w\mathbf{q}_{\mathrm{ref},v} - \mathbf{q}_{\mathrm{ref},v} \times \mathbf{q}_v \\ &= q_w\mathbf{q}_{\mathrm{ref},v} - q_{\mathrm{ref},w}\mathbf{q}_v + \mathbf{q}_v \times \mathbf{q}_{\mathrm{ref},v}.
\end{aligned}
$$

Therefore

$$
\boxed{\;
\mathbf{e}_q(\mathbf{q}, \mathbf{q}_{\mathrm{ref}}) =
\mathrm{vec}\!\left(
\mathbf{q}_{\mathrm{ref}} \otimes \mathbf{q}^{-1}
\right)
\;}
$$

where $\mathrm{vec}(\cdot)$ extracts the vector part of the quaternion.

<details>
<summary>Why the multiplication order matters</summary>

If the product is written in the opposite order,

$$
\mathbf{q}^{-1} \otimes \mathbf{q}_{\mathrm{ref}},
$$

then the vector part becomes

$$
\mathrm{vec}\!\left(
\mathbf{q}^{-1} \otimes \mathbf{q}_{\mathrm{ref}}
\right) =
q_w\mathbf{q}_{\mathrm{ref},v} - q_{\mathrm{ref},w}\mathbf{q}_v - \mathbf{q}_v \times \mathbf{q}_{\mathrm{ref},v}.
$$

The cross-product term has the opposite sign. The OCS2 implementation uses the residual from

$$
\mathbf{q}_{\mathrm{ref}} \otimes \mathbf{q}^{-1},
$$

which gives the plus sign in

$$
\mathbf{q}_v \times \mathbf{q}_{\mathrm{ref},v}.
$$

</details>

## Relation to the SO(3) Log Error

Let the rotation represented by the relative quaternion be

$$
R_{\Delta} =
R_{\mathrm{ref}} R^T.
$$

Its SO(3) logarithm is

$$
\boldsymbol{\phi} =
\log(R_{\Delta})^\vee =
\theta \hat{\mathbf{u}},
$$

where $\theta = \|\boldsymbol{\phi}\|$ and $\hat{\mathbf{u}}$ is the unit rotation axis.

The corresponding unit quaternion can be written as

$$
\mathbf{q}_{\Delta} =
\begin{bmatrix}
\hat{\mathbf{u}}\sin(\theta/2) \\
\cos(\theta/2)
\end{bmatrix}.
$$

Since OCS2 uses the vector part of $\mathbf{q}_{\Delta}$,

$$
\mathbf{e}_q =
\hat{\mathbf{u}}\sin(\theta/2).
$$

Using $\boldsymbol{\phi} = \theta\hat{\mathbf{u}}$,

$$
\boxed{\;
\mathbf{e}_q =
\frac{\sin(\theta/2)}{\theta}\boldsymbol{\phi} =
\frac{1}{2}\operatorname{sinc}(\theta/2)\boldsymbol{\phi}
\;}
$$

with the usual continuous limit at $\theta = 0$.

Therefore, for small attitude error,

$$
\sin(\theta/2) \approx \theta/2,
$$

so

$$
\boxed{\;
\mathbf{e}_q \approx \frac{1}{2}\boldsymbol{\phi} =
\frac{1}{2}\log(R_{\mathrm{ref}}R^T)^\vee
\;}
$$

This explains why the residual points in the same local rotation-error direction as the SO(3) log error, but is not numerically identical to it. The SO(3) log error has magnitude $\theta$, while the quaternion-vector residual has magnitude $\sin(\theta/2)$.

## Cost Term and YAML Weights

For frame-relation tracking, the residual stack is

$$
\mathbf{r} =
\begin{bmatrix}
\mathbf{p}_{st} - \mathbf{p}_{\mathrm{ref}} \\
\mathbf{e}_q(\mathbf{q}_{st}, \mathbf{q}_{\mathrm{ref}})
\end{bmatrix}
\in \mathbb{R}^6.
$$

The weighted least-squares cost is

$$
\ell =
\frac{1}{2}\mathbf{r}^T W \mathbf{r},
\qquad
W =
\operatorname{diag}
\left(
w_x,\; w_y,\; w_z,\;
w_{q_x},\; w_{q_y},\; w_{q_z}
\right).
$$

So the frame-relation tracking weight order in `mpc_solver.yaml` is

$$
\boxed{\;
\left[
\texttt{x},\; \texttt{y},\; \texttt{z},\;
\texttt{quat\_error\_x},\;
\texttt{quat\_error\_y},\;
\texttt{quat\_error\_z}
\right]
\;}
$$

The last three weights act on quaternion-vector residual components. They do not act on Euler angles, and they are not direct weights on the exact SO(3) log vector $\theta\hat{\mathbf{u}}$.

## Residual Jacobian

Let

$$
\mathbf{r}_v := \mathbf{q}_{\mathrm{ref},v},
\qquad
r_w := q_{\mathrm{ref},w}.
$$

The residual is

$$
\mathbf{e}_q =
q_w\mathbf{r}_v - r_w\mathbf{q}_v + \mathbf{q}_v \times \mathbf{r}_v.
$$

Using the skew matrix convention

$$
[\mathbf{r}_v]_{\times}\mathbf{a} =
\mathbf{r}_v \times \mathbf{a},
$$

we can rewrite the cross-product term as

$$
\mathbf{q}_v \times \mathbf{r}_v = -[\mathbf{r}_v]_{\times}\mathbf{q}_v.
$$

Therefore

$$
\mathbf{e}_q =
\left(-r_w I - [\mathbf{r}_v]_{\times}\right)\mathbf{q}_v + q_w\mathbf{r}_v.
$$

The Jacobian with respect to Eigen's quaternion coefficient order

$$
\begin{bmatrix}
q_x & q_y & q_z & q_w
\end{bmatrix}^T
$$

is

$$
\boxed{\;
\frac{\partial \mathbf{e}_q}
{\partial [q_x\;q_y\;q_z\;q_w]} =
\begin{bmatrix} -r_w & r_z & -r_y & r_x \\ -r_z & -r_w & r_x & r_y \\
r_y & -r_x & -r_w & r_z
\end{bmatrix}
\;}
$$

This is the matrix implemented by `quaternionDistanceJacobian(q, qRef)`.

For the frame-relation cost, this quaternion Jacobian is then chained with the local angular-velocity-to-quaternion derivative and the relative angular Jacobian:

$$
\frac{\partial \mathbf{e}_q}{\partial x} =
\frac{\partial \mathbf{e}_q}{\partial \mathbf{q}_{st}}
\frac{\partial \mathbf{q}_{st}}{\partial \boldsymbol{\omega}_{st}}
\frac{\partial \boldsymbol{\omega}_{st}}{\partial x}.
$$

This is why the final orientation cost derivative is compatible with the OCS2 state layout, even though the residual itself is defined in quaternion coordinates.

## Corresponding Cost Implementation

The equations above map directly to the frame-relation tracking cost implementation:

- Residual construction: [`FrameRelationTrackingCost::evaluateResidual()`](../../../../mobile_manipulator_mpc/src/constraint/FrameRelationTrackingCost.cpp#L346-L369)
- Weighted scalar cost: [`FrameRelationTrackingCost::getValue()`](../../../../mobile_manipulator_mpc/src/constraint/FrameRelationTrackingCost.cpp#L111-L121)
- Gauss-Newton quadratic approximation: [`FrameRelationTrackingCost::getQuadraticApproximation()`](../../../../mobile_manipulator_mpc/src/constraint/FrameRelationTrackingCost.cpp#L158-L164)
- Orientation Jacobian chain: [`FrameRelationTrackingCost::evaluateResidualJacobian()`](../../../../mobile_manipulator_mpc/src/constraint/FrameRelationTrackingCost.cpp#L445-L468)
- Quaternion residual and quaternion Jacobian: [`RotationTransforms.h`](../../../robotics/ocs2_robotic_tools/include/ocs2_robotic_tools/common/RotationTransforms.h#L49-L71)

The residual stack is formed as

```cpp
residual.head<3>() = sourceToTarget.translation() - reference.head<3>();
residual.tail<3>() = quaternionDistance<scalar_t>(orientation, targetOrientation);
```

which corresponds to

$$
\mathbf{r} =
\begin{bmatrix}
\mathbf{p}_{st} - \mathbf{p}_{\mathrm{ref}} \\
\mathbf{e}_q(\mathbf{q}_{st}, \mathbf{q}_{\mathrm{ref}})
\end{bmatrix}.
$$

The scalar cost is computed as

```cpp
totalCost += scalar_t(0.5) * residual.cwiseProduct(weights).dot(residual);
```

which corresponds to

$$
\ell =
\frac{1}{2}\mathbf{r}^T W\mathbf{r}.
$$

The Jacobian chain in the cost is implemented as

```cpp
const matrix_t relativeAngularJacobian =
    rotationWorldToSource * (targetJacobian.bottomRows<3>() - sourceJacobian.bottomRows<3>());

const matrix_t relativeOrientationJacobian =
    (quaternionDistanceJacobian(relativeOrientation, targetOrientation) *
     angularVelocityToQuaternionTimeDerivative(relativeOrientation)) *
    relativeAngularJacobian;
```

which is the code form of

$$
\frac{\partial \mathbf{e}_q}{\partial x} =
\frac{\partial \mathbf{e}_q}{\partial \mathbf{q}_{st}}
\frac{\partial \mathbf{q}_{st}}{\partial \boldsymbol{\omega}_{st}}
\frac{\partial \boldsymbol{\omega}_{st}}{\partial x}.
$$

## Practical Notes

- Normalize quaternions before evaluating the residual. The derivation assumes unit quaternions.
- Keep reference quaternion trajectories sign-continuous when possible. The cost is zero for $\mathbf{q} = \pm\mathbf{q}_{\mathrm{ref}}$, but abrupt quaternion sign flips can still change the residual sign and affect linearization.
- For small errors, a quaternion-error weight roughly acts on half of the SO(3) log error. If replacing this residual with a true SO(3) log residual, orientation weights generally need retuning.
- The residual is bounded by $\|\mathbf{e}_q\|\le 1$, while the SO(3) log error magnitude is $\theta$. This matters for large attitude errors.

## References

- [`RotationTransforms.h`](../../../robotics/ocs2_robotic_tools/include/ocs2_robotic_tools/common/RotationTransforms.h): implementation of `quaternionDistance()` and `quaternionDistanceJacobian()`.
- [`FrameRelationTrackingCost.cpp`](../../../../mobile_manipulator_mpc/src/constraint/FrameRelationTrackingCost.cpp): frame-relation residual stack and Gauss-Newton weighted least-squares cost.
