# NAIAD

### Named-Axis Decomposition of Idle Actuation in a Learned Controller for Over-Actuated Underwater Vehicles

**Read what a learned underwater controller asks of the hull, and account for the thrust that cancels within its actuators.**

Tianze Zhang · Lei Wu · Xiaowen Tao

China University of Petroleum (East China) · Trinity College Dublin

[Overview](#overview) · [Method](#method) · [Results](#results) · [Experimental-setup](#experimental-setup) · [Citation](#citation)

## Overview

An over-actuated underwater vehicle has more thrusters than degrees of freedom. Classical control separates the six-axis demand from the allocation of that demand across thrusters. An end-to-end learned controller emits thruster commands directly, leaving both the demand and the use of actuator redundancy implicit.

**NAIAD** audits a trained controller through three complementary operations:

- **Resolve the demand:** project eight thruster commands onto surge, sway, heave, roll, pitch, and yaw using the vehicle's geometry.
- **Read the control laws:** express each recovered axis as a sum of signed univariate Kolmogorov-Arnold network (KAN) terms over recorded quantities.
- **Account for idle actuation:** replay a bounded minimum-norm command sequence through the same stateful actuator to separate the actuator baseline, policy residual, and their interaction.

The controller checkpoint remains unchanged throughout the analysis. The study evaluates a DreamerV3 world-model controller on a simulated BlueROV Heavy-class vehicle tracking a lemniscate in both directions.

## Method

### 1. Geometry projection before the actuator

Let $\mathbf{a}_t\in[-1,1]^8$ be the normalized thruster command and $\mathbf{B}\in\mathbb{R}^{6\times8}$ the configuration matrix derived from the vehicle geometry. The named-axis demand is

$$
\mathbf{u}_t=\mathbf{B}\mathbf{a}_t.
$$

This projection acts **before** the actuator response, keeping deadzone, forward-reverse asymmetry, and actuator filtering out of the target transformation. The demand is expressed in command-derived coordinates; it is not a measurement of delivered force or torque.

With $\mathbf{P}=\mathbf{I}-\mathbf{B}^{\dagger}\mathbf{B}$, the command decomposes as

$$
\mathbf{a}_t=\mathbf{B}^{\dagger}\mathbf{u}_t+\mathbf{P}\mathbf{a}_t.
$$

The studied layout has rank six and a two-dimensional null space. Its two idle modes are supported on the horizontal and vertical thruster groups, respectively.

### 2. Strictly additive named-axis readout

Each axis is modeled by a single additive layer with no hidden units:

$$
\hat{u}_j(\mathbf{x})=b_j+\sum_{i=1}^{59}\psi_{j,i}(x_i),
\qquad j=1,\ldots,6.
$$

Each edge combines a SiLU base response with a cubic B-spline. The resulting **354 individually readable terms** describe how recorded quantities contribute to the recovered demands. Leading curves can be compressed into low-order polynomials, hinge expressions, and threshold clauses.

| Evidence block | Dimensions | Contents |
| --- | ---: | --- |
| Instantaneous state and reference | 31 | Tracking errors, velocities, reference acceleration, angular rate, gravity direction, realized throttle, reference speed and yaw rate, episode phase |
| Previous command | 8 | Thruster commands from the preceding step |
| Causal history | 17 | Exponential averages of errors, angular rate, and commands |
| Error integral | 3 | Clipped position-error integrals |
| **Total** | **59** | **Named quantities available from recorded operation** |

History features use preceding samples and reset at episode boundaries. Latent states are not readout inputs. To construct the training target, the frozen controller is queried with 32 posterior samples at each executed state; the random-generator state is then restored before advancing the rollout.

### 3. Counterfactual attribution of idle thrust

At each recorded step, solve

$$
\mathbf{a}^{\min}_t=
\underset{-\mathbf{1}\leq\mathbf{a}'\leq\mathbf{1}}{\arg\min}
\|\mathbf{a}'\|_2^2
\quad\text{subject to}\quad
\mathbf{B}\mathbf{a}'=\mathbf{B}\mathbf{a}_t.
$$

Replay the actual and counterfactual sequences through the same nonlinear, stateful actuator from the same initial state. Project the resulting thrust vectors into the null space to obtain the actual idle thrust $\mathbf{n}_t$ and baseline $\mathbf{n}^{\mathrm{base}}_t$. With $\mathbf{r}_t=\mathbf{n}_t-\mathbf{n}^{\mathrm{base}}_t$,

$$
\sum_t\|\mathbf{n}_t\|_2^2
=\sum_t\|\mathbf{n}^{\mathrm{base}}_t\|_2^2
+\sum_t\|\mathbf{r}_t\|_2^2
+2\sum_t\langle\mathbf{n}^{\mathrm{base}}_t,\mathbf{r}_t\rangle.
$$

Attribution is performed on vectors before normalization, retaining the interaction term. The counterfactual preserves the **command-space demand**; nonlinear actuator dynamics mean this does not by itself guarantee identical delivered wrench or closed-loop motion.

## Results

The following values are reported on held-out episodes in the manuscript.

| Finding | Reported result |
| --- | ---: |
| Total idle thrust energy / total actual thrust energy | **11.88%** |
| Actuator baseline / total actual thrust energy | **2.03%** |
| Policy residual / total actual thrust energy | **9.92%** |
| Baseline-residual interaction / total actual thrust energy | **−0.06%** |
| Energy-identity closure error | **1.28 × 10⁻⁸** |
| Maximum additive-accounting discrepancy | **8.3 × 10⁻⁷** |
| Closed-form agreement for the selected edges in Table 5 | **≥ 0.991** |
| Threshold-clause agreement reported in the manuscript text | **≥ 0.994** |

*Energy shares are independently rounded, so displayed percentages need not sum exactly.*

- **Idle actuation is concentrated in the vertical thrusters.** The policy residual is approximately 4.9 times the actuator baseline under the specified counterfactual.
- **Recent actuation dominates the recovered demands.** Realized throttle and previous commands contribute more strongly than tracking-error terms across the six axes.
- **The readout remains consistent across traversal directions.** Direction-specific evaluation reports similar recovery behavior for clockwise and counterclockwise traversal of the reference.

**Metric interpretation.** Exact additive accounting means the terms sum to the surrogate's own output; it does not imply exact prediction of the original controller. Closed-form and clause agreement measure approximation of individual learned edges. The paper's thrust-energy metric is a sum of squared thrust norms, not measured electrical energy or demonstrated battery savings.

## Experimental Setup

| Item | Configuration |
| --- | --- |
| Vehicle | BlueROV Heavy-class; 8 thrusters, 6 degrees of freedom |
| Controller | Frozen DreamerV3 checkpoint |
| Task | Bidirectional lemniscate tracking in simulation |
| Recorded evidence | 128 episodes; 14,177 transitions |
| Control interval | 16 ms |
| Posterior samples per transition | 32 |
| Data partition | Whole-episode training, validation, and held-out splits |
| Readout | 59 inputs, 6 outputs, 354 edges, 4,608 parameters |
| Splines | Cubic; 8 grid intervals |
| Input normalization | Median and interquartile range; clipping at 4 interquartile ranges |
| Optimizer | AdamW; learning rate 0.003 |
| Training limit | 2,000 epochs; patience 360 |
| Implementation described in the paper | PyTorch and an additive KAN layer |
| Reported hardware | Intel Core i9-14900K; NVIDIA RTX 4090 |
| Reported operating system | Ubuntu 24.04 |

## Reproduction Workflow

The paper describes the following analysis sequence:

1. Load the vehicle geometry, frozen controller checkpoint, and simulator actuator model.
2. Collect episode logs and posterior-averaged demand targets without modifying the controller or changing executed rollouts.
3. Construct the 59 causal features and partition complete episodes into training, validation, and held-out sets.
4. Fit the additive readout and retain the checkpoint selected by validation performance.
5. Export signed contribution plots, edge curves, compact expressions, and threshold clauses.
6. Solve the bounded minimum-norm allocation and replay both command sequences through the stateful actuator.
7. Check the replay against recorded thrust, verify energy closure, and evaluate both traversal directions separately.

## Scope and Future Work

The reported evidence concerns one simulated vehicle layout, one controller checkpoint, and its trained tracking task. Physical-vehicle validation, comparisons across actuator layouts, and onboard monitoring with recovered clauses and idle-actuation metrics are identified as future work.

## Citation

Bibliographic entry based on the supplied manuscript; publication metadata can be added when available.

```bibtex
@misc{zhang_naiad,
  title  = {{NAIAD}: Named-Axis Decomposition of Idle Actuation in a Learned Controller for Over-Actuated Underwater Vehicles},
  author = {Zhang, Tianze and Wu, Lei and and Tao, Xiaowen},
}
```
