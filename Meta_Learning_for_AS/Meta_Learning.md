# Meta_Learning

- 可以和LPV的一些方法作比较。

## Why LPV = Meta-learning structure (conceptually)

An LPV system is:
$$
x_{k+1} = A(\theta) x_k + B(\theta) u_k
$$
where
$$
\theta \in \mathcal{P}
$$
is a scheduling parameter
 (e.g. mass, stiffness, damping…)

Each value of $\theta$ gives you:
$$
f_\theta(x,u)
$$
→ a different dynamics
→ a different optimal controller

So each $\theta$ corresponds to:
$$
\text{one task}
$$

| LPV                            | Meta-learning               |
| ------------------------------ | --------------------------- |
| Parameterized dynamics         | Task distribution           |
| Scheduling variable ( \theta ) | Task identity               |
| Gain scheduling                | Fast adaptation             |
| Interpolation between models   | Learning prior across tasks |

**In control language**

LPV solves:

> Given $\theta$, design $K(\theta)$

Meta-learning solves:

> Learn how to quickly get near-optimal $K_\theta$
>  from small data — without full ID

