## Motivating examples - Why is return important?

Two important phrases

- A core concept: state value
- A fundamental tool: the Bellman equation

![image-20260213101130946](C:\Application\TasksList\EPFL_related\Typora_Doc\Reinforcement Learning\assets\image-20260213101130946.png)

*While return is important, how to calculate it?*

Method 1: by definition

Let $v_i$ denote the return obtained starting from $s_i$ $(i=1,2,3,4)$
$$
v_1 = r_1 + \gamma r_2 + \gamma^2 r_3 + \cdots\\
v_2 = r_2+ \gamma r_3 + \gamma^2 r_4 + \cdots \\
v_3 = r_3 + \gamma r_4 + \gamma^2 r_1 + \cdots \\
v_4 = r_4 + \gamma r_1 + \gamma^2 r_2 + \cdots
$$
Method 2: 
$$
v_1 = r_1 + \gamma v_2 \\
v_2 = r_2 + \gamma v_3 \\
v_3 = r_3 + \gamma v_4 \\
v_4 = r_4 + \gamma v_1 \\
$$
The returns rely on each other, which is called ***Bootstrapping!***

How to solve these equations? Write in the following matrix-vector form:
$$
\begin{bmatrix}
v_1 \\
v_2 \\
v_3 \\
v_4
\end{bmatrix}
=
\begin{bmatrix}
r_1 \\
r_2 \\
r_3 \\
r_4
\end{bmatrix}
+
\begin{bmatrix}
\gamma v_2 \\
\gamma v_3 \\
\gamma v_4 \\
\gamma v_1
\end{bmatrix}
=
\begin{bmatrix}
r_1 \\
r_2 \\
r_3 \\
r_4
\end{bmatrix}
+
\gamma
\begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
1 & 0 & 0 & 0
\end{bmatrix}
\begin{bmatrix}
v_1 \\
v_2 \\
v_3 \\
v_4
\end{bmatrix}
$$
which can be rewritten as
$$
v = r + \gamma P v
$$
That's what called the Bellman equation for this specific deterministic problem.

Core idea:  the value of one state relies on the values of the other states.



## State Value

**Notations:**

Consider the following single-step process:
$$
S_t \xrightarrow{A_t} R_{t+1},\; S_{t+1}
$$

- $t, t+1$: discrete time instances
- $S_t$: state at time $t$
- $A_t$: the action taken at state $S_t$
- $R_{t+1}$: the reward obtained after taking $A_t$
- $S_{t+1}$: the state transited to after taking $A_t$

Note that $S_t,; A_t,; R_{t+1}$ are all random variables.

This step is governed by the following probability distributions:

- $S_t \rightarrow A_t$ is governed by $\pi(A_t = a \mid S_t = s)$
- $S_t, A_t \rightarrow R_{t+1}$ is governed by $p(R_{t+1} = r \mid S_t = s, A_t = a)$
- $S_t, A_t \rightarrow S_{t+1}$ is governed by $p(S_{t+1} = s' \mid S_t = s, A_t = a)$

At this moment, we assume we know the model (i.e., the probability distributions).

Consider the following multi-step trajectory:
$$
S_t \xrightarrow{A_t} R_{t+1}, S_{t+1}
\xrightarrow{A_{t+1}} R_{t+2}, S_{t+2}
\xrightarrow{A_{t+2}} R_{t+3}, \dots
$$
The discounted return is
$$
G_t = R_{t+1} + \gamma R_{t+2} + \gamma^2 R_{t+3} + \cdots
$$

- $\gamma \in [0,1)$ is a discount rate.
- $G_t$ is also a random variable since $R_{t+1}, R_{t+2}, \dots$ are random variables.

***State Value:*** the expectation (or called expected value or mean) of $G_t$ is defined as the state-value function or simply state value (depends on state and policy):
$$
v_\pi(s) = \mathbb{E}[G_t \mid S_t = s]
$$
Remarks:

- It is a function of $s$. It is a conditional expectation with the condition that the state starts from $s$.
- It is based on the policy $\pi$. For a different policy, the state value may be different.
- It represents the **“value”** of a state. If the state value is greater, then the policy is better because greater cumulative rewards can be obtained. 值越大证明这个状态是越有价值的。

Q: What is the relationship between return and state value?

A: The state value is the mean of all possible returns that can be obtained starting from a state. If everything — $\pi(a \mid s)$, $p(r \mid s,a)$, $p(s' \mid s,a)$ — is deterministic, then state value is the same as return.

就是说State Value 是对所有possible trajectories的return求的期望值。也就是说是统计意义的中心。

## Bellman equation: Derivation

Answers how to calculate State Value.

**Derivation**

Consider a random trajectory:
$$
S_t \xrightarrow{A_t} R_{t+1}, S_{t+1}
\xrightarrow{A_{t+1}} R_{t+2}, S_{t+2}
\xrightarrow{A_{t+2}} R_{t+3}, \dots
$$
The return $G_t$ can be written as
$$
\begin{aligned}
G_t =& R_{t+1} + \gamma R_{t+2} + \gamma^2 R_{t+3} + \cdots\\
=& R_{t+1} + \gamma\bigl(R_{t+2} + \gamma R_{t+3} + \cdots\bigr)\\
=& R_{t+1} + \gamma G_{t+1}
\end{aligned}
$$
Then, it follows from the definition of the state value that
$$
\begin{aligned}
v_\pi(s) =& \mathbb{E}[G_t \mid S_t = s]\\
=& \mathbb{E}[R_{t+1} + \gamma G_{t+1} \mid S_t = s]\\
=& \mathbb{E}[R_{t+1} \mid S_t = s] + \gamma\mathbb{E}[G_{t+1} \mid S_t = s].
\end{aligned}
$$
Next, calculate the two terms, respectively.

First, calculate the <u>first term</u>  $\mathbb{E}[R_{t+1} \mid S_t = s]$:
$$
\begin{aligned}
\mathbb{E}[R_{t+1} \mid S_t = s]
&= \sum_a \pi(a \mid s)\mathbb{E}[R_{t+1} \mid S_t = s, A_t = a] \\
&= \sum_a \pi(a \mid s)\sum_r p(r \mid s,a)r
\end{aligned}
$$
Note that this is the mean of *immediate rewards*.

Second, calculate the <u>second term</u> $\mathbb{E}[G_{t+1} \mid S_t = s]$:
$$
\begin{aligned}
\mathbb{E}[G_{t+1} \mid S_t = s]
&= \sum_{s'} \mathbb{E}[G_{t+1} \mid S_t = s, S_{t+1} = s']p(s' \mid s) \\
&= \sum_{s'} \mathbb{E}[G_{t+1} \mid S_{t+1} = s']p(s' \mid s) \\
&= \sum_{s'} v_\pi(s')p(s' \mid s) \\
&= \sum_{s'} v_\pi(s') \sum_a p(s' \mid s,a)\pi(a \mid s)
\end{aligned}
$$
Note that this is the mean of *future rewards*. $\mathbb{E}[G_{t+1} \mid S_t = s, S_{t+1} = s'] = \mathbb{E}[G_{t+1} \mid S_{t+1} = s']$ due to the memoryless Markov property.

Those together lead to ==Bellman Equation==

Therefore, we have
$$
\begin{aligned}
v_\pi(s)
&= \mathbb{E}[R_{t+1} \mid S_t = s] + \gamma\mathbb{E}[G_{t+1} \mid S_t = s] \\
&= \sum_a \pi(a \mid s)\sum_r p(r \mid s,a)r+\gamma \sum_a \pi(a \mid s)\sum_{s'} p(s' \mid s,a)v_\pi(s') \\
&= \sum_a \pi(a \mid s)
\left[
\sum_r p(r \mid s,a)r+\gamma \sum_{s'} p(s' \mid s,a)v_\pi(s')
\right], \quad \forall s \in S.
\end{aligned}
$$
Highlights:

- The above equation is called the Bellman equation, which characterizes the relationship among the state-value functions of different states.
- It consists of two terms: the immediate reward term and the future reward term.
- A set of equations: every state has an equation like this.

Some Symbols

- $v_\pi(s)$ and $v_\pi(s')$ are state values to be calculated. Bootstrapping!
- $\pi(a \mid s)$ is a given policy. Solving the equation is called ==policy evaluation==.
- $p(r \mid s,a)$ and $p(s' \mid s,a)$ represent the ==dynamic model==. What if the model is known or unknown?

后两行是依赖的概率。如果能把state value 计算出来，就是在做policy evaluation。

知道model 和不知道model都有算法。

For a given state, different policies generally produce different state values because they induce different future trajectories. To evaluate which policy is better, we compare their expected long-term returns. If the initial state is fixed at $s_0$, a policy $\pi_1$ is better than $\pi_2$ if $v_{\pi_1}(s_0) > v_{\pi_2}(s_0)$. If the initial state is drawn from a distribution $\rho_0$, we compare the expected value
$$
J(\pi) = \mathbb{E}_{s_0 \sim \rho_0}[v_\pi(s_0)],
$$
which represents the average long-term performance starting from typical initial conditions. In this sense, the quality of a policy is determined by the expected cumulative return it generates from the initial state (or initial state distribution), rather than by the values of all states individually. 策略的好坏取决于它带来的长期回报，而我们通常比较的是从初始状态（或初始分布）出发的 state value。

计算例子略

## Bellman Equation: Matrix-vector Form.

Elementwise-form:
$$
\begin{aligned}
v_\pi(s)
&= \sum_a \pi(a \mid s)
\left[
\sum_r p(r \mid s,a)r+\gamma \sum_{s'} p(s' \mid s,a)v_\pi(s')
\right], \quad \forall s \in S.
\end{aligned}
$$
Rewrite the Bellman equation as
$$
v_\pi(s) = r_\pi(s) + \gamma \sum_{s'} p_\pi(s' \mid s) v_\pi(s')
$$
where
$$
r_\pi(s) \triangleq \sum_a \pi(a \mid s)\sum_r p(r \mid s,a)r,
\qquad
p_\pi(s' \mid s) \triangleq \sum_a \pi(a \mid s)p(s' \mid s,a).
$$


Suppose the states could be indexed as $s_i$ $(i = 1, \dots, n)$.

For state $s_i$, the Bellman equation is
$$
v_\pi(s_i)=r_\pi(s_i)+\gamma \sum_{s_j} p_\pi(s_j \mid s_i) v_\pi(s_j)
$$
Put all these equations for all the states together and rewrite in a matrix-vector form
$$
v_\pi = r_\pi + \gamma P_\pi v_\pi
$$
where

- $v_\pi = [v_\pi(s_1), \dots, v_\pi(s_n)]^T \in \mathbb{R}^n$
- $r_\pi = [r_\pi(s_1), \dots, r_\pi(s_n)]^T \in \mathbb{R}^n$
- $P_\pi \in \mathbb{R}^{n \times n}$, where $[P_\pi]_{ij} = p_\pi(s_j \mid s_i)$, is the state transition matrix.

