**State** status of agent wrt the environment. Specifically Location in grid world.

**State Space** The set of all states 
$$
\mathcal{S} = \{s_i\}^9_{i=1}
$$


![image-20260213085911072](C:\Users\15244\AppData\Roaming\Typora\typora-user-images\image-20260213085911072.png)

**Action** For each states there are some possible actions

**Action space of a state** The set of all possible actions of state. Note that the action space are related on certain state. For different state the action space might be different.
$$
\mathcal{A}(s_i)=\{a_i\}^5_{i=1}
$$
**State transition** when taking an action, the agent may from one state to another. 

For example for $s_1$, if $a_2$ represents moving to right place by one step, then $s_1\xrightarrow{a_2} s_2$

**Forbidden area** We choose the general and challenging case, where the forbidden area is accessible but with penalty.

**State transition probability** Use probability to describe state transition (not a one-hot)

Using conditional probability to describe 
$$
p(s_2|s_1,a_2)=0.5 \\
p(s_1|s_1,a_2)=0.5
$$
refers to: taking $a_2$ from $s_1$, it gets 0.5 chance to move to $s_1$ while 0.5 chance to $s_2$

**Policy** tells the agent what actions to take at a state. $\pi(a_i|s_j)=p$: at $s_j$ the agent takes action $a_i$ with $p$ probability. Sum of all probability for one state should be 1. 

In coding part we do uniform distribution and assign an action for each section.

**Reward** A real number. We get after taking an action. Can be interpreted as a *Human-machine interface*, with which we can guide the agent to behave as we expected.

"+" represents encouragement while "-" represents punishment.

Similarly, we could using conditional probability to represents it.
$$
p(r=-1|s_1,a_1) = 0.7,\ p(r=+1|s_1,a_1) = 0.3
$$
**Trajectory** is a state-action-reward chain
$$
s_1\xrightarrow[r=0]{a_2}s_2\xrightarrow[r=0]{a_3}s_5\xrightarrow[r=0]{a_3}s_8\xrightarrow[r=1]{a_2}s_9
$$
**Return** of a trajectory is the sum of all rewards collected along the trajectory.
$$
\text{return}=0+0+0+1=1
$$
Return could be used to evaluate whether a policy is good or not.

A trajectory may be infinite: For example in our case when we arrive $s_9$, agent might stay at  that place and accumulate rewards.
$$
s_8\xrightarrow[r=1]{a_2}s_9\xrightarrow[r=1]{a_2}s_9\xrightarrow[r=1]{a_2}\ldots \\
\text{return}=0+0+0+1+1+1+\ldots =\infty
$$
**Discounting rate** $\gamma\in[0,1)$

**Discounting return** = $0_+\gamma0+\gamma^20+\gamma^31+\gamma^41+\ldots$

1) The sum becomes finite. 
2) Balance the far and near future rewards: 
   1) if $\gamma$ is close to 0, the value of the discounted return is dominated by the rewards obtained in the near future. ***Short-sighted***
   2) if $\gamma$ is close to 1, the value of the discounted return is dominated by the rewards obtained in the far future. ***Long-sighted***

**Episode(trail)** When interacting with the environment following a policy, the agent may stop at some terminal states. The resulting trajectory is called an episode or a trail. An episode is usually assumed to be a finite trajectory. Tasks with episodes are called episodic tasks.

Some tasks may have no terminal states. Such tasks are called **continuing tasks**



***Markov Decision Process (MDP)***

- **Sets:**

  - **State:** the set of states $S$
  - **Action:** the set of actions $A(s)$ is associated for state $s \in S$
  - **Reward:** the set of rewards $\mathcal{R}(s,a)$

- **Probability distribution:**

  - **State transition probability:** at state $s$, taking action $a$, the probability to transit to state $s'$ is $p(s' \mid s,a)$
  - **Reward probability:** at state $s$, taking action $a$, the probability to get reward $r$ is $p(r \mid s,a)$

- **Policy:** at state $s$, the probability to choose action $a$ is $\pi(a \mid s)$

- **Markov property (memoryless property):**
  $$
  p(s_{t+1}\mid a_t,s_t,\ldots,a_1,s_0)=p(s_{t+1}\mid a_t,s_t)
  $$

  $$
  p(r_{t+1}\mid a_t,s_t,\ldots,a_1,s_0)=p(r_{t+1}\mid a_t,s_t)
  $$

 **Markov Decision Process** becomes **Markov Process** is the policy $\pi$ is Given.







