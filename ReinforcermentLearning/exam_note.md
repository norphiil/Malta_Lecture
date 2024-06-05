# Reinforcement Learning Exam Study Notes

## Introduction to Reinforcement Learning

1. **What is Reinforcement Learning?**
   - **Definition:** RL is a type of machine learning where an agent interacts with an environment, taking actions to maximize cumulative reward over time.
   - **Components:**
     - **Agent:** The decision maker.
     - **Environment:** The external system the agent interacts with.
     - **Actions (A):** Choices available to the agent.
     - **States (S):** Situations or configurations of the environment.
     - **Rewards (R):** Numerical feedback from the environment.

2. **Reward Signals**
   - **Definition:** Rewards are numerical values that indicate the immediate benefit of an action.
   - **Purpose:** They serve as feedback for the agent to understand the desirability of its actions.

3. **RL Applications**
   - **Robotics:** For learning complex tasks like grasping objects.
   - **Gaming:** AI agents that learn to play games like Chess, Go, and video games (e.g., AlphaGo, Dota 2).
   - **Autonomous Vehicles:** For navigation and decision-making.
   - **Finance:** For portfolio management and trading.
   - **Healthcare:** Personalized treatment strategies.

4. **Formalization of the RL Framework**
   - **MDP:** Consists of a set of states \( S \), a set of actions \( A \), a reward function \( R(s, a) \), and a transition probability function \( P(s' | s, a) \).

5. **States and Policies**
   - **State (s):** A representation of the current situation in the environment.
   - **Policy (π):** A strategy that specifies the action \( a \) to take in state \( s \). Can be deterministic (π(s) = a) or stochastic (π(a | s) = probability of taking action \( a \) in state \( s \)).

6. **State and Action Value Functions**
   - **State Value Function (V(s)):** Expected return (cumulative future reward) starting from state \( s \) and following policy \( π \).
   - **Action Value Function (Q(s, a)):** Expected return starting from state \( s \), taking action \( a \), and then following policy \( π \).

7. **Types of RL Agents**
   - **Model-Free:** Learn directly from interactions with the environment (e.g., Q-Learning, SARSA).
   - **Model-Based:** Use a model of the environment for planning (e.g., Dyna-Q).
   - **On-Policy:** Learn the value of the policy being executed (e.g., SARSA).
   - **Off-Policy:** Learn the value of an optimal policy while following another policy (e.g., Q-Learning).

## Multi-armed Bandits

1. **Introduction to Multi-armed Bandits**
   - **Problem Definition:** An agent must choose between multiple options (arms) with unknown reward distributions to maximize total reward over time.

2. **Action Values and Regret**
   - **Action Value (Q(a)):** Expected reward of taking action \( a \).
   - **Regret:** The difference between the reward of the best possible action and the reward of the chosen action over time.

3. **Exploration vs Exploitation**
   - **Exploration:** Trying different actions to gather more information.
   - **Exploitation:** Choosing the best-known action to maximize immediate reward.

4. **Action Value Estimates**
   - Methods to estimate action values include sample averages, Bayesian methods, and other statistical approaches.

5. **Greedy and ε-Greedy Algorithms**
   - **Greedy:** Always selecting the action with the highest estimated value.
   - **ε-Greedy:** Selecting the best-known action with probability \( 1 - ε \) and exploring randomly with probability \( ε \).

6. **Upper-Confidence Bound (UCB)**
   - **Algorithm:** Selects actions based on the upper confidence bound of the estimated action values, balancing exploration and exploitation.

7. **Gradient Bandit Algorithms**
   - **Method:** Uses gradient ascent on expected reward to improve policy. Each action has a preference which is updated based on received rewards.

## Markov Decision Processes (MDPs)

1. **Definition of Markov Decision Processes**
   - **MDP:** A formal framework to describe decision-making problems, characterized by states \( S \), actions \( A \), transition probabilities \( P(s' | s, a) \), and rewards \( R(s, a) \).

2. **Markov Property**
   - **Definition:** The future state depends only on the current state and action, not on the sequence of events that preceded it.

3. **Discount Factors**
   - **Discount Factor (γ):** A value between 0 and 1 that determines the present value of future rewards. A higher \( γ \) values future rewards more equally to immediate rewards.

4. **The Bellman Optimality Equation**
   - **Equations:** Recursive definitions that provide the foundation for calculating the optimal policy and value functions.

5. **Value Function Approximation**
   - **Purpose:** Used when the state space is too large to compute the exact value function. Methods include linear, non-linear, and deep learning-based approximations.

6. **Extensions to MDPs**
   - **POMDPs:** Partially Observable MDPs where the agent has incomplete information about the state.
   - **Hierarchical MDPs:** Decomposing the problem into smaller MDPs to handle complex tasks.

## Dynamic Programming

1. **Overview of Dynamic Programming (DP)**
   - **Concept:** Solving complex problems by breaking them down into simpler subproblems. Key methods include policy evaluation, policy iteration, and value iteration.

2. **Policy Evaluation**
   - **Objective:** Calculate the state value function for a given policy \( π \).

3. **Policy Iteration**
   - **Process:** Alternates between policy evaluation (calculating \( V(s) \) for a policy) and policy improvement (updating the policy based on \( V(s) \)) until convergence.

4. **Value Iteration**
   - **Algorithm:** Iteratively updates the value function using the Bellman optimality equation until it converges to the optimal value function.

## Monte Carlo Methods

1. **Introduction to Model-free Learning**
   - **Concept:** Learning methods that do not require a model of the environment. They estimate value functions based on sample episodes of experience.

2. **Monte Carlo Prediction**
   - **Objective:** Estimate the value function based on averaging returns obtained from sampled episodes.

3. **On-Policy vs Off-Policy Learning**
   - **On-Policy:** Learning the value of the policy currently being followed.
   - **Off-Policy:** Learning the value of a target policy while following a different behavior policy.

4. **Model-free Policy Iteration**
   - **Concept:** Combining policy evaluation and policy improvement without using a model.

5. **On-Policy Monte Carlo Control**
   - **Algorithm:** Uses Monte Carlo methods to improve the policy based on sampled episodes. The policy is updated to be greedy with respect to the value function. An example algorithm is the **every-visit Monte Carlo**, where each state-action pair’s value is updated every time it is visited in an episode.

6. **Importance Sampling for Off-Policy Learning**
   - **Concept:** Adjusts for the difference between the target policy (the one being learned) and the behavior policy (the one generating the data). Importance sampling corrects the discrepancy by weighting the returns by the probability ratio of the target and behavior policies.

## Temporal-Difference Methods

1. **Overview of Temporal-Difference (TD) Learning**
   - **Concept:** Combines ideas from Monte Carlo methods and dynamic programming. It updates estimates based partly on actual rewards and partly on estimated future values.

2. **TD(0) Prediction**
   - **Algorithm:** Updates the value of the current state based on the observed reward and the estimated value of the next state. The update rule is:
     \[
     V(s) \leftarrow V(s) + \alpha [R_{t+1} + \gamma V(s_{t+1}) - V(s_t)]
     \]
   - **α:** Learning rate.
   - **γ:** Discount factor.

3. **TD vs Monte Carlo (MC)**
   - **TD:** Updates after each step, leading to faster learning and the ability to learn from incomplete episodes.
   - **MC:** Updates only after the end of an episode, which can be less efficient and slower.

4. **TD and MC Learning with Limited Experience**
   - **Efficiency:** TD methods can learn more efficiently from limited experience by updating after each step, while MC methods may require many episodes to converge.

5. **On-Policy TD Control with SARSA**
   - **SARSA:** An on-policy TD control algorithm where the next action is taken according to the current policy:
     \[
     Q(s_t, a_t) \leftarrow Q(s_t, a_t) + \alpha [R_{t+1} + \gamma Q(s_{t+1}, a_{t+1}) - Q(s_t, a_t)]
     \]

6. **Off-Policy TD Control with Q-Learning**
   - **Q-Learning:** An off-policy TD control algorithm that updates action values using the maximum action value of the next state:
     \[
     Q(s_t, a_t) \leftarrow Q(s_t, a_t) + \alpha [R_{t+1} + \gamma \max_a Q(s_{t+1}, a) - Q(s_t, a_t)]
     \]

7. **Maximization Bias and Double Q-Learning**
   - **Maximization Bias:** Occurs when the Q-learning update rule tends to overestimate action values.
   - **Double Q-Learning:** Uses two value functions to mitigate the bias, updating one with the value of the other:
     \[
     Q_A(s_t, a_t) \leftarrow Q_A(s_t, a_t) + \alpha [R_{t+1} + \gamma Q_B(s_{t+1}, \arg\max_a Q_A(s_{t+1}, a)) - Q_A(s_t, a_t)]
     \]

8. **Expected SARSA**
   - **Expected SARSA:** A variation of SARSA that uses the expected value over all possible actions for the next state:
     \[
     Q(s_t, a_t) \leftarrow Q(s_t, a_t) + \alpha [R_{t+1} + \gamma \sum_a \pi(a|s_{t+1})Q(s_{t+1}, a) - Q(s_t, a_t)]
     \]

## Function Approximation

1. **Function Approximation for Scalability**
   - **Purpose:** Makes RL applicable to large or continuous state spaces by approximating value functions with parameterized models.

2. **Gradient Descent**
   - **Method:** An optimization technique to minimize the error in the function approximation. The parameters are updated in the direction that reduces the error.

3. **Value Function Approximation for MC and TD Methods**
   - **Techniques:** Linear and non-linear methods can be used to approximate value functions, enabling the application of MC and TD methods to large state spaces.

4. **State Features**
   - **Concept:** Representing states with features, which are functions of the state that capture important properties and make the learning process more efficient.

5. **Linear Function Approximation**
   - **Method:** Approximating the value function as a linear combination of state features:
     \[
     \hat{V}(s, \mathbf{w}) = \mathbf{w}^T \mathbf{\phi}(s)
     \]
   - **\(\mathbf{w}\):** Weight vector.
   - **\(\mathbf{\phi}(s)\):** Feature vector of state \( s \).

6. **Batch Methods**
   - **Concept:** Learning from a batch of experiences, which can stabilize learning and improve sample efficiency.

7. **Non-linear Function Approximation**
   - **Method:** Using non-linear models like neural networks for more powerful and flexible function approximation, allowing for the handling of complex state spaces.

## Policy Gradient

1. **Policy-based Reinforcement Learning**
   - **Concept:** Directly parameterizes the policy and optimizes it to maximize expected reward, especially useful for continuous action spaces.

2. **Objective Functions for Policy-based RL Algorithms**
   - **Objective:** Maximizing the expected return by adjusting the policy parameters.

3. **Policy Gradient**
   - **Method:** Uses gradient ascent to update policy parameters in the direction that increases expected return.

4. **REINFORCE Algorithm**
   - **Algorithm:** A Monte Carlo policy gradient method that updates policy parameters based on the return from sampled episodes:
     \[
     \mathbf{\theta} \leftarrow \mathbf{\theta} + \alpha G_t \nabla_{\mathbf{\theta}} \log \pi_{\mathbf{\theta}}(a_t|s_t)
     \]

5. **Actor-Critic Methods**
   - **Concept:** Combines policy gradient (actor) and value function (critic) to reduce variance in updates:
     - **Actor:** Updates the policy parameters.
     - **Critic:** Evaluates the policy by estimating value functions.

6. **RL for Continuous Action Spaces**
   - **Methods:** Policy gradient methods like DDPG (Deep Deterministic Policy Gradient) and PPO (Proximal Policy Optimization) handle continuous action spaces efficiently.

## Reinforcement Learning and Planning

1. **Model-Based Reinforcement Learning**
   - **Concept:** Uses models of the environment to simulate and plan actions, improving sample efficiency.

2. **Learning Models using RL**
   - **Methods:** Techniques like supervised learning to approximate the transition and reward functions from collected data.

3. **Dyna Algorithms**
   - **Integration:** Combines model-free and model-based methods by using simulated experiences from the model to update the policy.

4. **Forward Search Algorithms**
   - **Concept:** Planning methods that simulate future actions to make better decisions.

5. **Monte Carlo Tree Search (MCTS)**
   - **Algorithm:** A search method for decision making in large spaces, combining Monte Carlo sampling and tree search.
