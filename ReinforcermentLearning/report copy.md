# An Overview and Analysis of Reinforcement Learning Algorithms for Blackjack

## Introduction

Reinforcement Learning (RL) is a type of machine learning where an agent learns to make decisions by taking actions in an environment to maximize cumulative reward. In this study, we apply RL algorithms to the game of Blackjack. We implemented and compared the performance of Monte Carlo, SARSA, and Q-learning algorithms. Each algorithm's performance was measured based on win, draw, and loss counts over multiple episodes.

## Background

### Blackjack

Blackjack is a popular card game where the goal is to have a hand value as close to 21 as possible without exceeding it. The game involves a player and a dealer, with the player making decisions based on their hand and the dealer's visible card.

### Reinforcement Learning Algorithms

1. **Monte Carlo Methods**: These methods rely on averaging sample returns to learn value functions and policies. They are simple and effective for episodic tasks.
2. **SARSA (State-Action-Reward-State-Action)**: An on-policy TD control algorithm that updates Q-values based on the action taken.
3. **Q-Learning**: An off-policy TD control algorithm that updates Q-values using the maximum future reward, regardless of the action taken by the agent.

## Implementation

### Overview of Algorithms

The project implements three reinforcement learning (RL) algorithms for playing the game of Blackjack: Monte Carlo Control with exploring starts, SARSA (State-Action-Reward-State-Action), and Q-Learning. Each algorithm follows a distinct approach to learning optimal policies through interaction with the environment.

1. **Monte Carlo Control with Exploring Starts**:
   - **Exploring Starts**: This technique ensures exploration by starting episodes with both states and actions sampled uniformly.
   - **Epsilon-Greedy Policy**: During action selection, it balances exploration (random actions) and exploitation (greedy actions based on learned Q-values) based on a specified epsilon parameter.
   - **Q-Value Update**: Q-values are updated after completing episodes using the average return observed for each state-action pair encountered.

2. **SARSA (State-Action-Reward-State-Action)**:
   - **Epsilon-Greedy Policy**: Similar to Monte Carlo, SARSA uses an epsilon-greedy policy to balance exploration and exploitation.
   - **On-Policy Update**: Q-values are updated after each action using the current action and the next action derived from the policy.
   - **Terminal State Handling**: Adjustments are made in terminal states to ensure the correct calculation of Q-values based on rewards and next state values.

3. **Q-Learning**:
   - **Epsilon-Greedy Policy**: Q-Learning also employs an epsilon-greedy policy but learns from the maximum Q-value of the next state rather than the action actually taken.
   - **Off-Policy Update**: Q-values are updated irrespective of the action actually chosen, based on the maximum Q-value of the next state.
   - **Comparison with SARSA**: Q-Learning tends to converge to the optimal policy faster than SARSA due to its off-policy nature but may overestimate Q-values in certain scenarios.

### Implementation Details

Each algorithm is encapsulated within a respective class inheriting from `PlayerRLAgent`, which extends the `Player` class. This design allows for modular implementation and testing of different RL algorithms against the same environment.

- **PlayerRLAgent Class**:
  - Manages Q-tables (`q_table` and `q_table_counts`) to store and update Q-values for state-action pairs.
  - Defines methods for action selection (`choose_action`) based on epsilon-greedy policies and updating Q-values (`update_q_table`) based on rewards and next states.

- **MonteCarloOnPolicyControl Class**:
  - Inherits from `PlayerRLAgent`.
  - Implements Monte Carlo Control with methods specific to handling returns, exploring starts, and updating Q-values accordingly.

- **SARSAOnPolicyControl Class**:
  - Inherits from `PlayerRLAgent`.
  - Implements SARSA with methods for updating Q-values based on state transitions and rewards, considering both exploration and exploitation.

- **QLearningOffPolicyControl Class**:
  - Inherits from `SARSAOnPolicyControl`.
  - Extends SARSA by using an off-policy approach to update Q-values, learning from the maximum Q-value of the next state-action pair.

### Algorithm Configurations

Each algorithm instance is configured with specific parameters such as learning rate (`alpha`), discount factor (`gamma`), and exploration rate (`epsilon`). These parameters are crucial in determining how aggressively the agent explores new actions versus exploiting known good actions.

### Exploration vs Exploitation Trade-offs

Exploration and exploitation are fundamental to the success of RL algorithms in Blackjack:
- **Exploration**: Ensures the agent explores various actions and states, preventing premature convergence to suboptimal policies.
- **Exploitation**: Exploits learned knowledge (Q-values) to make optimal decisions based on past experiences.

## Experiment Setup

### Parameters

- **Alpha (α)**: Learning rate, set to 0.1.
- **Gamma (γ)**: Discount factor, set to 0.9.
- **Epsilon (ε)**: Exploration rate, set to 0.1.

### Configurations

We tested each algorithm with different epsilon configurations:

1. **1/k**: Decreases over time.
2. **e^(-k/1000)**: Exponentially decreases over time.
3. **e^(-k/10000)**: Slower exponential decrease.

### Metrics

Performance was measured by counting wins, draws, and losses over 100000 episodes. The results were plotted and analyzed to understand the learning behavior and stability of each algorithm.

## Results and Analysis

### Monte Carlo Method

#### Win-Loss-Darw Counts

![Monte Carlo (1/k) Results](plots/line_mc_f_1k.png)
![Monte Carlo (e^(-k/1000)) Results](plots/line_mc_f_-k1000.png)
![Monte Carlo (e^(-k/10000)) Results](plots/line_mc_f_-k10000.png)

The Monte Carlo method with `1/k` showed a steady learning curve but required many episodes to converge. The `e^(-k/1000)` and `e^(-k/10000)` configurations converged faster, with the former being slightly more aggressive in exploration. This indicates that the rate of exploration decay plays a significant role in the speed of convergence and overall performance of the algorithm.

#### State-Action Pair Counts

![State-Action Pair Counts (Ace) - MC 1/k](plots/count_true_mc_f_1k.png)
![State-Action Pair Counts (No Ace) - MC 1/k](plots/count_false_mc_f_1k.png)

The state-action pair counts reveal the frequency of each action taken in different states. This provides insight into the learned policy and the effectiveness of the exploration strategy.

### SARSA

#### Win-Loss-Darw Counts

![SARSA (1/k) Results](plots/line_sarsa_1k.png)
![SARSA (e^(-k/1000)) Results](plots/line_sarsa_-k1000.png)
![SARSA (e^(-k/10000)) Results](plots/line_sarsa_-k10000.png)

SARSA's performance was stable across all configurations. The `1/k` configuration showed a gradual improvement, while `e^(-k/1000)` and `e^(-k/10000)` converged faster. This suggests that SARSA, being an on-policy algorithm, benefits from a balance between exploration and exploitation, which is effectively managed by the epsilon decay strategies.

#### State-Action Pair Counts

![State-Action Pair Counts (Ace) - SARSA 1/k](plots/count_true_sarsa_1k.png)
![State-Action Pair Counts (No Ace) - SARSA 1/k](plots/count_false_sarsa_1k.png)

The state-action pair counts for SARSA further illustrate the balance between exploration and exploitation. The distribution of actions across states shows the learned policy's adaptability to different game scenarios.

### Q-Learning

#### Win-Loss-Darw Counts

![Q-Learning (1/k) Results](plots/line_qlearn_1k.png)
![Q-Learning (e^(-k/1000)) Results](plots/line_qlearn_-k1000.png)
![Q-Learning (e^(-k/10000)) Results](plots/line_qlearn_-k10000.png)

Q-learning showed the fastest convergence, particularly with the `e^(-k/1000)` configuration. The `1/k` configuration, while effective, was slower compared to exponential decay strategies. This rapid convergence can be attributed to Q-learning's off-policy nature, which allows it to learn from the maximum Q-value of the next state-action pair, irrespective of the action actually taken.

#### State-Action Pair Counts

![State-Action Pair Counts (Ace) - Q-Learning 1/k](plots/count_true_qlearn_1k.png)
![State-Action Pair Counts (No Ace) - Q-Learning 1/k](plots/count_false_qlearn_1k.png)

The state-action pair counts for Q-learning highlight the aggressive nature of its learning strategy. The distribution of actions across states shows a strong bias towards certain actions, indicating a high degree of exploitation.

### Strategy Tables

The strategy tables provide a visual representation of the optimal actions in different states as learned by each algorithm. These tables serve as a valuable tool for understanding the learned policies and their implications on the game strategy.

#### Monte Carlo Strategy Table

![Strategy Table with Ace - MC](plots/strategy_table_with_ace_mc_f_1k.png)
![Strategy Table No Ace - MC](plots/strategy_table_no_ace_mc_f_1k.png)

#### SARSA Strategy Table

![Strategy Table with Ace - SARSA](plots/strategy_table_with_ace_sarsa_1k.png)
![Strategy Table No Ace - SARSA](plots/strategy_table_no_ace_sarsa_1k.png)

#### Q-Learning Strategy Table

![Strategy Table with Ace - Q-Learning](plots/strategy_table_with_ace_qlearn_1k.png)
![Strategy Table No Ace - Q-Learning](plots/strategy_table_no_ace_qlearn_1k.png)

## Discussion

### Exploration vs. Exploitation

The balance between exploration and exploitation significantly impacted each algorithm's performance. Exponential decay (`e^(-k/1000)` and `e^(-k/10000)`) allowed for faster convergence by reducing exploration more quickly compared to the `1/k` method. This resulted in quicker stabilization of policies but might risk local optima.

In terms of algorithm comparison, Monte Carlo, being suitable for problems where episodes can be fully simulated, showed effective but slow convergence. SARSA, being an on-policy algorithm, demonstrated reliable and stable learning with moderate convergence speed. Q-Learning, being an off-policy algorithm, showed fast convergence, especially with exponential decay strategies, but can be less stable.

### Algorithm Comparison

- **Monte Carlo**: Suitable for problems where episodes can be fully simulated. Effective but slow convergence.
- **SARSA**: Reliable and stable learning with moderate convergence speed. Suitable for on-policy learning.
- **Q-Learning**: Fast convergence, especially with exponential decay strategies, but can be less stable. Best for off-policy learning.

## Conclusion

This study demonstrates the application of Monte Carlo, SARSA, and Q-learning algorithms in Blackjack. Each algorithm's configuration significantly affects its learning performance. Exponential decay exploration strategies generally lead to faster convergence. Future work can explore combining these methods or using more advanced techniques like Deep Q-Learning for further improvements. The results of this study provide valuable insights into the dynamics of reinforcement learning algorithms and their application in complex environments like Blackjack.

## References

1. [Geiser, J., & Hasseler, T. (n.d.). **Beating Blackjack - A Reinforcement Learning Approach**. Stanford University. This research seeks to develop various learning algorithms for Blackjack play and to verify common strategic approaches to the game. He present these implementation of Sarsa, and Q-Learning.](https://web.stanford.edu/class/aa228/reports/2020/final117.pdf)

2. [Avish Buramdoyal, Tim Gebbie. (2023). **Variations on the Reinforcement Learning performance of Blackjack**. *arXiv:2308.07329 [cs.AI]*. Retrieved from arXiv:2308.07329](https://arxiv.org/abs/2308.07329v1)
