import chainer
import chainer.functions as F
import chainer.links as L
import chainerrl
import numpy as np

class QFunction(chainer.Chain):
    def __init__(self, obs_size, n_actions, n_hidden_channels = 40):
        super().__init__(
            l0 = L.Linear(obs_size, n_hidden_channels),
            l1 = L.Linear(n_hidden_channels, n_hidden_channels),
            l2 = L.Linear(n_hidden_channels, n_hidden_channels),
            l3 = L.Linear(n_hidden_channels, n_actions))

    def __call__(self, x, test = False):
        h = F.leaky_relu(self.l0(x))
        h = F.leaky_relu(self.l1(h))
        h = F.leaky_relu(self.l2(h))
        return chainerrl.action_value.DiscreteActionValue(self.l3(h))

class DoubleDQN:
    def __init__(self):
        # hyper parameters
        self.hps = {}
        
        # environment paramsters
        self.hps['obs_size'] = 2  # Dimension of State
        self.hps['n_actions'] = 2 # Dimension of Input
        self.hps['n_episodes'] = 1000
        self.hps['max_episode_len'] = 5000

        # DQN parameters
        self.hps['eps'] = 1e-2
        self.hps['n_hidden_channels'] = 40
        self.hps['n_hidden_layers'] = 2
        self.hps['gamma'] = 0.95
        self.hps['capacity'] = 1e6
        self.hps['start_epsilon'] = 0.9
        self.hps['end_epsilon'] = 0.05
        self.hps['decay_steps'] = 5e5
        self.hps['replay_start_size'] = 500
        self.hps['update_interval'] = 1
        self.hps['target_upda'] = 100

        self.agent_files
        self.n_agent = len(agent_files)
        
        self.agents = []
        for _ in range(n_agent):
            q_func = FCStateQFunctionWithDiscreteAction(
                ndim_obs=hps['obs_size'], n_actions=hps['n_actions'],
                n_hidden_channels=hps['n_hidden_channels'],
                n_hidden_layers=hps['n_hidden_layers'], nonlinearity=F.tanh)

            optimizer = chainer.optimizers.Adam(eps=hps['eps'])
            optimizer.setup(q_func)
 
            explorer = chainerrl.explorers.LinearDecayEpsilonGreedy(
                start_epsilon=hsps['start_epsilon'], end_epsilon=hps['end_epsilon'],
                decay_steps=hps['decay_steps'], random_action_func = lambda: np.random.randint(9))
            
            replay_buffer = chainerrl.replay_buffer.PrioritizedReplayBuffer()

            agent = chainerrl.agents.DoubleDQN(
                q_func, optimizer, replay_buffer, gamma=hps['gamma'], explorer,
                replay_start_size=hps['replay_start_size'],
                update_frequency=hps['update_interval'],
                target_update_interval=hps['target_update_interval'], phi=phi)
            self.agents.append(agent)


    def learn(self):
        total_step = 0

        obs_log = []
        reward_log = []
        
        for i in range(1, hps['n_episodes'] + 1):
            reward = 0
            while True:
                obs_log.append(obs)
                reward_log.append(reward)

                # choose action
                action = agents[0].act_and_train(obs, reward)

                # take action and observe reward and state(obs)

                
                if :
                    reward = 
                    agents[turn].stop_episode_and_train(, reward, True)
                else:
                    turn = 1 if turn == 0 else 0
        

if __name__ == '__main__':
    pass
