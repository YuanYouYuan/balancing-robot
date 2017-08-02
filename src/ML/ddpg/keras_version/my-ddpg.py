import random
import numpy as np
import gym

from collections        import namedtuple
from keras.layers       import Input, Dense, Activation, Lambda
from keras.optimizers   import Adam
from keras.regularizers import l2
from keras              import models
from keras              import layers
from keras              import backend as K

L2            = 0.01
HIDDEN        = 200
CAPACITY      = 1000000
N_EPISODE     = 1000
BATCH_SIZE    = 64
GAMMA         = 0.99
TAU           = 0.001
ENV           = 'Pendulum-v0'
USE_BATCHNORM = False
CLIP_VALUE    = 40

class ActorNetwork(object):

    # deterministic policy: f(states) -> action

    def __init__(self, input_dim: 'state', output_dim: 'action', action_bound, use_batchnorm):

        self.input_dim     = input_dim
        self.output_dim    = output_dim
        self.action_bound  = action_bound
        self.use_batchnorm = use_batchnorm

        self.build_network()
    
    def build_network(self):
        
        # network: 𝜇(state) -> continuous action
        # loss: policy gradient, mean(d_Q(s, a)/d_a * d_𝜇(s)/d_𝜃) + L2_reg

        states = Input(shape=(self.input_dim,), name='states')

        # layer1 
        net = Dense(units=HIDDEN, kernel_regularizer=l2(L2))(states)
        if USE_BATCHNORM:
            net = layers.BatchNormalization()(net)
        net = Activation('relu')(net)

        # layer2 
        net = Dense(units=HIDDEN, kernel_regularizer=l2(L2))(net)
        if USE_BATCHNORM:
            net = layers.BatchNormalization()(net)
        net = Activation('relu')(net)

        # layer3 
        net = Dense(units=self.output_dim, kernel_regularizer=l2(L2))(net)
        actions = Activation('tanh')(net)
        actions = Lambda(lambda x: x * self.action_bound)(actions)

        self.model = models.Model(inputs=states, outputs=actions) 

        Q_grad_a = Input(shape=(self.output_dim,))
        loss = K.mean(-Q_grad_a * actions)

        for l2_regularizer_loss in self.model.losses:
            loss += l2_regularizer_loss

        op = Adam(clipvalue=CLIP_VALUE)
        updates_op = op.get_updates(params=self.model.trainable_weights, 
                                    constraints=self.model.constraints, 
                                    loss=loss)

        self.train_fn = K.function(inputs=[self.model.input, Q_grad_a, K.learning_phase()], 
                                   outputs=[], 
                                   updates=updates_op)

class CriticNetwork(object):
    
    # f(state, action) -> Q_value

    def __init__(self, input_dim: 'state', output_dim: 'action', use_batchnorm):
        
        self.input_dim     = input_dim
        self.output_dim    = output_dim
        self.use_batchnorm = use_batchnorm

        self.build_network()

    def build_network(self):

        # 1: states -> fc -> (bn) -> ReLU -> fc
        # 2: actions -> fc
        # 3: merge 1, 2 -> (bn) -> ReLU ->
        # 4: 3 -> fc (= Q_pred)

        # Q_grad_a = d_Q_pred/d_action, which is required for ActorNetwork

        states = Input(shape=(self.input_dim,), name='states')
        actions = Input(shape=(self.output_dim,), name='actions')

        # layer 1: states -> fc -> (bn) -> ReLU
        net = Dense(units=HIDDEN, kernel_regularizer=l2(L2))(states)
        if USE_BATCHNORM:
            net = layers.BatchNormalization()(net)
        net = Activation("relu")(net)

        # Layer 2:
        # Merge[Layer1 -> fc, actions -> fc] -> (bn) -> ReLU
        states_out = Dense(units=HIDDEN, kernel_regularizer=l2(L2))(net)
        actions_out = Dense(units=HIDDEN, kernel_regularizer=l2(L2))(actions)
        net = layers.Add()([states_out, actions_out])
        if USE_BATCHNORM:
            net = layers.BatchNormalization()(net)
        net = Activation("relu")(net)


        # Layer 3:
        Q_pred = Dense(units=1, kernel_regularizer=l2(L2))(net)
        
        Q_grad_a = K.gradients(Q_pred, actions)
        self.model = models.Model(inputs=[states, actions], outputs=Q_pred)

        op = Adam()
        self.model.compile(optimizer=op, loss='mse')
        self.get_Q_grad_a = K.function(inputs=[*self.model.input, K.learning_phase()], outputs=Q_grad_a)

def soft_update(local_model, target_model, tau):
    
    # θ_target = τ * θ_local + (1 - τ) * θ_target
    
    local_weights  = np.array(local_model.get_weights())
    target_weights = np.array(target_model.get_weights())

    assert len(local_weights) is len(target_weights)

    target_model.set_weights(tau*local_weights + (1-tau)*target_weights)

    


class ReplayBuffer(object):
    
    def __init__(self, capacity):
        self.capacity = capacity
        self.position = 0
        self.buffer = []

    def push(self, s, a, r, d, s2):

        # s : state       , n x input_dim
        # a : action      , n x output_dim
        # r : reward      , n x 1
        # d : done        , boolean
        # s2: next state  , n x input_dim
        
        if len(self.buffer) < self.capacity:
            self.buffer.append(None)

        a = np.squeeze(a)
        s = np.ravel(s)
        s2 = np.ravel(s2)

        Transition = namedtuple('Transition', field_names=['state', 'action', 'reward', 'done', 'next_state'])
        self.buffer[self.position] = Transition(s, a, r, d, s2)
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size):
        return random.sample(self.buffer, k=batch_size)
    
    def __len__(self):
        return len(self.buffer)


class OUNoise:

    # Ornstein-Uhlenbeck process
    # state = noise

    def __init__(self, action_dim, mu=0.0, theta=0.15, sigma=0.3, seed=123):
        self.action_dim = action_dim
        self.mu         = mu
        self.theta      = theta
        self.sigma      = sigma
        self.state      = np.ones(self.action_dim) * self.mu
        self.reset()
        np.random.seed(seed)

    def reset(self):
        self.state      = np.ones(self.action_dim) * self.mu

    def noise(self):
        x = self.state
        dx = self.theta * (self.mu - x) + self.sigma * np.random.randn(len(x))
        self.state = x + dx
        return self.state
        
class Agent(object):

    # initialize local and target network
    # get actions
    # update parameters

    def __init__(self, input_dim, output_dim, action_bound=1.0, use_batchnorm=True):

        # 4 netwroks in total
        # local and target critic network
        # local and target actor  network

        self.input_dim     = input_dim
        self.output_dim    = output_dim
        self.action_bound  = action_bound
        self.use_batchnorm = use_batchnorm

        self.local_critic  = CriticNetwork(self.input_dim, self.output_dim, self.use_batchnorm)
        self.target_critic = CriticNetwork(self.input_dim, self.output_dim, self.use_batchnorm)

        self.local_actor   = ActorNetwork(self.input_dim, self.output_dim, self.action_bound, self.use_batchnorm)
        self.target_actor  = ActorNetwork(self.input_dim, self.output_dim, self.action_bound, self.use_batchnorm)

        self.noise = OUNoise(self.output_dim)

    def get_actions(self, states):
        states = np.reshape(states, [-1, self.input_dim])
        actions = self.local_actor.model.predict(states)
        return actions + self.noise.noise()

    def update(self, transitions, gamma, tau):

        # y = r + γ * target_critic(s2, target_actor(s2))

        states  = np.vstack([t.state for t in transitions if t is not None])

        actions = np.array([t.action for t in transitions if t is not None ])
        actions = actions.astype(np.float32).reshape(-1, self.output_dim)

        rewards = np.array([t.reward for t in transitions if t is not None])
        rewards = rewards.astype(np.float32).reshape(-1, 1)

        dones   = np.array([t.done for t in transitions if t is not None])
        dones   = dones.astype(np.float32).reshape(-1, 1)

        next_states = np.vstack([t.next_state for t in transitions if t is not None])
        
        # for next states and target network
        # actor(states) -> actions
        # critic(states, actions) -> values
        target_actions = self.target_actor.model.predict_on_batch(next_states)
        target_values  = self.target_critic.model.predict_on_batch([next_states, target_actions])

        y = rewards + gamma * target_values * (1 - dones)

        assert target_values.shape[1] == 1, target_values.shape
        assert y.shape[1] == 1, y.shape

        self.local_critic.model.train_on_batch([states, actions], y)

        Q_grad_a = self.local_critic.get_Q_grad_a([states, actions, 0])
        Q_grad_a = np.reshape(Q_grad_a, (-1, self.output_dim))

        assert Q_grad_a.shape[1] == self.output_dim, Q_grad_a.shape

        self.local_actor.train_fn([states, Q_grad_a, 1])

        soft_update(self.local_critic.model, self.target_critic.model, tau)
        soft_update(self.local_actor.model, self.target_actor.model, tau)

    def initialize_target(self):
        self.target_critic.model.set_weights(self.local_critic.model.get_weights())
        self.target_actor.model.set_weights(self.local_actor.model.get_weights())
        

def train_episode(env, agent, replay_buffer, batch_size, gamma, tau):
    
    # return total reward from this episode

    s = env.reset()
    done = False

    total_reward = 0

    while not done:
        a = agent.get_actions(s)
        s2, r, done, info = env.step(a)
        replay_buffer.push(s, a, r, done, s2)

        total_reward += r

        if len(replay_buffer) > batch_size:
            transition_batch = replay_buffer.sample(batch_size)
            agent.update(transition_batch, gamma, tau)

        s = s2

        if done:
            return total_reward




def main():

    try:
        env = gym.make(ENV)
        env = gym.wrappers.Monitor(env, directory='monitors', force=True)

        input_dim    = env.observation_space.shape[0]
        output_dim   = env.action_space.shape[0]
        action_bound = env.action_space.high
        
        print(env.observation_space)
        print(env.action_space)

        agent = Agent(input_dim, output_dim, action_bound, USE_BATCHNORM)
        agent.initialize_target()

        replay_buffer = ReplayBuffer(CAPACITY)

        for ep in range(N_EPISODE):
            reward = train_episode(env, agent, replay_buffer, BATCH_SIZE, GAMMA, TAU)
            print(ep, reward)

    finally:
        env.close()



if __name__ == '__main__':
    main()
