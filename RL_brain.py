# coding:utf-8
import numpy as np
import tensorflow as tf
import random
import math
import operator

np.random.seed(1)
tf.set_random_seed(1)


class DQN:
    def __init__(
            self,
            n_features,
            n_actions_l=5,
            n_actions_r=5,
            learning_rate=0.01,
            reward_decay=0.9,
            e_greedy=0.9,
            replace_target_iter=100,
            memory_size=500,
            batch_size=32,
            e_greedy_increment=None,
            output_graph=False,
    ):
        self.n_actions_l = n_actions_l
        self.n_actions_m = 1
        self.n_actions_r = n_actions_r
        self.n_actions = n_actions_l + 1 + n_actions_r
        self.n_features = n_features
        self.n_left = 18
        self.n_mid = 18
        self.n_right = 18
        self.n_state = self.n_features + self.n_right + self.n_mid + self.n_left
        self.lr = learning_rate
        self.gamma = reward_decay
        self.epsilon_max = e_greedy
        self.replace_target_iter = replace_target_iter
        self.memory_size = memory_size
        self.batch_size = batch_size
        self.epsilon_increment = e_greedy_increment
        self.epsilon = 0 if e_greedy_increment is not None else self.epsilon_max
        self.memory_counter = 0
        # total learning step
        self.learn_step_counter = 0

        # initialize zero memory [s, a, r, s_]
        self.memory = np.zeros((self.memory_size, (n_features+54)*2+3))

        # consist of [target_net, evaluate_net]
        self._build_net()
        t_params = tf.get_collection('target_net_params')
        e_params = tf.get_collection('eval_net_params')
        self.replace_target_op = [tf.assign(t, e) for t, e in zip(t_params, e_params)]

        self.sess = tf.Session()

        if output_graph:
            # $ tensorboard --logdir=logs
            # tf.train.SummaryWriter soon be deprecated, use following
            tf.summary.FileWriter("logs/", self.sess.graph)

        self.sess.run(tf.global_variables_initializer())
        self.cost_his = []

    def _build_net(self):
        # eval net
        self.q_target = tf.placeholder(tf.float32, [None, self.n_actions])
        self.s_left = tf.placeholder(tf.float32, [None, 18], name='s_left')
        self.s_mid = tf.placeholder(tf.float32, [None, 18], name='s_mid')
        self.s_right = tf.placeholder(tf.float32, [None, 18], name='s_right')
        self.s_feature = tf.placeholder(tf.float32, [None, self.n_features], name='s_state')

        # self.q_eval_high (dim = 3) self. q_eval_low (dim = 11)

        with tf.variable_scope('loss'):
            self.loss = tf.reduce_mean(tf.squared_difference(self.q_target, self.q_eval_low))
        with tf.variable_scope('train'):
            self._train_op = tf.train.RMSPropOptimizer(self.lr).minimize(self.loss)

        # target net
        self.s_left_ = tf.placeholder(tf.float32, [None, 18], name='s_left_')
        self.s_mid_ = tf.placeholder(tf.float32, [None, 18], name='s_mid_')
        self.s_right_ = tf.placeholder(tf.float32, [None, 18], name='s_right_')
        self.s_feature_ = tf.placeholder(tf.float32, [None, self.n_features], name='s_state_')

    def store_transition(self, s, a_high, a_low, r, s_):
        if not hasattr(self, 'memory_counter'):
            self.memory_counter = 0
        if a_high == 0:
            a = a_low
        elif a_high == 2:
            a = self.n_actions_l + self.n_actions_m + a_low
        else:
            a = self.n_actions_l + a_low
        transition = np.hstack((s, [a, r], s_))
        # replace the old memory with new memory
        index = self.memory_counter % self.memory_size
        self.memory[index, :] = transition
        self.memory_counter += 1

    def choose_action(self, observation_s_left, observation_s_mid, observation_s_right, observation_s_state):
        # to have batch dimension when feed into tf placeholder
        observation_s_left = observation_s_left[np.newaxis, :]
        observation_s_mid = observation_s_mid[np.newaxis, :]
        observation_s_right = observation_s_right[np.newaxis, :]
        observation_s_state = observation_s_state[np.newaxis, :]
        if np.random.uniform() < self.epsilon:
            # forward feed the observation and get q value for every actions
            actions_value_high, actions_value_low = self.sess.run([self.q_eval_high, self.q_eval_low],
                                                                  feed_dict={self.s_left: observation_s_left,
                                                                             self.s_mid: observation_s_mid,
                                                                             self.s_right: observation_s_right,
                                                                             self.s_feature: observation_s_state
                                                                             })
            action_high = np.argmax(actions_value_high)
            if action_high == 0:
                action_low = np.argmax(actions_value_low[:self.n_actions_l])
            elif action_high == 1:
                action_low = 0
            else:
                action_low = np.argmax(actions_value_low[self.n_actions_r+1:])
        else:
            action_high = np.random.randint(0, 3)
            if action_high == 0:
                action_low = np.random.randint(0, 5)
            elif action_high == 1:
                action_low = 0
            else:
                action_low = np.random.randint(0, 5)
        return [action_high, action_low]

    def learn(self):
        if self.learn_step_counter % self.replace_target_iter == 0:
            self.sess.run(self.replace_target_op)
            print('\ntarget_params_replaced\n')
            # sample batch memory from all memory
        if self.memory_counter > self.memory_size:
            sample_index = np.random.choice(self.memory_size, size=self.batch_size)
        else:
            sample_index = np.random.choice(self.memory_counter, size=self.batch_size)
        batch_memory = self.memory[sample_index, :]

        q_next_low, q_eval_low = self.sess.run(
            [self.q_next_low, self.q_eval_low],
            feed_dict={
                self.s_left_: batch_memory[:, -(self.n_features+self.n_right+self.n_mid+self.n_left):-(self.n_features+self.n_right+self.n_mid)],
                self.s_mid_: batch_memory[:, -(self.n_features+self.n_right+self.n_mid):-(self.n_features+self.n_right)],
                self.s_right_: batch_memory[:, -(self.n_features+self.n_right):-self.n_features],
                self.s_feature_: batch_memory[:, -self.n_features:],  # fixed params
                self.s_left: batch_memory[:, :self.n_left],  # newest params
                self.s_mid: batch_memory[:, self.n_left:self.n_left+self.n_mid],
                self.s_right: batch_memory[:, self.n_left+self.n_mid:self.n_left+self.n_mid+self.n_right],
                self.s_feature: batch_memory[:, self.n_left+self.n_mid+self.n_right:self.n_left+self.n_mid+self.n_right+self.n_features]
            })

        q_target = q_eval_low.copy()
        batch_index = np.arange(self.batch_size, dtype=np.int32)
        eval_act_index = batch_memory[:, self.n_state].astype(int)
        reward = batch_memory[:, self.n_state + 1]
        q_target[batch_index, eval_act_index] = reward + self.gamma * np.max(q_next_low, axis=1)

        _, self.cost = self.sess.run([self._train_op, self.loss],
                                     feed_dict={self.s_left_: batch_memory[:, -(self.n_features+self.n_right+self.n_mid+self.n_left):-(self.n_features+self.n_right+self.n_mid)],
                                                self.s_mid_: batch_memory[:, -(self.n_features+self.n_right+self.n_mid):-(self.n_features+self.n_right)],
                                                self.s_right_: batch_memory[:, -(self.n_features+self.n_right):-self.n_features],
                                                self.s_feature_: batch_memory[:, -self.n_features:],  # fixed params
                                                self.q_target: q_target})
        self.cost_his.append(self.cost)

        # increasing epsilon
        self.epsilon = self.epsilon + self.epsilon_increment if self.epsilon < self.epsilon_max else self.epsilon_max
        self.learn_step_counter += 1

    def plot_cost(self):
        import matplotlib.pyplot as plt
        plt.plot(np.arange(len(self.cost_his)), self.cost_his)
        plt.ylabel('Cost')
        plt.xlabel('training steps')
        plt.show()
