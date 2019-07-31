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
            reward_decay=0.7,
            e_greedy=0.99,
            replace_target_iter=100,
            memory_size=500,
            batch_size=32,
            e_greedy_increment=None,
            output_graph=False,
    ):
        self.n_actions_high = 3
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
        self.epsilon = 0.5 if e_greedy_increment is not None else self.epsilon_max
        self.memory_counter = 0
        # total learning step
        self.learn_step_counter = 0

        # initialize zero memory [s, a, r, s_]
        self.memory = np.zeros((self.memory_size, (n_features+54)*2+2))

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
        self.cost_his_l = []
        self.cost_his_h = []

    def conv1d(self, x, weight, strides):
        return tf.nn.conv1d(x, weight, strides, padding='SAME', data_format="NWC")

    def _build_net(self):
        # eval net
        self.q_target_low = tf.placeholder(tf.float32, [None, self.n_actions])
        self.q_target_high = tf.placeholder(tf.float32, [None, self.n_actions_high])
        self.s_left = tf.placeholder(tf.float32, [None, 18, 1], name='s_left')
        self.s_mid = tf.placeholder(tf.float32, [None, 18, 1], name='s_mid')
        self.s_right = tf.placeholder(tf.float32, [None, 18, 1], name='s_right')
        self.s_feature = tf.placeholder(tf.float32, [None, self.n_features], name='s_feature')

        # self.q_eval_high (dim = 3) self. q_eval_low (dim = 11)
        with tf.variable_scope('eval_net'):
            c_names, w_initializer, b_initializer = \
                ['eval_net_params', tf.GraphKeys.GLOBAL_VARIABLES],\
                tf.random_normal_initializer(0., 0.3), tf.constant_initializer(0.1)  # config of layers
            n_conv_l1 = 8
            n_conv_l2 = 8

            # build left cnn
            with tf.variable_scope('l_conv_l1'):
                w_l_conv_l1 = tf.get_variable('w_l_conv_l1', [3, 1, n_conv_l1], initializer=w_initializer, collections=c_names)
                b_l_conv_l1 = tf.get_variable('b_l_conv_l1', [1, n_conv_l1], initializer=b_initializer, collections=c_names)
                l_conv_l1 = tf.nn.relu(self.conv1d(self.s_left, w_l_conv_l1, 3)+b_l_conv_l1)
                # l_conv_l1 = tf.nn.relu(self.conv1d(self.s_left, w_l_conv_l1, 3))
            with tf.variable_scope('l_conv_l2'):
                w_l_conv_l2 = tf.get_variable('w_l_conv_l2', [1, n_conv_l1, n_conv_l2], initializer=w_initializer, collections=c_names)
                b_l_conv_l2 = tf.get_variable('b_l_conv_l2', [1, n_conv_l2], initializer=b_initializer, collections=c_names)
                l_conv_l2 = tf.nn.relu(self.conv1d(l_conv_l1, w_l_conv_l2, 1)+b_l_conv_l2)
            l_conv_output = tf.reshape(l_conv_l2, [-1, 6*8])

            # build mid cnn
            with tf.variable_scope('m_conv_l1'):
                w_m_conv_l1 = tf.get_variable('w_m_conv_l1', [3, 1, n_conv_l1], initializer=w_initializer, collections=c_names)
                b_m_conv_l1 = tf.get_variable('b_m_conv_l1', [1, n_conv_l1], initializer=b_initializer, collections=c_names)
                m_conv_l1 = tf.nn.relu(self.conv1d(self.s_mid, w_m_conv_l1, 3)+b_m_conv_l1)
            with tf.variable_scope('m_conv_l2'):
                w_m_conv_l2 = tf.get_variable('w_m_conv_l2', [1, n_conv_l1, n_conv_l2], initializer=w_initializer, collections=c_names)
                b_m_conv_l2 = tf.get_variable('b_m_conv_l2', [1, n_conv_l2], initializer=b_initializer, collections=c_names)
                m_conv_l2 = tf.nn.relu(self.conv1d(m_conv_l1, w_m_conv_l2, 1)+b_m_conv_l2)
            m_conv_output = tf.reshape(m_conv_l2, [-1, 6 * 8])
            # build right cnn
            with tf.variable_scope('r_conv_l1'):
                w_r_conv_l1 = tf.get_variable('w_r_conv_l1', [3, 1, n_conv_l1], initializer=w_initializer, collections=c_names)
                b_r_conv_l1 = tf.get_variable('b_r_conv_l1', [1, n_conv_l1], initializer=b_initializer, collections=c_names)
                r_conv_l1 = tf.nn.relu(self.conv1d(self.s_right, w_r_conv_l1, 3)+b_r_conv_l1)
            with tf.variable_scope('r_conv_l2'):
                w_r_conv_l2 = tf.get_variable('w_r_conv_l2', [1, n_conv_l1, n_conv_l2], initializer=w_initializer, collections=c_names)
                b_r_conv_l2 = tf.get_variable('b_r_conv_l2', [1, n_conv_l2], initializer=b_initializer, collections=c_names)
                r_conv_l2 = tf.nn.relu(self.conv1d(r_conv_l1, w_r_conv_l2, 1)+b_r_conv_l2)
            r_conv_output = tf.reshape(r_conv_l2, [-1, 6 * 8])
            # merge
            fully_connected_input = tf.concat([self.s_feature, l_conv_output, m_conv_output, r_conv_output], 1)

            # build fully connected layer
            n_high_l1 = 50
            with tf.variable_scope('high_l1'):
                w_high_l1 = tf.get_variable('w_high_l1', [self.n_features+6*8*3, n_high_l1], initializer=w_initializer, collections=c_names)
                b_high_l1 = tf.get_variable('b_high_l1', [1, n_high_l1], initializer=b_initializer, collections=c_names)
                high_l1 = tf.nn.relu(tf.matmul(fully_connected_input, w_high_l1) + b_high_l1)
            with tf.variable_scope('fully_l2'):
                w_high_l2 = tf.get_variable('w_high_l2', [n_high_l1, self.n_actions_high], initializer=w_initializer, collections=c_names)
                b_high_l2 = tf.get_variable('b_high_l2', [1, self.n_actions_high], initializer=b_initializer, collections=c_names)
                self.q_eval_high = tf.matmul(high_l1, w_high_l2) + b_high_l2
            # merge low left data

            low_left = tf.concat([self.s_feature, l_conv_output, m_conv_output, tf.transpose([tf.cast(tf.argmax(self.q_eval_high, 1),dtype=tf.float32)])], 1)

            # merge low mid data
            low_mid = tf.concat([fully_connected_input, tf.transpose([tf.cast(tf.argmax(self.q_eval_high, 1), dtype=tf.float32)])], 1)

            # merge low right data
            low_right = tf.concat([self.s_feature, r_conv_output, m_conv_output, tf.transpose([tf.cast(tf.argmax(self.q_eval_high, 1),dtype=tf.float32)])], 1)

            # build low left fully connected layer
            n_low_l1 = 50
            with tf.variable_scope('low_l_l1'):
                w_low_l_l1 = tf.get_variable('w_low_l_l1', [self.n_features + 6 * 8 * 2 + 1, n_low_l1], initializer=w_initializer, collections=c_names)
                b_low_l_l1 = tf.get_variable('b_low_l_l1', [1, n_low_l1], initializer=b_initializer, collections=c_names)
                low_l_l1 = tf.nn.relu(tf.matmul(low_left, w_low_l_l1) + b_low_l_l1)
            with tf.variable_scope('low_l_l2'):
                w_low_l_l2 = tf.get_variable('w_low_l_l2', [n_low_l1, self.n_actions_l], initializer=w_initializer, collections=c_names)
                b_low_l_l2 = tf.get_variable('b_low_l_l2', [1, self.n_actions_l], initializer=b_initializer, collections=c_names)
                self.q_eval_low_l = tf.matmul(low_l_l1, w_low_l_l2) + b_low_l_l2

            # build low mid fully connected layer
            with tf.variable_scope('low_m_l1'):
                w_low_m_l1 = tf.get_variable('w_low_m_l1', [self.n_features + 6 * 8 * 3 + 1, n_low_l1], initializer=w_initializer, collections=c_names)
                b_low_m_l1 = tf.get_variable('b_low_m_l1', [1, n_low_l1], initializer=b_initializer, collections=c_names)
                low_m_l1 = tf.nn.relu(tf.matmul(low_mid, w_low_m_l1) + b_low_m_l1)
            with tf.variable_scope('low_l_l2'):
                w_low_m_l2 = tf.get_variable('w_low_m_l2', [n_low_l1, self.n_actions_m], initializer=w_initializer, collections=c_names)
                b_low_m_l2 = tf.get_variable('b_low_m_l2', [1, self.n_actions_m], initializer=b_initializer, collections=c_names)
                self.q_eval_low_m = tf.matmul(low_m_l1, w_low_m_l2) + b_low_m_l2

            # build low right fully connected layer
            with tf.variable_scope('low_r_l1'):
                w_low_r_l1 = tf.get_variable('w_low_r_l1', [self.n_features + 6 * 8 * 2 + 1, n_low_l1], initializer=w_initializer, collections=c_names)
                b_low_r_l1 = tf.get_variable('b_low_r_l1', [1, n_low_l1], initializer=b_initializer, collections=c_names)
                low_r_l1 = tf.nn.relu(tf.matmul(low_right, w_low_r_l1) + b_low_r_l1)
            with tf.variable_scope('low_r_l2'):
                w_low_r_l2 = tf.get_variable('w_low_r_l2', [n_low_l1, self.n_actions_r], initializer=w_initializer, collections=c_names)
                b_low_r_l2 = tf.get_variable('b_low_r_l2', [1, self.n_actions_r], initializer=b_initializer, collections=c_names)
                self.q_eval_low_r = tf.matmul(low_r_l1, w_low_r_l2) + b_low_r_l2
            self.q_eval_low = tf.concat([self.q_eval_low_l, self.q_eval_low_m, self.q_eval_low_r], 1)
        # loss and train
        with tf.variable_scope('loss_h'):
            self.loss_h = tf.reduce_mean(tf.squared_difference(self.q_target_high, self.q_eval_high))
        with tf.variable_scope('tran_h'):
            self._train_op_h = tf.train.RMSPropOptimizer(self.lr).minimize(self.loss_h)
        with tf.variable_scope('loss_l'):
            self.loss_l = tf.reduce_mean(tf.squared_difference(self.q_target_low, self.q_eval_low))
        with tf.variable_scope('train_l'):
            self._train_op_l = tf.train.RMSPropOptimizer(self.lr).minimize(self.loss_l)

        # target net
        self.s_left_ = tf.placeholder(tf.float32, [None, 18, 1], name='s_left_')
        self.s_mid_ = tf.placeholder(tf.float32, [None, 18, 1], name='s_mid_')
        self.s_right_ = tf.placeholder(tf.float32, [None, 18, 1], name='s_right_')
        self.s_feature_ = tf.placeholder(tf.float32, [None, self.n_features], name='s_feature_')

        with tf.variable_scope('target_net'):
            # c_names(collections_names) are the collections to store variables
            c_names = ['target_net_params', tf.GraphKeys.GLOBAL_VARIABLES]
            n_conv_l1 = 8
            n_conv_l2 = 8

            # build left cnn
            with tf.variable_scope('l_conv_l1'):
                w_l_conv_l1 = tf.get_variable('w_l_conv_l1', [3, 1, n_conv_l1], initializer=w_initializer,
                                               collections=c_names)
                b_l_conv_l1 = tf.get_variable('b_l_conv_l1', [1, n_conv_l1], initializer=b_initializer,
                                               collections=c_names)
                l_conv_l1 = tf.nn.relu(self.conv1d(self.s_left_, w_l_conv_l1, 3) + b_l_conv_l1)
            with tf.variable_scope('l_conv_l2'):
                w_l_conv_l2 = tf.get_variable('w_l_conv_l2', [1, n_conv_l1, n_conv_l2], initializer=w_initializer,
                                               collections=c_names)
                b_l_conv_l2 = tf.get_variable('b_l_conv_l2', [1, n_conv_l2], initializer=b_initializer,
                                               collections=c_names)
                l_conv_l2 = tf.nn.relu(self.conv1d(l_conv_l1, w_l_conv_l2, 1) + b_l_conv_l2)
            l_conv_output_ = tf.reshape(l_conv_l2, [-1, 6 * 8])

            # build mid cnn
            with tf.variable_scope('m_conv_l1'):
                w_m_conv_l1 = tf.get_variable('w_m_conv_l1', [3, 1, n_conv_l1], initializer=w_initializer,
                                               collections=c_names)
                b_m_conv_l1 = tf.get_variable('b_m_conv_l1', [1, n_conv_l1], initializer=b_initializer,
                                               collections=c_names)
                m_conv_l1 = tf.nn.relu(self.conv1d(self.s_mid_, w_m_conv_l1, 3) + b_m_conv_l1)
            with tf.variable_scope('m_conv_l2'):
                w_m_conv_l2 = tf.get_variable('w_m_conv_l2', [1, n_conv_l1, n_conv_l2], initializer=w_initializer,
                                               collections=c_names)
                b_m_conv_l2 = tf.get_variable('b_m_conv_l2', [1, n_conv_l2], initializer=b_initializer,
                                               collections=c_names)
                m_conv_l2 = tf.nn.relu(self.conv1d(m_conv_l1, w_m_conv_l2, 1) + b_m_conv_l2)
            m_conv_output_ = tf.reshape(m_conv_l2, [-1, 6 * 8])
            # build right cnn
            with tf.variable_scope('r_conv_l1'):
                w_r_conv_l1 = tf.get_variable('w_r_conv_l1', [3, 1, n_conv_l1], initializer=w_initializer,
                                               collections=c_names)
                b_r_conv_l1 = tf.get_variable('b_r_conv_l1', [1, n_conv_l1], initializer=b_initializer,
                                               collections=c_names)
                r_conv_l1 = tf.nn.relu(self.conv1d(self.s_right_, w_r_conv_l1, 3) + b_r_conv_l1)
            with tf.variable_scope('r_conv_l2'):
                w_r_conv_l2 = tf.get_variable('w_r_conv_l2', [1, n_conv_l1, n_conv_l2], initializer=w_initializer,
                                               collections=c_names)
                b_r_conv_l2 = tf.get_variable('b_r_conv_l2', [1, n_conv_l2], initializer=b_initializer,
                                               collections=c_names)
                r_conv_l2 = tf.nn.relu(self.conv1d(r_conv_l1, w_r_conv_l2, 1) + b_r_conv_l2)
            r_conv_output_ = tf.reshape(r_conv_l2, [-1, 6 * 8])
            # merge
            fully_connected_input_ = tf.concat([self.s_feature_, l_conv_output_, m_conv_output_, r_conv_output_], 1)

            # build fully connected layer
            n_high_l1 = 50
            with tf.variable_scope('high_l1'):
                w_high_l1 = tf.get_variable('w_high_l1', [self.n_features + 6 * 8 * 3, n_high_l1],
                                            initializer=w_initializer, collections=c_names)
                b_high_l1 = tf.get_variable('b_high_l1', [1, n_high_l1], initializer=b_initializer, collections=c_names)
                high_l1 = tf.nn.relu(tf.matmul(fully_connected_input, w_high_l1) + b_high_l1)
            with tf.variable_scope('fully_l2'):
                w_high_l2 = tf.get_variable('w_high_l2', [n_high_l1, self.n_actions_high], initializer=w_initializer,
                                            collections=c_names)
                b_high_l2 = tf.get_variable('b_high_l2', [1, self.n_actions_high], initializer=b_initializer,
                                            collections=c_names)
                self.q_next_high = tf.matmul(high_l1, w_high_l2) + b_high_l2
            # merge low left data
            low_left_ = tf.concat([self.s_feature_, l_conv_output_, m_conv_output_, tf.transpose([tf.cast(tf.argmax(self.q_next_high, 1),dtype=tf.float32)])], 1)

            # merge low mid data
            low_mid_ = tf.concat([fully_connected_input_, tf.transpose([tf.cast(tf.argmax(self.q_next_high, 1),dtype=tf.float32)])], 1)

            # merge low right data
            low_right_ = tf.concat([self.s_feature_, r_conv_output_, m_conv_output_, tf.transpose([tf.cast(tf.argmax(self.q_next_high, 1),dtype=tf.float32)])], 1)

            # build low left fully connected layer
            n_low_l1 = 50
            with tf.variable_scope('low_l_l1'):
                w_low_l_l1 = tf.get_variable('w_low_l_l1', [self.n_features + 6 * 8 * 2 + 1, n_low_l1],
                                             initializer=w_initializer, collections=c_names)
                b_low_l_l1 = tf.get_variable('b_low_l_l1', [1, n_low_l1], initializer=b_initializer,
                                             collections=c_names)
                low_l_l1 = tf.nn.relu(tf.matmul(low_left_, w_low_l_l1) + b_low_l_l1)
            with tf.variable_scope('low_l_l2'):
                w_low_l_l2 = tf.get_variable('w_low_l_l2', [n_low_l1, self.n_actions_l], initializer=w_initializer,
                                             collections=c_names)
                b_low_l_l2 = tf.get_variable('b_low_l_l2', [1, self.n_actions_l], initializer=b_initializer,
                                             collections=c_names)
                self.q_next_low_l = tf.matmul(low_l_l1, w_low_l_l2) + b_low_l_l2

            # build low mid fully connected layer
            with tf.variable_scope('low_m_l1'):
                w_low_m_l1 = tf.get_variable('w_low_m_l1', [self.n_features + 6 * 8 * 3 + 1, n_low_l1],
                                             initializer=w_initializer, collections=c_names)
                b_low_m_l1 = tf.get_variable('b_low_m_l1', [1, n_low_l1], initializer=b_initializer,
                                             collections=c_names)
                low_m_l1 = tf.nn.relu(tf.matmul(low_mid_, w_low_m_l1) + b_low_m_l1)
            with tf.variable_scope('low_l_l2'):
                w_low_m_l2 = tf.get_variable('w_low_m_l2', [n_low_l1, self.n_actions_m], initializer=w_initializer,
                                             collections=c_names)
                b_low_m_l2 = tf.get_variable('b_low_m_l2', [1, self.n_actions_m], initializer=b_initializer,
                                             collections=c_names)
                self.q_next_low_m = tf.matmul(low_m_l1, w_low_m_l2) + b_low_m_l2

            # build low right fully connected layer
            with tf.variable_scope('low_r_l1'):
                w_low_r_l1 = tf.get_variable('w_low_r_l1', [self.n_features + 6 * 8 * 2 + 1, n_low_l1],
                                             initializer=w_initializer, collections=c_names)
                b_low_r_l1 = tf.get_variable('b_low_r_l1', [1, n_low_l1], initializer=b_initializer,
                                             collections=c_names)
                low_r_l1 = tf.nn.relu(tf.matmul(low_right_, w_low_r_l1) + b_low_r_l1)
            with tf.variable_scope('low_r_l2'):
                w_low_r_l2 = tf.get_variable('w_low_r_l2', [n_low_l1, self.n_actions_r], initializer=w_initializer,
                                             collections=c_names)
                b_low_r_l2 = tf.get_variable('b_low_r_l2', [1, self.n_actions_r], initializer=b_initializer,
                                             collections=c_names)
                self.q_next_low_r = tf.matmul(low_r_l1, w_low_r_l2) + b_low_r_l2
            self.q_next_low = tf.concat([self.q_next_low_l, self.q_next_low_m, self.q_next_low_r], 1)

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

    def choose_action(self, observation):
        # to have batch dimension when feed into tf placeholder
        observation_s_left = observation[:self.n_left]
        observation_s_mid = observation[self.n_left:self.n_left+self.n_mid]
        observation_s_right = observation[self.n_left+self.n_mid:self.n_left+self.n_mid+self.n_right]
        observation_s_state = observation[self.n_left+self.n_mid+self.n_right:]
        observation_s_state = observation_s_state[np.newaxis, :]
        if np.random.uniform() < self.epsilon:
            # forward feed the observation and get q value for every actions
            print("greedy choose")
            actions_value_high, actions_value_low = self.sess.run([self.q_eval_high, self.q_eval_low],
                                                                  feed_dict={self.s_left: observation_s_left.reshape(-1,18,1),
                                                                             self.s_mid: observation_s_mid.reshape(-1,18,1),
                                                                             self.s_right: observation_s_right.reshape(-1,18,1),
                                                                             self.s_feature: observation_s_state
                                                                             })
            action_high = np.argmax(actions_value_high)
            print(actions_value_high)
            print(actions_value_low)
            if action_high == 0:
                actions_low = actions_value_low[:, :self.n_actions_l]
                action_low = np.argmax(actions_low, axis=1)[0]
            elif action_high == 1:
                action_low = 0
            else:
                actions_low = actions_value_low[:, -self.n_actions_r:]
                action_low = np.argmax(actions_low, axis=1)[0]
        else:
            print("random choose")
            action_high = np.random.randint(0, 3)
            if action_high == 0:
                action_low = np.random.randint(0, 5)
            elif action_high == 1:
                action_low = 0
            else:
                action_low = np.random.randint(0, 5)
        return [action_high, action_low]

    def learn(self):
        # replace data and increase epsilon
        if self.learn_step_counter % self.replace_target_iter == 0:
            self.sess.run(self.replace_target_op)
            print('\ntarget_params_replaced\n')
            self.epsilon = self.epsilon + self.epsilon_increment if self.epsilon < self.epsilon_max else self.epsilon_max
        # sample batch memory from all memory
        if self.memory_counter > self.memory_size:
            sample_index = np.random.choice(self.memory_size, size=self.batch_size)
        else:
            sample_index = np.random.choice(self.memory_counter, size=self.batch_size)
        batch_memory = self.memory[sample_index, :]
        # train low net 10 times every 20 steps
        if self.learn_step_counter % 20 < 10:
            q_next_high, q_next_low, q_eval_low = self.sess.run(
                [self.q_next_high, self.q_next_low, self.q_eval_low],
                feed_dict={
                    self.s_left_: batch_memory[:, -(self.n_features+self.n_right+self.n_mid+self.n_left):-(self.n_features+self.n_right+self.n_mid)].reshape(-1, 18, 1),
                    self.s_mid_: batch_memory[:, -(self.n_features+self.n_right+self.n_mid):-(self.n_features+self.n_right)].reshape(-1, 18, 1),
                    self.s_right_: batch_memory[:, -(self.n_features+self.n_right):-self.n_features].reshape(-1, 18, 1),
                    self.s_feature_: batch_memory[:, -self.n_features:],  # fixed params
                    self.s_left: batch_memory[:, :self.n_left].reshape(-1, 18, 1),  # newest params
                    self.s_mid: batch_memory[:, self.n_left:self.n_left+self.n_mid].reshape(-1, 18, 1),
                    self.s_right: batch_memory[:, self.n_left+self.n_mid:self.n_left+self.n_mid+self.n_right].reshape(-1, 18, 1),
                    self.s_feature: batch_memory[:, self.n_left+self.n_mid+self.n_right:self.n_left+self.n_mid+self.n_right+self.n_features]
                })
            q_target_low = q_eval_low.copy()
            eval_act_index = batch_memory[:, self.n_state].astype(int)
            reward = batch_memory[:, self.n_state + 1]
            for batch_index in range(self.batch_size):
                if np.argmax(q_next_high[batch_index, :]) == 0:
                    q_next = np.max(q_next_low[batch_index, :self.n_actions_l])
                elif np.argmax(q_next_high[batch_index, :]) == 1:
                    q_next = q_next_low[batch_index, self.n_actions_l]
                else:
                    q_next = np.max(q_next_low[batch_index, -self.n_actions_r:])
                q_target_low[batch_index, eval_act_index[batch_index]] = reward[batch_index] + self.gamma * q_next
            _, self.cost_l = self.sess.run([self._train_op_l, self.loss_l],
                                           feed_dict={self.s_left: batch_memory[:, :self.n_left].reshape(-1, 18, 1),  # newest params
                                           self.s_mid: batch_memory[:, self.n_left:self.n_left+self.n_mid].reshape(-1, 18, 1),
                                           self.s_right: batch_memory[:, self.n_left+self.n_mid:self.n_left+self.n_mid+self.n_right].reshape(-1, 18, 1),
                                           self.s_feature: batch_memory[:, self.n_left+self.n_mid+self.n_right:self.n_left+self.n_mid+self.n_right+self.n_features],
                                           self.q_target_low: q_target_low})
            self.cost_his_l.append(self.cost_l)
        # train high net 10 step every 20 steps
        if self.learn_step_counter % 20 >= 10:
            q_next_high, q_eval_high = self.sess.run(
                [self.q_next_high, self.q_eval_high],
                feed_dict={
                    self.s_left_: batch_memory[:, -(self.n_features + self.n_right + self.n_mid + self.n_left):-(self.n_features + self.n_right + self.n_mid)].reshape(-1, 18, 1),
                    self.s_mid_: batch_memory[:, -(self.n_features + self.n_right + self.n_mid):-(self.n_features + self.n_right)].reshape(-1, 18, 1),
                    self.s_right_: batch_memory[:, -(self.n_features + self.n_right):-self.n_features].reshape(-1, 18, 1),
                    self.s_feature_: batch_memory[:, -self.n_features:],  # fixed params
                    self.s_left: batch_memory[:, :self.n_left].reshape(-1, 18, 1),  # newest params
                    self.s_mid: batch_memory[:, self.n_left:self.n_left + self.n_mid].reshape(-1, 18, 1),
                    self.s_right: batch_memory[:, self.n_left + self.n_mid:self.n_left + self.n_mid + self.n_right].reshape(-1, 18, 1),
                    self.s_feature: batch_memory[:, self.n_left + self.n_mid + self.n_right:self.n_left + self.n_mid + self.n_right + self.n_features]
                })
            q_target_high = q_eval_high.copy()
            eval_act_index = batch_memory[:, self.n_state].astype(int)
            reward = batch_memory[:, self.n_state + 1]
            for batch_index in range(self.batch_size):
                if eval_act_index[batch_index] < 5:
                    q_target_high[batch_index, 0] = reward[batch_index] + self.gamma * np.max(q_next_high[batch_index, :])
                elif eval_act_index[batch_index] == 5:
                    q_target_high[batch_index, 1] = reward[batch_index] + self.gamma * np.max(q_next_high[batch_index, :])
                else:
                    q_target_high[batch_index, 2] = reward[batch_index] + self.gamma * np.max(q_next_high[batch_index, :])
            _, self.cost_h = self.sess.run([self._train_op_h, self.loss_h],
                                           feed_dict={self.s_left: batch_memory[:, :self.n_left].reshape(-1, 18, 1),  # newest params
                                           self.s_mid: batch_memory[:, self.n_left:self.n_left+self.n_mid].reshape(-1, 18, 1),
                                           self.s_right: batch_memory[:, self.n_left+self.n_mid:self.n_left+self.n_mid+self.n_right].reshape(-1, 18, 1),
                                           self.s_feature: batch_memory[:, self.n_left+self.n_mid+self.n_right:self.n_left+self.n_mid+self.n_right+self.n_features],
                                           self.q_target_high: q_target_high})
            self.cost_his_h.append(self.cost_h)
        # increasing learn step counter
        self.learn_step_counter += 1

    def plot_cost(self):
        import matplotlib.pyplot as plt
        plt.plot(np.arange(len(self.cost_his_l)), self.cost_his_l)
        plt.plot(np.arange(len(self.cost_his_h)), self.cost_his_h)
        plt.ylabel('Cost')
        plt.xlabel('training steps')
        plt.show()
