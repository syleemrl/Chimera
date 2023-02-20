import tensorflow as tf
import numpy as np
from tensorflow.keras import Sequential, Input
from tensorflow.keras.layers import Dense

actor_layer_size = 1024
regression_layer_size = 1024
critic_layer_size = 512
activ = "relu"

class Actor(object):
	def __init__(self, num_states, num_actions, num_phi, name):
		self.mean, self.logstd = self.create_network(num_states, num_actions, num_phi, name)

	def create_network(self, num_states, num_actions, num_phi, name):
		postfix = name+'_mean'
		mean = Sequential()
		mean.add(Dense(actor_layer_size, activation=activ, input_shape=(num_states, ), dtype=tf.float32, name=postfix+'/layer_with_weights-0'))
		mean.add(Dense(actor_layer_size, activation=activ, dtype=tf.float32, name=postfix+'/layer_with_weights-1'))
		mean.add(Dense(actor_layer_size, activation=activ, dtype=tf.float32, name=postfix+'/layer_with_weights-2'))
		mean.add(Dense(actor_layer_size, activation=activ, dtype=tf.float32, name=postfix+'/layer_with_weights-3'))
		mean.add(Dense(num_actions, dtype=tf.float32, name=postfix+'/layer_with_weights-4'))
	
		self.logstd_space = tf.Variable(initial_value = np.zeros(num_actions-num_phi), dtype=tf.float32, name=name+'_logstd', trainable=True)
		self.logstd_time = tf.Variable(initial_value = np.ones(num_phi) * -2, dtype=tf.float32, trainable=False)

		return mean, self.logstd_space 
	
	@tf.function
	def std(self):
		return tf.exp(tf.concat((self.logstd_space, self.logstd_time), axis=0))

	@tf.function
	def neglogp(self, action, mean):
		return 0.5 * tf.reduce_sum(input_tensor=tf.square((action - mean) / self.std()), axis=-1) \
			   + 0.5 * np.log(2.0 * np.pi) * tf.cast(tf.shape(input=action)[-1], dtype=tf.float32) \
			   + tf.reduce_sum(tf.concat((self.logstd_space, self.logstd_time), axis=0), axis=-1)

	@tf.function
	def get_action(self, states):
		mean = self.mean(states)
		action = mean + self.std() * tf.random.normal(tf.shape(mean))
		neglogprob = self.neglogp(action, mean)
		return action, neglogprob

	@tf.function
	def get_mean_action(self, states):
		return self.mean(states)

	def get_variable(self, trainable_only=False):
		if trainable_only:
			return self.mean.trainable_variables + [self.logstd]
		else:
			return self.mean.variables + [self.logstd] 


class FC(object):
	def __init__(self, num_state, num_action, name, fc_layer_size=critic_layer_size, regularizer=None):
		self.value = self.create_network(num_state, num_action, fc_layer_size, name, regularizer)

	def create_network(self, num_state, num_action, fc_layer_size, name, regularizer):	
		value = Sequential()
		value.add(Dense(fc_layer_size, activation=activ, input_shape=(num_state, ), kernel_regularizer=regularizer, dtype=tf.float32, name=name+'/layer_with_weights-0'))
		value.add(Dense(fc_layer_size, activation=activ, dtype=tf.float32, kernel_regularizer=regularizer, name=name+'/layer_with_weights-1'))
		value.add(Dense(num_action, dtype=tf.float32, kernel_regularizer=regularizer, name=name+'/layer_with_weights-2'))
	
		return value

	@tf.function
	def get_value(self, states):
		return self.value(states)

	def get_variable(self, trainable_only=False):
		if trainable_only:
			return self.value.trainable_variables
		else:
			return self.value.variables
