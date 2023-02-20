import sys
if not hasattr(sys, 'argv'):
    sys.argv  = ['']
from network import FC
import random
import numpy as np
import tensorflow as tf
import time
from IPython import embed
from copy import deepcopy

regression_layer_size = 512
gpu_devices = tf.config.experimental.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(gpu_devices[0], True)
class Regression(object):
	def __init__(self, learning_rate=0.0001):
		random.seed(int(time.time()))
		np.random.seed(int(time.time()))
	
		self.learning_rate = learning_rate	

	def init_run(self, directory, num_input, num_output):
		self.directory = directory + "/"
		self.num_input = num_input
		self.num_output = num_output

		self.build_optimize()

		self.ckpt = tf.train.Checkpoint(
			regression=self.regression.value
		)

		self.load(self.directory+"reg_network-0")

	def init_train(self, directory, num_input, num_output, load_param, batch_size=128, num_iter=4000):
		self.directory = directory

		self.num_iter = num_iter
		self.batch_size = batch_size

		self.num_input = num_input
		self.num_output = num_output
		#build network and optimizer
		self.build_optimize()

		self.ckpt = tf.train.Checkpoint(
			regression=self.regression.value
		)
		self.never_trained = True
		if load_param:
			self.load(self.directory+"reg_network-0")
			self.never_trained = False

		print("init regression network done")

	def build_optimize(self):

		self.regression = FC(self.num_input, self.num_output, 'regression', regression_layer_size, tf.keras.regularizers.l2(0.01))
		self.regression_trainer = tf.keras.optimizers.Adam(learning_rate=self.learning_rate)

	@tf.function
	def train_regression_network(self, input, output):
		with tf.GradientTape() as tape:
			loss = tf.reduce_mean(tf.square(self.regression.get_value(input) - output))
		params = self.regression.get_variable(True)
		grads = tape.gradient(loss, params)
		
		self.regression_trainer.apply_gradients(zip(grads, params))
		return loss

	def set_training_data(self, tuples):
		self.regression_x = tuples[0]
		self.regression_y = tuples[1]

	def save(self):
		self.ckpt.write(self.directory+'reg_network-0')

	def load(self, path):
		print("Loading parameters from {}".format(path))
		saved_variables = tf.train.list_variables(path)
		saved_values = []
		for v in saved_variables:
			saved_values.append(tf.train.load_variable(path,v[0]))
		saved_dict = {n[0] : v for n, v in zip(saved_variables, saved_values)}
		trainable_variables = self.regression.get_variable(True)
	
		for v in trainable_variables:
			key = v.name[:-2]+'/.ATTRIBUTES/VARIABLE_VALUE'
			if key in saved_dict:
				saved_v = saved_dict[key]
				if v.shape == saved_v.shape:
					print("Restore {}".format(key))
					v.assign(saved_v)

	def print_network_summary(self):
		print_list = []
		print_list.append('===============================================================')
		for v in self.lossvals:
			print_list.append('{}: {:.3f}'.format(v[0], v[1]))
		print_list.append('===============================================================')
		for s in print_list:
			print(s)

	def train(self):
		lossval_prev = 0
		no_improvement = 0
		num_iter = self.num_iter
		if self.never_trained:
			num_iter = self.num_iter * 2
		for it in range(num_iter):
			lossval_reg = 0

			if int(len(self.regression_x) // self.batch_size) == 0:
				lossval_reg += self.train_regression_network(self.regression_x, self.regression_y)
				num_batch = 1
			else:
				ind = np.arange(len(self.regression_x))
				np.random.shuffle(ind)
				for s in range(int(len(ind)//self.batch_size)):
					selectedIndex = ind[s*self.batch_size:(s+1)*self.batch_size]
					lossval_reg += self.train_regression_network(self.regression_x[selectedIndex], self.regression_y[selectedIndex])
				num_batch = int(len(ind)//self.batch_size)

			if it % 10 == 0:
				if it != 0 and lossval_prev / num_batch - lossval_reg / num_batch < 0.2 * 1e-5:
					no_improvement += 1
				else:
					no_improvement = 0
				lossval_prev = lossval_reg
				print('{}: {:.6f}'.format(it, lossval_reg / num_batch), no_improvement)

			if not self.never_trained and no_improvement >= 10:
				print('regression network training done: ', it, lossval_reg )
				break
		self.save()
		self.never_trained = False

	def run(self, input):
		output = self.regression.get_value(input)

		return output