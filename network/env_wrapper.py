import simEnv
import time
import datetime
import math
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import copy
from utils import RunningMeanStd
from IPython import embed
import os
from copy import deepcopy

class EnvWrapper(object):
	def __init__(self, num_slave, directory, 
				 sim_config,
				 pretrain, plot=False, noreg=False,
				 vsi_thres=[0.5, 0.02], fix_time_iterations=50):		

		if pretrain != "":
			self.pretrain = True 
			self.fix_weight = False
		else:
			self.pretrain = False
			self.fix_weight = True

		self.vsi_thres = vsi_thres
		self.fix_time_iterations = fix_time_iterations

		self.noreg = noreg
		self.sim_env = simEnv.Env(num_slave, directory, 
								  sim_config,
								  self.pretrain, noreg)

		self.num_state = self.sim_env.getNumState()
		self.num_action = self.sim_env.getNumAction()
		self.num_slave = num_slave
		
		self.motionlength = self.sim_env.getMotionLength()
		self.maxlength = self.motionlength[0]
		for l in self.motionlength:
			if self.maxlength < l:
				self.maxlength = l

		self.num_phi = len(self.motionlength)
		self.reward_label = self.sim_env.getRewardLabels()

		self.RMS = RunningMeanStd(shape=(self.num_state))	
		self.plot = plot
		self.directory = directory

		self.start_time = time.time()		

		self.num_total_iterations = 0
		self.num_total_episodes = 0
		self.num_total_transitions = 0
		self.total_rewards = []
		self.total_rewards_by_parts = np.array([[]]*len(self.reward_label))

		self.num_nan_per_iteration = 0
		self.num_episodes_per_iteration = 0
		self.num_transitions_per_iteration = 0
		self.nt_elapsed_iteration = np.array([0.0]*self.num_phi)
		self.rewards_per_iteration = 0
		
		self.rewards_by_part_per_iteration = []
		self.transition_per_episodes_history = []

		self.stats_per_iteration = []
		self.rewards_slaves = [np.array([0.0]*len(self.reward_label))]*self.num_slave

		self.terminated = [False]*self.num_slave
		self.states = [0]*self.num_slave

		self.pool = self.sim_env.getInitialTiming()

		if self.plot:
			plt.ion()

		if not self.pretrain:
			config_path, filename = self.sim_env.getConfigFilePath()
			print(config_path)
			print(filename)
			os.system('cp '+ config_path[0] + ' '+directory+'/config.txt')
			os.system('cp '+ config_path[1] + ' '+directory+'/'+filename[1])

	def get_states(self):
		return np.array(self.states).astype('float32') 

	def set_terminated(self, idx):
		self.terminated[idx] = True
	
	def get_terminated(self, idx):
		return self.terminated[idx]
	
	def get_all_terminated(self):
		for i in range(self.num_slave):
			if not self.terminated[i]:
				return False
		return True 

	def reset(self, i):
		idx = np.random.randint(len(self.pool))
		param = self.pool[idx]
		self.sim_env.reset(i, np.array(param, dtype=np.float32))	

		state = np.array([self.sim_env.getState(i)])
		self.states[i] = self.RMS.apply(state)[0]
		self.terminated[i] = False
		self.rewards_slaves[i] = np.array([0.0]*len(self.reward_label))

	def step_sim(self, actions):
		rewards = []
		dones = []
		nt_starts = []
		nt_ends = []
		t_times = []
		nan_count = 0
		self.sim_env.setActionAll(actions)
		self.sim_env.stepAll()

		for j in range(self.num_slave):
			is_terminal, terminal_reason, nan_occur, nt_start, nt_end, t_elapsed = self.sim_env.getStepInfo(j)
			if not nan_occur:
				r = self.sim_env.getRewardVector(j)
				rewards.append(r)
				dones.append(is_terminal)
			else:
				rewards.append([None])
				dones.append(True)
				nan_count += 1

			t_times.append(t_elapsed)
			nt_starts.append(nt_start)
			nt_ends.append(nt_end)

		states = self.sim_env.getStateAll()
		return states, rewards, dones, nt_starts, nt_ends, t_times, nan_count 

	def step(self, actions, record=True):
		phases = self.sim_env.getCurrentFrame()
		self.states, rewards, dones,  nt_starts, nt_ends, t_times, nan_count = self.step_sim(actions)

		states_updated = self.RMS.apply(self.states[~np.array(self.terminated)])
		self.states[~np.array(self.terminated)] = states_updated
		params = self.sim_env.getCurrentFrameOnCycle()

		if record:
			self.num_nan_per_iteration += nan_count
			for i in range(self.num_slave):
				if not self.terminated[i] and rewards[i][0] != None:
					self.rewards_per_iteration += rewards[i][0]
					self.rewards_by_part_per_iteration.append(rewards[i])
					self.num_transitions_per_iteration += 1
					self.rewards_slaves[i] += rewards[i]
					if dones[i]:
						self.num_episodes_per_iteration += 1
						self.nt_elapsed_iteration += (nt_ends[i] - nt_starts[i])
						self.stats_per_iteration.append([nt_starts[i], nt_ends[i], self.rewards_slaves[i]])


		rewards = [[rewards[i][0], rewards[i][1], rewards[i][2]] for i in range(len(rewards))]

		return rewards, dones, phases

	def to_sinusoidal_param(self, param):
		param_sinusoidal = np.zeros(shape=(param.shape[0], param.shape[1]*2))
		for j in range(self.num_phi):
			param_sinusoidal[:,2*j] = np.sin(2 * np.pi * param[:,j] / self.motionlength[j])
			param_sinusoidal[:,2*j+1] = np.cos(2 * np.pi * param[:,j] / self.motionlength[j])
		return param_sinusoidal

	def normalize_param(self, param):
		param_normalized = np.zeros(shape=(param.shape[0], param.shape[1]*2))

		for j in range(self.num_phi):
			param_normalized[:,j] = param[:,j] / self.motionlength[j]
		return param_normalized

	def load_start_distribution(self, filename):
		self.pool = []
		f = open(filename, "r")
		lines = f.readlines()
		for l in lines:
			vec = []
			for l0 in l.split():
				vec.append(float(l0))
			self.pool.append(vec)
		f.close()

	def save_start_distribution(self, filename):
		f = open(filename, "w")
		for p in self.pool:
			for p0 in p:
				f.write(str(p0)+' ')
			f.write('\n')
		f.close()

	def update_rsi_and_reference(self):
		if self.fix_weight:	
			return
		self.pool = []
		paramlist = self.sim_env.buildParamTree()

		self.update_rsi_and_reference_visit_equal(paramlist)
		if not self.noreg:
			self.sim_env.optimizeReference()
			self.sim_env.trainRegressionNetwork()

	def update_rsi_and_reference_visit_equal(self, paramlist):
		num_batch = len(paramlist) // self.num_slave
		paramlist = paramlist[:self.num_slave*num_batch]
		candidates_density = self.sim_env.getDensity(np.array(paramlist, dtype=np.float32), num_batch)
		mean = np.mean(candidates_density)
		std = np.std(candidates_density)

		candidates_density, paramlist = zip(*sorted(zip(candidates_density, paramlist), key=lambda x: x[0]))		

		size = int(len(paramlist)*self.vsi_thres[0])
		paramlist = paramlist[-size:]
		ind = np.arange(len(paramlist))
		np.random.shuffle(ind)
	
		threshold = self.vsi_thres[1]
		for i in range(len(paramlist)):
			if len(self.pool) == 0:
				self.pool.append(paramlist[ind[i]])
			else:
				param_tiled = np.tile(paramlist[ind[i]], (len(self.pool), 1))
				dist = self.normalize_param(param_tiled - self.pool)
				dist = np.linalg.norm(dist, axis=1)
					
				exist = False
				for j in range(len(dist)):
					if dist[j] < threshold:
						exit = True 
						break

				if not exist:
					self.pool.append(paramlist[ind[i]])
			if len(self.pool) > 500:
				done = True
				break

	def plot_fig(self, y_list, title, num_fig=1, ylim=True, path=None):
		if self.plot:
			plt.figure(num_fig, clear=True, figsize=(5.5, 4))
		else:
			plt.figure(num_fig, figsize=(5.5, 4))
		plt.title(title)

		i = 0
		for y in y_list:
			plt.plot(y[0], label=y[1])
			i+= 1
		plt.legend(loc=2)
		if self.plot:
			plt.show()
			if ylim:
				plt.ylim([0,1])
			plt.pause(0.001)
		if path is not None:
			plt.savefig(path, format="png")

	def print_summary(self, save=True):
		r_per_e = self.rewards_per_iteration / self.num_episodes_per_iteration
		rp_per_i = np.array(self.rewards_by_part_per_iteration).sum(axis=0) / self.num_transitions_per_iteration
		if save:
			self.total_rewards.append(r_per_e)
			self.num_total_transitions += self.num_transitions_per_iteration
			self.num_total_episodes += self.num_episodes_per_iteration
			self.num_total_iterations += 1
			self.total_rewards_by_parts = np.insert(self.total_rewards_by_parts, self.total_rewards_by_parts.shape[1], 
				np.asarray(self.rewards_by_part_per_iteration).sum(axis=0)/self.num_episodes_per_iteration, axis=1)

			print_list = []
			print_list.append('===============================================================')
			print_list.append(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
			print_list.append("Elapsed time : {:.2f}s".format(time.time() - self.start_time))
			print_list.append('Num iter : {}'.format(self.num_total_iterations))
			print_list.append('total episode count : {}'.format(self.num_total_episodes))
			print_list.append('total transition count : {}'.format(self.num_total_transitions))

			t_per_e = 0
			if self.num_total_episodes != 0:
				t_per_e = self.num_total_transitions / self.num_total_episodes
			print_list.append('total transition per episodes : {:.2f}'.format(t_per_e))

			print_list.append('episode count : {}'.format(self.num_episodes_per_iteration))
			print_list.append('transition count : {}'.format(self.num_transitions_per_iteration))
			
			t_per_e = 0
			if self.num_episodes_per_iteration != 0:
				t_per_e = self.num_transitions_per_iteration / self.num_episodes_per_iteration
			self.transition_per_episodes_history.append(t_per_e)

			print_list.append('transition per episodes : {:.2f}'.format(t_per_e))
			print_list.append('rewards per episodes : {:.2f}'.format(self.total_rewards[-1]))

			nt_per_t  = 0
			if self.num_transitions_per_iteration != 0:
				nt_per_t = self.nt_elapsed_iteration / self.num_transitions_per_iteration;
			print_list.append('normalized time elapsed per transition : {}'.format(nt_per_t))		

			if self.num_nan_per_iteration != 0:
				print_list.append('nan count : {}'.format(self.num_nan_per_iteration))
			print_list.append('===============================================================')

			for s in print_list:
				print(s)
			
			if self.directory != None:
				out = open(self.directory+"results", "a")
				for s in print_list:
					out.write(s+'\n')
				out.close()

			if self.plot:
				y_list = [[np.asarray(self.transition_per_episodes_history), 'steps']]
				for i in range(len(self.total_rewards_by_parts)):
					y_list.append([np.asarray(self.total_rewards_by_parts[i]), self.reward_label[i]])

				self.plot_fig(y_list, "rewards" , 1, False, path=self.directory+"result.png")

				y_list = y_list[1:]
				for i in range(len(y_list)):
					y_list[i][0] = np.array(y_list[i][0])/np.array(self.transition_per_episodes_history)

				self.plot_fig(y_list, "rewards_per_step", 2, False, path=self.directory+"result_per_step.png")

		self.num_nan_per_iteration = 0
		self.num_episodes_per_iteration = 0
		self.num_transitions_per_iteration = 0
		self.rewards_per_iteration = 0
		self.rewards_by_part_per_iteration = []
		self.nt_elapsed_iteration = np.array([0.0]*self.num_phi)

		summary = dict()
		summary['r_per_e'] = r_per_e
		summary['rp_per_i'] = rp_per_i
		summary['t_per_e'] = t_per_e

		if not self.pretrain and self.num_total_iterations >= self.fix_time_iterations and self.num_total_iterations % 20 == 0:
			self.sim_env.setFixWeight(False)
			w = min(1.0, (self.num_total_iterations - (self.fix_time_iterations - 20)) / self.fix_time_iterations)
			self.sim_env.setWeightAll(w)
			if w > 0.5:
				self.fix_weight = False
		

		return summary
