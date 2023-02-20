#include <fstream>
#include <iostream>
#include "SimConfigParser.h"
#include "Functions.h"

namespace SIM
{
SimConfig 
SimConfigParser::
parseCharConfig(std::string _path)
{
	SimConfig simConfig;
	std::ifstream is;
	char buffer[256];

	findAttribute(_path, is, "assembled:");
	if(!is.eof()) {
		is >> buffer;
		simConfig.assembledSkel = getString(buffer);
		simConfig.isAssembled = true;
	} else {
		simConfig.isAssembled = false;		
	}

	findAttribute(_path, is, "num_base:");
	if(!is.eof()) {
		is >> buffer;
		simConfig.numBase = atoi(buffer);	
	}

	findAttribute(_path, is, "base:");
	if(!is.eof()) {
		getWordsUntil(is, "{");

		for(int i = 0; i < simConfig.numBase; i++) {
			getWordsUntil(is, "name:");
			is >> buffer;
			simConfig.baseSkels.push_back(getString(buffer));
			getWordsUntil(is, "axis:");

			Eigen::Vector3i sign;
			Eigen::Vector3i axis;
			for(int i = 0; i < 3; i++) {
				is >> buffer;
				
				std::string s = "";
				s += buffer[0];
				axis(i) = atoi(s.c_str());

				if(buffer[1] == '+')
					sign(i) = 1;
				else
					sign(i) = -1;
			}
			simConfig.axisOrder.push_back(std::pair<Eigen::Vector3i, Eigen::Vector3i>(axis, sign));
		}
		getWordsUntil(is, "}");
	}



	findAttribute(_path, is, "timing_func_base:");
	int numTimingFunc = 0;
	if(!is.eof()) {
		getWordsUntil(is, "{");
		while(!is.eof()) {
			char* key = getWordsUntil(is, "{", "}");
			if(!strcmp(key,"}")) {
				break;
			}
			getWordsUntil(is, "timing_func_idx:");
			is >> buffer;
			int idx = atoi(buffer);

			getWordsUntil(is, "base:");
			is >> buffer;
			simConfig.timingFuncBase.insert(std::pair<int, int>(idx, atoi(buffer)));
			getWordsUntil(is, "}");
			numTimingFunc += 1;
		}
	}
	simConfig.numTimingFunc = numTimingFunc;
	
	findAttribute(_path, is, "joint_setting:");
	getWordsUntil(is, "{");
	while(!is.eof()) {
		is >> buffer;

		if(!strcmp(buffer,"}"))
			break;
		else if(!strcmp(buffer,"{")) {
			getWordsUntil(is, "name:");
			is >> buffer;
			std::string joint = getString(buffer);
			while(is >> buffer) {
				if(!strcmp(buffer,"torque_limits:")) {
					is >> buffer;
					simConfig.torqueLimits.insert(std::pair<std::string, double>(joint, atof(buffer)));
				} else if(!strcmp(buffer,"kp:")) {
					is >> buffer;
					simConfig.kps.insert(std::pair<std::string, double>(joint, atof(buffer)));
				} else if(!strcmp(buffer,"timing_func_idx:")) {
					is >> buffer;
					simConfig.timingFuncs.insert(std::pair<std::string, int>(joint, atoi(buffer)));
				} else if(!strcmp(buffer,"ee_joint:")) {
					is >> buffer;
					if(atoi(buffer) == 1)
						simConfig.eeJoints.push_back(joint);
				} else if(!strcmp(buffer,"pos_joint:")) {
					is >> buffer;
					if(atoi(buffer) == 1)
						simConfig.posJoints.push_back(joint);
				} else if(!strcmp(buffer,"}")) {
					if(simConfig.torqueLimits.find(joint) != simConfig.torqueLimits.end())
						simConfig.torqueLimits.insert(std::pair<std::string, double>(joint, 1e6));
					break;
				}
			}
		}
	}

	return simConfig;
}
SimConfig
SimConfigParser::
parseSimConfig(std::string _path)
{
	std::ifstream is;
	char buffer[256];

	findAttribute(_path, is, "character_sim:");
	is >> buffer;
	std::string charPath = getDirectoryName(_path) + getString(buffer);
	SimConfig simConfig = parseCharConfig(charPath);
	simConfig.charPath = charPath;
	
	findAttribute(_path, is, "motion:");
	if(!is.eof()) {
		getWordsUntil(is, "{");
		for(int j = 0; j  < simConfig.numTimingFunc; j++) {
			is >> buffer;
			simConfig.baseMotions.push_back(getString(buffer));
		}
		getWordsUntil(is, "}");
	}
	
	
	findAttribute(_path, is, "motion_processing:");
	if(!is.eof()) {
		getWordsUntil(is, "{");
		while(is >> buffer) {
			if(!strcmp(buffer,"blend:")) {
				is >> buffer;
				simConfig.po.blend = atoi(buffer);
			} else if(!strcmp(buffer,"blend_interval:")) {
				is >> buffer;
				simConfig.po.blendInterval = atoi(buffer);
			} else if(!strcmp(buffer,"}"))
				break;
		}
	}

	findAttribute(_path, is, "terminal_condition:");
	if(!is.eof()) {
		getWordsUntil(is, "{");
		while(is >> buffer) {
			if(!strcmp(buffer,"iteration:")) {
				is >> buffer;
				simConfig.tc.terminalIteration = atoi(buffer);
			} else if(!strcmp(buffer,"root_diff_pos:")) {
				is >> buffer;
				simConfig.tc.rootDiffPos = atof(buffer);
			} else if(!strcmp(buffer,"head_diff_pos:")) {
				is >> buffer;
				simConfig.tc.headDiffPos = atof(buffer);
			} else if(!strcmp(buffer,"root_diff_dir:")) {
				is >> buffer;
				simConfig.tc.rootDiffDir = atof(buffer);
			} else if(!strcmp(buffer,"root_height_lower:")) {
				is >> buffer;
				simConfig.tc.rootHeightLower = atof(buffer);
			} else if(!strcmp(buffer,"root_height_upper:")) {
				is >> buffer;
				simConfig.tc.rootHeightUpper = atof(buffer);
			} else if(!strcmp(buffer,"}"))
				break;
		}
	}
	findAttribute(_path, is, "training_option:");
	if(!is.eof()) {
		getWordsUntil(is, "{");
		while(is >> buffer) {
			if(!strcmp(buffer,"enable_self_collsion:")) {
				is >> buffer;
				simConfig.to.enableSelfCollision = atoi(buffer);
			} else if(!strcmp(buffer,"fix_cycle_end:")) {
				is >> buffer;
				simConfig.to.fixCycleEnd = atoi(buffer);
			} else if(!strcmp(buffer,"timewarping_type:")) {
				is >> buffer;
				simConfig.to.timewarpingType = atoi(buffer);
			} else if(!strcmp(buffer,"use_reference_root:")) {
				is >> buffer;
				simConfig.to.useReferenceRoot = atoi(buffer);
			} else if(!strcmp(buffer,"}"))
				break;
		}
	}

	findAttribute(_path, is, "constraints:");
	if(!is.eof()) {
		simConfig.c.loadConstraint = true;
		getWordsUntil(is, "{");
		while(is >> buffer) {
			if(!strcmp(buffer,"speed:")) {
				is >> buffer;
				simConfig.c.speed = atof(buffer);
			} else if(!strcmp(buffer,"base_style:")) {
				Eigen::VectorXd style(simConfig.numTimingFunc);
				for(int i = 0; i < style.rows(); i++) {
					is >> buffer;
					style(i) = atof(buffer);
				}
				simConfig.c.baseStyle = style;
			} else if(!strcmp(buffer,"constraints_sync:")) {
				is >> buffer;
				while(1) {
					Eigen::VectorXd phase(simConfig.numTimingFunc);

					is >> buffer;
					if(!strcmp(buffer,"}"))
						break;
					phase(0) = atof(buffer);
					for(int i = 1; i < phase.rows(); i++) {
						is >> buffer;
						phase(i) = atof(buffer);
					}
					simConfig.c.constraintSync.push_back(phase);
				}
			} else if(!strcmp(buffer,"constraints_style:")) {
				is >> buffer;
				while(1) {
					std::tuple<int, double, double> style;
					is >> buffer;
					if(!strcmp(buffer,"}"))
						break;
					std::get<0>(style) = atoi(buffer);
					is >> buffer;
					std::get<1>(style) = atof(buffer);
					is >> buffer;
					std::get<2>(style) = atof(buffer);
					
					simConfig.c.constraintStyle.push_back(style);
				}
			} else if(!strcmp(buffer,"}"))
				break;
		}
	}

	return simConfig;
}
void 
SimConfigParser::
printSimConfig(SimConfig _data)
{
	std::cout << "is assembled : " << _data.isAssembled << std::endl;
	if(_data.isAssembled)
		std::cout << "assembled skel: " << _data.assembledSkel << std::endl;

	std::cout << "num base: " << _data.numBase << std::endl;
	for(int i = 0; i < _data.numBase; i++) {
		std::cout << "skel: " << _data.baseSkels[i] << std::endl;
	}
	std::cout << "num timing functions: " << _data.numTimingFunc << std::endl;
	for(int i = 0; i < _data.numTimingFunc; i++) {
		std::cout << "motion: " << _data.baseMotions[i] << std::endl;
	}

	for(auto it = _data.kps.begin(); it != _data.kps.end(); it++) {
		std::string name = it->first;

		std::cout << "joint " << name << std::endl;
		std::cout << "kp: " << _data.kps[name] << std::endl;
		std::cout << "timing func idx: " << _data.timingFuncs[name] << std::endl;

	}
}
}