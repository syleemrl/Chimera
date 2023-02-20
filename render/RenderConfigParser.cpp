#include <fstream>
#include <iostream>
#include "RenderConfigParser.h"
#include "Functions.h"

std::pair<std::string, std::vector<RenderData>> 
RenderConfigParser::
parseRenderConfig(std::string _path)
{
	std::pair<std::string, std::vector<RenderData>> renderData;

	std::ifstream is(_path);
	char buffer[256];

	if(!is)
	{
		std::cout << "Can't open render config file: " << _path << std::endl;
		return renderData;
	}

	SIM::getWordsUntil(is, "type:");
	is >> buffer;
	renderData.first = SIM::getString(buffer);

	SIM::getWordsUntil(is, "data:");
	SIM::getWordsUntil(is, "{");

	
	while(is >> buffer)
	{	
		if(!strcmp(buffer,"{")) {
			RenderData data;
			while(is >> buffer) {
				if(!strcmp(buffer,"{")) {
					SIM::getWordsUntil(is, "character:");
					is >> buffer;
					std::string character = SIM::getString(buffer);

					SIM::getWordsUntil(is, "motion:");
					is >> buffer;
					std::string motion = SIM::getString(buffer);					

					data.CMpair.push_back(std::pair<std::string, std::string>(character, motion));					
					SIM::getWordsUntil(is, "}");

				} else if(!strcmp(buffer,"motion:")) {
					is >> buffer;
					if(data.CMpair.size() == 0) {
						data.CMpair.push_back(std::pair<std::string, std::string>("", SIM::getString(buffer)));					
					} else {
						data.CMpair[0].second = SIM::getString(buffer);
					}
				} else if(!strcmp(buffer,"character:")) {
					is >> buffer;
					if(data.CMpair.size() == 0) {
						data.CMpair.push_back(std::pair<std::string, std::string>(SIM::getString(buffer), ""));					
					} else {
						data.CMpair[0].first = SIM::getString(buffer);
					}
				} else if(!strcmp(buffer,"network:")) {
					is >> buffer;
					data.network = SIM::getString(buffer);
				} else if(!strcmp(buffer,"config:")) {
					is >> buffer;
					data.simConfig = SIM::getString(buffer);
				} else if(!strcmp(buffer,"}")) {
					renderData.second.push_back(data);
					break;
				}
			}
		} else if(!strcmp(buffer,"}"))
		break;
	}
	
	return renderData;
}
std::vector<std::string> 
RenderConfigParser::
getFilenames(std::vector<RenderData> _data)
{
	std::vector<std::string> result;
	for(int i = 0; i < _data.size(); i++) {
		if(_data[i].CMpair.size() != 0)
			result.push_back(_data[i].CMpair[0].second);
		else if(_data[i].simConfig != "")
			result.push_back(_data[i].simConfig);
		else			
			result.push_back(_data[i].network);

	}
	return result;
}
std::vector<std::string> 
RenderConfigParser::
getSkelnames(std::vector<RenderData> _data, int _idx)
{
	std::vector<std::string> result;
	for(int i = 0; i < _data[_idx].CMpair.size(); i++) {
		result.push_back(_data[_idx].CMpair[i].first);
	}
	return result;
}
std::pair<bool, std::vector<Eigen::VectorXd>>
RenderConfigParser::
parseStartDistribution(std::string _file, int _n)
{
	std::vector<Eigen::VectorXd> pool;
	std::ifstream is(_file);

	if(!is.is_open()) {
		return std::pair<bool, std::vector<Eigen::VectorXd>>(false, pool);
	}
	char buffer[256];
	while(is >> buffer)
	{
		Eigen::VectorXd vec(_n);
		for(int i = 0; i <_n; i++) {
			if(i != 0)
				is >> buffer;
			double t = atof(buffer);
			vec(i) = t;
		}
		pool.push_back(vec);
	}
	is.close();
	return std::pair<bool, std::vector<Eigen::VectorXd>>(true, pool);
}
void
RenderConfigParser::
printRenderData(std::vector<RenderData> _data)
{
	for(int i = 0; i < _data.size(); i++) {
		std::cout << "data " << i << std::endl;
		if(_data[i].network != "")
			std::cout << "\tnetwork: " << _data[i].network << std::endl;
		if(_data[i].simConfig != "")
			std::cout << "\tsim config: " << _data[i].simConfig << std::endl;

		for(int j = 0; j < _data[i].CMpair.size(); j++) {
			std::cout << "\tpair " << j << std::endl;
			std::cout << "\t\tcharacter: " << _data[i].CMpair[j].first << std::endl;
			std::cout << "\t\tmotion: " << _data[i].CMpair[j].second << std::endl;
		}
	}
}
