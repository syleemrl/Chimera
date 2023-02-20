#ifndef __RENDER_CONFIG_PARSER_H__
#define __RENDER_CONFIG_PARSER_H__
#include <string>
#include <vector>
#include <Eigen/Dense>
struct RenderData
{
	std::vector<std::pair<std::string, std::string>> CMpair;
	std::string simConfig="";
	std::string network="";
};
class RenderConfigParser
{
public:
	static std::pair<std::string, std::vector<RenderData>> parseRenderConfig(std::string _path);
	static std::vector<std::string> getFilenames(std::vector<RenderData> _data);
	static std::vector<std::string> getSkelnames(std::vector<RenderData> _data,  int _idx);
	static std::pair<bool, std::vector<Eigen::VectorXd>> parseStartDistribution(std::string _file, int _n);
	static void printRenderData(std::vector<RenderData> _data);
};
#endif