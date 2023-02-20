#ifndef __SIM_CONFIG_PARSER_H__
#define __SIM_CONFIG_PARSER_H__
#include <string>
#include <vector>
#include <map>
#include <Eigen/Core>
namespace SIM
{
struct ProcessingOption
{
	ProcessingOption() {}
	bool blend = false;
	int blendInterval = 2;
};
struct TrainingOption
{
	bool enableSelfCollision = false;
	bool fixCycleEnd = true;
	int timewarpingType = 2;
	bool useReferenceRoot = false;
};
struct TerminalCondition
{
	int terminalIteration;
	double headDiffPos;
	double rootDiffPos;
	double rootDiffDir;
	double rootHeightLower;
	double rootHeightUpper;
};
struct Constraint
{
	bool loadConstraint=false;
	double speed=1.0;
	Eigen::VectorXd baseStyle;
	std::vector<Eigen::VectorXd> constraintSync;
	std::vector<std::tuple<int, double, double>> constraintStyle;
};
struct SimConfig
{
	std::string charPath;
	bool isAssembled;
	std::string assembledSkel;

	int numBase;
	std::vector<std::pair<Eigen::Vector3i, Eigen::Vector3i>> axisOrder;
	std::vector<std::string> baseSkels;
	std::vector<std::string> baseMotions;	

	std::vector<std::string> posJoints;
	std::vector<std::string> eeJoints;

	int numTimingFunc;

	std::map<int, int> timingFuncBase;
	std::map<std::string, int> timingFuncs;
	std::map<std::string, double> torqueLimits;
	std::map<std::string, double> kps;

	TerminalCondition tc;
	ProcessingOption po;
	TrainingOption to;
	Constraint c;
};
class SimConfigParser
{
public:
	static SimConfig parseCharConfig(std::string _path);
	static SimConfig parseSimConfig(std::string _path);
	static void printSimConfig(SimConfig _data);
};
}
#endif