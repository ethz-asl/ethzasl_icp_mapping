#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <map>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <boost/progress.hpp>

using namespace std;
using namespace Eigen;

typedef map<string, string> Params;

Params params;

template<typename T>
T getParam(const std::string& name, const T& defaultValue)
{
	Params::const_iterator it(params.find(name));
	if (it != params.end())
	{
		T v;
		istringstream iss(it->second);
		iss >> v;
		cerr << "Found parameter: " << name << ", value: " << v << endl;
		return v;
	}
	else
	{
		cerr << "Cannot find value for parameter: " << name << ", assigning default: " << defaultValue << endl;
		return defaultValue;
	}
}

#include "icp_chain_creation.h"
#include "pointmatcher/PointMatcher.h"

struct Datum
{
	TP transform;
	DP cloud;
};

typedef vector<Datum> Data;

Data data;

int main(int argc, char **argv)
{
	if (argc != 4)
	{
		cerr << "Usage: " << argv[0] << " data params output" << endl;
		return 1;
	}
	
	initParameters();
	
	// load data
	{
		// create labels
		DP::Labels labels;
		labels.push_back(DP::Label("x", 1));
		labels.push_back(DP::Label("y", 1));
		labels.push_back(DP::Label("z", 1));
		labels.push_back(DP::Label("pad", 1));
	
		// open file
		ifstream ifs(argv[1]);
		if (!ifs.good())
		{
			cerr << "Error, invalid data file " << argv[1] << endl;
			return 2;
		}
		char line[1024];
		
		// file header
		ifs.getline(line, 1024);
		cout << "Header: " << line << endl;
		// cloud count
		size_t cloudCount;
		ifs >> cloudCount;
		ifs.getline(line, 1024);
		cout << "Reading " << cloudCount << " clouds\n\n";
		data.reserve(cloudCount);
		for (size_t i = 0; i < cloudCount; ++i)
		{
			Datum datum;
			// cloud header
			ifs.getline(line, 1024);
			cout << "Cloud header: " << line << endl;
			// transform
			Scalar t_x, t_y, t_z, q_x,q_y,q_z,q_w;
			ifs >> t_x;
			ifs >> t_y;
			ifs >> t_z;
			ifs >> q_x;
			ifs >> q_y;
			ifs >> q_z;
			ifs >> q_w;
			cout << "Transform " << i << ": trans=(" << t_x << "," << t_y << "," << t_z << "), quat=(" << q_x << "," << q_y << "," << q_z << "," << q_w << ")\n";
			const Vector3 vect(t_x, t_y, t_z);
			const Quaternion<Scalar> quat(q_w, q_x, q_y, q_z);
			const Transform<Scalar,3,Affine> tr = Translation<Scalar,3>(vect) * quat;
			datum.transform = tr.matrix();
			// point count
			size_t pointCount;
			ifs >> pointCount;
			DP::Features tempCloud(4, pointCount);
			int dIndex(0);
			for (size_t j = 0; j < pointCount; ++j)
			{
				string sx, sy, sz;
				ifs >> sx;
				ifs >> sy;
				ifs >> sz;
				if (sx != "nan" && sy != "nan" && sz != "nan")
				{
					tempCloud(0, dIndex) = atof(sx.c_str());
					tempCloud(1, dIndex) = atof(sy.c_str());
					tempCloud(2, dIndex) = atof(sz.c_str());
					//cout << t_x << " " << t_y << " " << t_z << "\n";
					tempCloud(3, dIndex) = 1;
					++dIndex;
				}
				else
				{
					//cout << "nan at pos " << j << "\n";
				}
			}
			cout << "Cloud " << i << ", " << dIndex << " good points on " << pointCount << "\n\n";
			datum.cloud = DP(tempCloud.leftCols(dIndex), labels);
			data.push_back(datum);
			ifs.getline(line, 1024);
		}
	}
	if (data.size() < 2)
	{
		cerr << "Data must have at least 2 clouds, but only has " << data.size() << endl;
		return 2;
	}
	
	// load param list
	
	// open file
	ifstream ifs(argv[2]);
	if (!ifs.good())
	{
		cerr << "Error, invalid params file " << argv[2] << endl;
		return 3;
	}
	ofstream ofs(argv[3]);
	if (!ofs.good())
	{
		cerr << "Error, invalid output file " << argv[3] << endl;
		return 4;
	}
	
	// for each line in the experiment file
	unsigned expCount(0);
	while (ifs.good())
	{
		// read line and load params
		char line[65536];
		ifs.getline(line, sizeof(line));
		params.clear();
		istringstream iss(line);
		while (iss.good())
		{
			string keyVal;
			iss >> keyVal;
			if (!keyVal.empty())
			{
				const size_t delPos = keyVal.find_first_of('=');
				if (delPos != string::npos)
				{
					const string key = keyVal.substr(0, delPos);
					const string val = keyVal.substr(delPos+1);
					params[key] = val;
				}
			}
		}
		cout << "Exp " << expCount << ", loaded " << params.size() << " parameters\n\n";
		++expCount;
		
		// run experiment
		MSA::ICPSequence icp(3, "", false);
		populateParameters(icp);
		Histogram<Scalar> e_x(16, "e_x", "", false), e_y(16, "e_y", "", false), e_z(16, "e_z", "", false), e_a(16, "e_a", "", false);
		
		// init icp
		boost::timer t;
		TP T_gt_old(data[0].transform);
		icp(data[0].cloud);
		TP T_icp_old(icp.getTransform());
		
		// for each cloud, compute error
		unsigned failCount(0);
		for (size_t i = 1; i < data.size(); ++i)
		{
			// apply icp
			const TP T_gt(data[i].transform);
			try 
			{
				icp(data[i].cloud);
			}
			catch (MSA::ConvergenceError error)
			{
				++failCount;
				cerr << "ICP failed to converge at cloud " << i+1 << " : " << error.what() << endl;
			}
			const TP T_icp(icp.getTransform());
			// compute ground-truth transfrom
			const TP T_d_gt = T_gt * T_gt_old.inverse();
			// compute icp inverse transform
			const TP T_di_icp = T_icp_old * T_icp.inverse();
			// compute diff
			const TP T_d = T_d_gt * T_di_icp;
			
			// compute errors
			const Vector3 e_t(T_d.topRightCorner(3,1));
			e_x.push_back(e_t(0));
			e_y.push_back(e_t(1));
			e_z.push_back(e_t(2));
			const Quaternion<Scalar> quat(Matrix3(T_d.topLeftCorner(3,3)));
			e_a.push_back(2 * acos(quat.w()));
			
			// write back transforms
			T_gt_old = T_gt;
			T_icp_old = T_icp;
		}
		const double icpTotalDuration(t.elapsed());
		
		// write back results
		// general stats
		ofs << data.size() - 1 << " " << failCount << " ";
		// error
		e_x.dumpStats(ofs); ofs << " ";
		e_y.dumpStats(ofs); ofs << " ";
		e_z.dumpStats(ofs); ofs << " ";
		e_a.dumpStats(ofs); ofs << " ";
		// timing
		ofs << icpTotalDuration << " ";
		icp.keyFrameDuration.dumpStats(ofs); ofs << " ";
		icp.convergenceDuration.dumpStats(ofs); ofs << " ";
		// algo stats
		icp.iterationsCount.dumpStats(ofs); ofs << " ";
		icp.pointCountIn.dumpStats(ofs); ofs << " ";
		icp.pointCountReading.dumpStats(ofs); ofs << " ";
		icp.pointCountKeyFrame.dumpStats(ofs); ofs << " ";
		icp.pointCountTouched.dumpStats(ofs); ofs << " ";
		icp.overlapRatio.dumpStats(ofs); ofs << " ";
		// params for info
		ofs << "# ";
		for (Params::const_iterator it(params.begin()); it != params.end(); ++it)
			ofs << it->first << "=" << it->second << " ";
		// finish results
		ofs << "\n";
	}

	return 0;
}
