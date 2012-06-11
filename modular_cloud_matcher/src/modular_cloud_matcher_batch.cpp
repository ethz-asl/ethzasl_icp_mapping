#define EIGEN2_SUPPORT_STAGE30_FULL_EIGEN3_API
#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <map>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"

using namespace std;
using namespace Eigen;
using namespace PointMatcherSupport;

// FIXME: switch to yaml?

typedef map<string, string> Params;

Params params;

bool hasParam(const std::string& name)
{
	return params.find(name) != params.end();
}

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

template<typename T>
T getParam(const std::string& name)
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
		cerr << "Cannot find value for parameter: " << name;
		return T();
	}
}

#include "icp_chain_creation.h"

struct Datum
{
	TP transform;
	DP cloud;
};

typedef vector<Datum> Data;

Data data;
TP T_v_to_k(TP::Identity(4, 4));
TP T_k_to_v(TP::Identity(4, 4));

int main(int argc, char **argv)
{
	if (argc < 11)
	{
		cerr << "Usage: " << argv[0] << " data params output t_x t_y t_z q_x q_y q_z q_w (transform from gt to icp, for correction) [tf_output] [delta_tf_output] [delta_tf_steps]" << endl;
		return 1;
	}
	
	srand(time(0));
	
	// read correction
	{
		Eigen::Vector3f tr;
		tr(0) = atof(argv[4]);
		tr(1) = atof(argv[5]);
		tr(2) = atof(argv[6]);
		Eigen::eigen2_Quaternionf rot;
		rot.x() = atof(argv[7]);
		rot.y() = atof(argv[8]);
		rot.z() = atof(argv[9]);
		rot.w() = atof(argv[10]);
		T_k_to_v = (Eigen::eigen2_Translation3f(tr) * rot).matrix();
		T_v_to_k = T_k_to_v.inverse();
		cout << "Using correction:\n";
		cout << T_k_to_v << "\n";
	}
	
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
			PM::Matrix tempCloud(4, pointCount);
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
	
	// tf output file
	ofstream tfofs;
	if (argc >= 12)
	{
		tfofs.open(argv[11]);
		if (!tfofs.good())
		{
			cerr << "Error, invalid tf output file " << argv[4] << endl;
			return 5;
		}
	}
	ofstream dtfofs;
	if (argc >= 13)
	{
		dtfofs.open(argv[12]);
		if (!tfofs.good())
		{
			cerr << "Error, invalid delta tf output file " << argv[5] << endl;
			return 6;
		}
	}
	int deltaTfSteps(30);
	if (argc >= 14)
	{
		deltaTfSteps = atoi(argv[13]);
		cout << "Using tf steps of " << deltaTfSteps << endl;
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
		if (params.empty())
			break;
		cout << "Exp " << expCount << ", loaded " << params.size() << " parameters\n\n";
		++expCount;
		
		// run experiment
		PM::ICPSequence icp;
		populateParameters(icp);
		
		// init icp
		timer t;
		DP d;
		const TP T_gt_init(data[0].transform);
		TP T_gt_old(T_gt_init);
		d = data[0].cloud;
		unsigned failCount(0);
		try
    {
      icp(d);
    }
    catch (PM::ConvergenceError error)
    {
      ++failCount;
      cerr << "ICP failed to converge at cloud 0 : " << error.what() << endl;
    }
		
    TP T_icp_old(T_gt_init * T_k_to_v * icp.getTransform() * T_v_to_k);
		
		TP T_d_gt_acc(TP::Identity(4,4));
		TP T_d_icp_acc(TP::Identity(4,4));
		
		// for each cloud, compute error
		for (size_t i = 1; i < data.size(); ++i)
		{
			// apply icp
			try 
			{
				d = data[i].cloud;
				icp(d);
			}
			catch (PM::ConvergenceError error)
			{
				++failCount;
				cerr << "ICP failed to converge at cloud " << i+1 << " : " << error.what() << endl;
			}
			
			// get ground-truth
			const TP T_gt(data[i].transform); 
			// compute T given ICP in ground-truth coordinates
			const TP T_icp = T_gt_init * T_k_to_v * icp.getTransform() * T_v_to_k;
			// compute ground-truth transfrom
			const TP T_d_gt = T_gt.inverse() * T_gt_old;
			// compute icp inverse transform
			const TP T_d_icp = T_icp.inverse() * T_icp_old;
			
			/*cerr << "T_gt_init:\n" << T_gt_init << "\nT_gt:\n" << T_gt << "\nT_k_to_v:\n" << T_k_to_v << "\nicp::getTransform:\n" << icp.getTransform() << "\nT_v_to_k:\n" << T_v_to_k << "\n\nT_icp:\n" << T_icp << endl;
			cerr << "det icp::getTransform: " << icp.getTransform().determinant() << ", det T_icp: " << T_icp .determinant() << endl;*/
			
			// write output
			if (argc >= 12)
			{
				// tf
				const Vector3 t_gt(T_gt.topRightCorner(3,1));
				const Quaternion<Scalar> q_gt(Matrix3(T_gt.topLeftCorner(3,3)));
				tfofs << t_gt(0) << " " << t_gt(1) << " " << t_gt(2) << " " << q_gt.x() << " " << q_gt.y() << " " << q_gt.z() << " " << q_gt.w() << " ";
				const Vector3 t_icp(T_icp.topRightCorner(3,1));
				const Quaternion<Scalar> q_icp(Matrix3(T_icp.topLeftCorner(3,3)));
				tfofs << t_icp(0) << " " << t_icp(1) << " " << t_icp(2) << " " << q_icp.x() << " " << q_icp.y() << " " << q_icp.z() << " " << q_icp.w() << "\n";
			}
			
			// dump deltas
			/*if (argc >= 6)
			{
				// delta tf
				const Vector3 t_gt(T_d_gt.topRightCorner(3,1));
				const Quaternion<Scalar> q_gt(Matrix3(T_d_gt.topLeftCorner(3,3)));
				dtfofs << t_gt(0) << " " << t_gt(1) << " " << t_gt(2) << " " << q_gt.x() << " " << q_gt.y() << " " << q_gt.z() << " " << q_gt.w() << " ";
				const Vector3 t_icp(T_d_icp.topRightCorner(3,1));
				const Quaternion<Scalar> q_icp(Matrix3(T_d_icp.topLeftCorner(3,3)));
				dtfofs << t_icp(0) << " " << t_icp(1) << " " << t_icp(2) << " " << q_icp.x() << " " << q_icp.y() << " " << q_icp.z() << " " << q_icp.w() << "\n";
			}*/
			
			// compute errors
			const TP T_d = T_d_gt * T_d_icp.inverse();
			{
				const Vector3 e_t(T_d.topRightCorner(3,1));
				icp.inspector->addStat("e_x", e_t(0));
				icp.inspector->addStat("e_y", e_t(1));
				icp.inspector->addStat("e_z", e_t(2));
				const Quaternion<Scalar> quat(Matrix3(T_d.topLeftCorner(3,3)));
				icp.inspector->addStat("e_a", 2 * acos(quat.normalized().w()));
			}
			
			if (i % deltaTfSteps == 0)
			{
				// compute difference
				const TP T_d_acc = T_d_gt_acc * T_d_icp_acc.inverse();
				
				// compute errors
				const Vector3 e_t(T_d_acc.topRightCorner(3,1));
				icp.inspector->addStat("e_acc_x", e_t(0));
				icp.inspector->addStat("e_acc_y", e_t(1));
				icp.inspector->addStat("e_acc_z", e_t(2));
				const Quaternion<Scalar> quat(Matrix3(T_d_acc.topLeftCorner(3,3)));
				icp.inspector->addStat("e_acc_a", 2 * acos(quat.normalized().w()));
				
				// dump deltas
				if (argc >= 13)
				{
					// delta tf
					const Vector3 t_gt(T_d_gt_acc.topRightCorner(3,1));
					const Quaternion<Scalar> q_gt(Matrix3(T_d_gt_acc.topLeftCorner(3,3)));
					dtfofs << t_gt(0) << " " << t_gt(1) << " " << t_gt(2) << " " << q_gt.x() << " " << q_gt.y() << " " << q_gt.z() << " " << q_gt.w() << " ";
					const Vector3 t_icp(T_d_icp_acc.topRightCorner(3,1));
					const Quaternion<Scalar> q_icp(Matrix3(T_d_icp_acc.topLeftCorner(3,3)));
					dtfofs << t_icp(0) << " " << t_icp(1) << " " << t_icp(2) << " " << q_icp.x() << " " << q_icp.y() << " " << q_icp.z() << " " << q_icp.w() << "\n";
				}
				
				// reset accumulators
				T_d_gt_acc = TP::Identity(4,4);
				T_d_icp_acc = TP::Identity(4,4);
			}
			else
			{
				T_d_gt_acc = T_d_gt * T_d_gt_acc;
				T_d_icp_acc = T_d_icp * T_d_icp_acc;
			}
			
			// write back transforms
			T_gt_old = T_gt;
			T_icp_old = T_icp;
		}
		const double icpTotalDuration(t.elapsed());
		
		// write back results
		// general stats
		ofs << data.size() - 1 << " " << failCount << " * ";
		ofs << icpTotalDuration << " * ";
		icp.inspector->dumpStats(ofs);
		ofs << " # ";
		// params for info
		for (Params::const_iterator it(params.begin()); it != params.end(); ++it)
			ofs << it->first << "=" << it->second << " ";
		// finish results
		ofs << "\n";
	}

	return 0;
}
