
#define EIGEN2_SUPPORT_STAGE30_FULL_EIGEN3_API
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <map>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include "boost/foreach.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/tfMessage.h"
#include "tf2/buffer_core.h"
#include "rosbag/bag.h"
#include "rosbag/query.h"
#include "rosbag/view.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h" 
#include "pcl/ros/conversions.h"
#include "sensor_msgs/CameraInfo.h"
#include <argtable2.h>
#include <regex.h>      /* REG_ICASE */

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

void reloadParamsFromLine(ifstream& ifs)
{
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
}

void openWriteFile(ofstream& stream, arg_file * file, const int errorVal)
{
	if (file->count > 0)
	{
		stream.open(file->filename[0]);
		if (!stream.good())
		{
			cerr << "Error, invalid " << file->hdr.glossary << " " << file->filename[0] << endl;
			exit (errorVal);
		}
		else
			cout << "Writing to " << file->hdr.glossary << " " << file->filename[0] << endl;
	}
}

#include "icp_chain_creation.h"
#include "pointmatcher/PointMatcher.h"

struct Datum
{
	TP transform;
	DP cloud;
	ros::Time stamp;
};

typedef vector<Datum> Data;

Data data;
TP T_v_to_k(TP::Identity(4, 4));
TP T_k_to_v(TP::Identity(4, 4));

TP msgTransformToTP(const geometry_msgs::Transform& transform)
{
	const geometry_msgs::Vector3 gv(transform.translation);
	const geometry_msgs::Quaternion gq(transform.rotation);
	const Vector3 vect(gv.x, gv.y, gv.z);
	const Quaternion<Scalar> quat(gq.w, gq.x, gq.y, gq.z);
	const Transform<Scalar,3,Affine> tr = Translation<Scalar,3>(vect) * quat;
	return tr.matrix();
}

int main(int argc, char **argv)
{
	arg_file * bagFile = arg_file1(NULL, NULL, "<bag>", "bag file (mandatory)");
	arg_str  * cloudTopic = arg_str1(NULL, NULL, "<cloud_topic>", "cloud topic (mandatory)");
	arg_file * paramsFile = arg_file1(NULL, NULL, "<params>", "params file (mandatory)");
	arg_file * statFile = arg_file0(NULL, "stat", NULL, "statistic output file (default: no output)");
	
	arg_rex  * cmdGt = arg_rex1(NULL, NULL, "gt", NULL, REG_ICASE, "use ground truth");

	arg_str  * gtTargetFrame = arg_str1(NULL, "gt_target_frame", NULL, "tf target frame of ground truth (mandatory)");
	arg_str  * gtSourceFrame = arg_str1(NULL, "gt_source_frame", NULL, "tf source frame of ground truth (mandatory)");
	
	arg_dbl  * t_x = arg_dbl0(NULL, "t_x", NULL, "correction from gt to icp, translation on x (default: 0)");
	arg_dbl  * t_y = arg_dbl0(NULL, "t_y", NULL, "correction from gt to icp, translation on y (default: 0)");
	arg_dbl  * t_z = arg_dbl0(NULL, "t_z", NULL, "correction from gt to icp, translation on z (default: 0)");
	arg_dbl  * q_x = arg_dbl0(NULL, "q_x", NULL, "correction from gt to icp, quaternion component x (default: 0)");
	arg_dbl  * q_y = arg_dbl0(NULL, "q_y", NULL, "correction from gt to icp, quaternion component y (default: 0)");
	arg_dbl  * q_z = arg_dbl0(NULL, "q_z", NULL, "correction from gt to icp, quaternion component z (default: 0)");
	arg_dbl  * q_w = arg_dbl0(NULL, "q_w", NULL, "correction from gt to icp, quaternion component w (default: 1)");
	
	arg_file * tfFile = arg_file0(NULL, "tf", NULL, "tf output file (default: no output)");
	arg_file * dtfFile = arg_file0(NULL, "dtf", NULL, "delta tf output file (default: no output)");
	arg_int  * dtfSteps = arg_int0(NULL, "dtf_steps", NULL, "steps for computing delta tf (default: 30)");
	struct arg_end  * end = arg_end(20);
	
	void *argTableCloud[] = {bagFile, cloudTopic, paramsFile, statFile, tfFile, end};
	void* argTableCloudGt[] = {bagFile, cloudTopic, paramsFile, cmdGt, gtTargetFrame, gtSourceFrame, t_x, t_y, t_z, q_x, q_y, q_z, q_w, statFile, tfFile, dtfFile, dtfSteps, end};
		
	if (arg_nullcheck(argTableCloud) != 0 || 
		arg_nullcheck(argTableCloudGt) != 0)
		abort();
	
	const int nerrorsCloudGt = arg_parse(argc,argv,argTableCloudGt);
	const int nerrorsCloud = arg_parse(argc,argv,argTableCloud);
	if (nerrorsCloud > 0 && nerrorsCloudGt > 0)
	{
		cout << "Error(s) in command line:\n";
		arg_print_errors(stdout,end,argv[0]);
		cout << "\n";
		cout << "Usage, for cloud without ground-truth:\n";
		cout << argv[0];
		arg_print_syntax(stdout,argTableCloud,"\n");
		arg_print_glossary(stdout,argTableCloud,"  %-28s %s\n");
		cout << "\n";
		cout << "Usage, for cloud with ground-truth:\n";
		cout << argv[0];
		arg_print_syntax(stdout,argTableCloudGt,"\n");
		arg_print_glossary(stdout,argTableCloudGt,"  %-28s %s\n");
		return 1;
	}
	
	const bool useGt = (nerrorsCloudGt == 0);
	
	srand(time(0));
	
	initParameters();

	// setup correction
	if (useGt)
	{
		Eigen::Vector3f tr;
		tr(0) = t_x->count > 0 ? t_x->dval[0] : 0;
		tr(1) = t_y->count > 0 ? t_y->dval[0] : 0;
		tr(2) = t_z->count > 0 ? t_z->dval[0] : 0;
		Eigen::eigen2_Quaternionf rot;
		rot.x() = q_x->count > 0 ? q_x->dval[0] : 0;
		rot.y() = q_y->count > 0 ? q_y->dval[0] : 0;
		rot.z() = q_z->count > 0 ? q_z->dval[0] : 0;
		rot.w() = q_w->count > 0 ? q_w->dval[0] : 1;
		T_k_to_v = (Eigen::eigen2_Translation3f(tr) * rot).matrix();
		T_v_to_k = T_k_to_v.inverse();
		cout << "Using ground-truth with correction:\n";
		cout << T_k_to_v << "\n";
	}
	else
		cout << "Not using ground-truth" << endl;
	
	// load data
	try
	{
		sensor_msgs::CameraInfo camInfo;
		
		// 1 year of buffer
		tf2::BufferCore tfBuffer(ros::Duration(31536000,0));
		//TP firstTP;
		//bool gotFirstTP(false);
		if (useGt)
		{
			// first pass, read tf
			rosbag::Bag bag(bagFile->filename[0]);
			
			rosbag::View view(bag, rosbag::TopicQuery("/tf"));
			BOOST_FOREACH(rosbag::MessageInstance const m, view)
			{
				// TODO: filter
				typedef geometry_msgs::TransformStamped TS;
				tf::tfMessage::ConstPtr tfMsg = m.instantiate<tf::tfMessage>();
				for (tf::tfMessage::_transforms_type::const_iterator it = tfMsg->transforms.begin(); it != tfMsg->transforms.end(); ++it)
				{
					const TS& ts(*it);
					cout << ts.header.stamp << ":" << ts.header.frame_id << " <- " << ts.child_frame_id << endl;
					tfBuffer.setTransform(ts, "default");
				}
			}
			bag.close();
		}
		
		// create labels
		DP::Labels labels;
		labels.push_back(DP::Label("x", 1));
		labels.push_back(DP::Label("y", 1));
		labels.push_back(DP::Label("z", 1));
		labels.push_back(DP::Label("pad", 1));
		// TODO: add if use point cloud
		{
			// second pass, read points
			rosbag::Bag bag(argv[1]);
			
			rosbag::View view(bag, rosbag::TopicQuery(cloudTopic->sval[0]));
			BOOST_FOREACH(rosbag::MessageInstance const m, view)
			{
				sensor_msgs::PointCloud2::ConstPtr cloudMsg = m.instantiate<sensor_msgs::PointCloud2>();
				
				string err;
				TP tp = TP::Identity(4,4);
				if (useGt)
				{
					cout << "adding cloud with transform from " << gtSourceFrame->sval[0] << " to " << gtTargetFrame->sval[0] << " at time: " <<  cloudMsg->header.stamp <<  "..." << endl;
					if (tfBuffer.canTransform(gtTargetFrame->sval[0], gtSourceFrame->sval[0], cloudMsg->header.stamp, &err))
					{
						const geometry_msgs::TransformStamped ts = tfBuffer.lookupTransform(gtTargetFrame->sval[0], gtSourceFrame->sval[0], cloudMsg->header.stamp);
						tp = msgTransformToTP(ts.transform);
						cout << "success" << endl;
					}
					else
					{
						cout << "error" << endl;
						continue;
					}
				}
				
				Datum datum;
				datum.transform = tp;
				datum.stamp = cloudMsg->header.stamp;
				
				// create data points
				pcl::PointCloud<pcl::PointXYZ> cloud;
				pcl::fromROSMsg(*cloudMsg, cloud);
				
				// point count
				const size_t pointCount(cloud.points.size());
				DP::Features tempCloud(4, pointCount);
				int dIndex(0);
				for (size_t i = 0; i < pointCount; ++i)
				{
					if (!isnan(cloud.points[i].x) && 
						!isnan(cloud.points[i].y) &&
						!isnan(cloud.points[i].z)
					)
					{
						tempCloud(0, dIndex) = cloud.points[i].x;
						tempCloud(1, dIndex) = cloud.points[i].y;
						tempCloud(2, dIndex) = cloud.points[i].z;
						tempCloud(3, dIndex) = 1;
						++dIndex;
					}
					else
					{
						//cout << i << " : " << cloud.points[i].x <<", " << cloud.points[i].y << ", " << cloud.points[i].z << endl;
						//cout << "nan at pos " << j << "\n";
					}
				}
				cout << dIndex << " good points on " << pointCount << "\n\n";
				datum.cloud = DP(tempCloud.leftCols(dIndex), labels);
				data.push_back(datum);
			}
			bag.close();
		}
	}
	catch (rosbag::BagException e)
	{
		cerr << "Error, error reading bag file: " << e.what() << endl;
		return 2;
	}
	
  if(data.empty())
  {
    cerr << "No cloud loaded" << endl;
    return 2;
  }

	// load param list
	ifstream ifs(paramsFile->filename[0]);
	if (!ifs.good())
	{
		cerr << "Error, invalid params file " << paramsFile->filename[0] << endl;
		return 3;
	}
	
	// open stat and tf output files, if requested
	ofstream statofs, tfofs, dtfofs;
	openWriteFile(statofs, statFile, 4);
	openWriteFile(tfofs, tfFile, 5);
	openWriteFile(dtfofs, dtfFile, 6);
	
	int deltaTfSteps(30);
	if (useGt && dtfSteps->count > 0)
	{
		deltaTfSteps = dtfSteps->ival[0];
		cout << "Using tf steps of " << deltaTfSteps << endl;
	}
	
	cout << endl;
	
	// for each line in the experiment file
	unsigned expCount(0);
	while (ifs.good())
	{
		// read line and load params
		reloadParamsFromLine(ifs);
		if (params.empty())
			break;
		cout << "Exp " << expCount << ", loaded " << params.size() << " parameters\n\n";
		++expCount;
		
		// run experiment
		MSA::ICPSequence icp(3, "", false);
		populateParameters(icp);
		Histogram<Scalar> e_x(16, "e_x", "", false), e_y(16, "e_y", "", false), e_z(16, "e_z", "", false), e_a(16, "e_a", "", false);
		Histogram<Scalar> e_acc_x(16, "e_acc_x", "", false), e_acc_y(16, "e_acc_y", "", false), e_acc_z(16, "e_acc_z", "", false), e_acc_a(16, "e_acc_a", "", false);
		
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
		catch (MSA::ConvergenceError error)
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
			catch (MSA::ConvergenceError error)
			{
				++failCount;
				cerr << "ICP failed to converge at cloud " << i+1 << " : " << error.what() << endl;
			}
			
			/*
			//DEBUG: remove after
			cout << "Model id: " << i-1 << " with data id: " << i << endl;
			if(i==1)
				abort();
			*/
			
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
			
			// write tf output
			if (tfofs.good())
			{
				tfofs << data[i].stamp << " ";
				
				const Vector3 t_icp(T_icp.topRightCorner(3,1));
				const Quaternion<Scalar> q_icp(Matrix3(T_icp.topLeftCorner(3,3)));
				tfofs << t_icp(0) << " " << t_icp(1) << " " << t_icp(2) << " " << q_icp.x() << " " << q_icp.y() << " " << q_icp.z() << " " << q_icp.w() << " ";
				
				if (useGt)
				{
					const Vector3 t_gt(T_gt.topRightCorner(3,1));
					const Quaternion<Scalar> q_gt(Matrix3(T_gt.topLeftCorner(3,3)));
					tfofs << t_gt(0) << " " << t_gt(1) << " " << t_gt(2) << " " << q_gt.x() << " " << q_gt.y() << " " << q_gt.z() << " " << q_gt.w() << "\n";
				}
			}
			
			// compute errors
			const TP T_d = T_d_gt * T_d_icp.inverse();
			if (useGt)
			{
				const Vector3 e_t(T_d.topRightCorner(3,1));
				e_x.push_back(e_t(0));
				e_y.push_back(e_t(1));
				e_z.push_back(e_t(2));
				const Quaternion<Scalar> quat(Matrix3(T_d.topLeftCorner(3,3)));
				e_a.push_back(2 * acos(quat.normalized().w()));
			}
			
			if (i % deltaTfSteps == 0)
			{
				if (useGt)
				{
					// compute difference
					const TP T_d_acc = T_d_gt_acc * T_d_icp_acc.inverse();
					
					// compute errors
					const Vector3 e_t(T_d_acc.topRightCorner(3,1));
					e_acc_x.push_back(e_t(0));
					e_acc_y.push_back(e_t(1));
					e_acc_z.push_back(e_t(2));
					const Quaternion<Scalar> quat(Matrix3(T_d_acc.topLeftCorner(3,3)));
					e_acc_a.push_back(2 * acos(quat.normalized().w()));
				}
				
				// dump deltas
				if (dtfofs.good())
				{
					tfofs << data[i].stamp << " ";
					
					const Vector3 t_icp(T_d_icp_acc.topRightCorner(3,1));
					const Quaternion<Scalar> q_icp(Matrix3(T_d_icp_acc.topLeftCorner(3,3)));
					dtfofs << t_icp(0) << " " << t_icp(1) << " " << t_icp(2) << " " << q_icp.x() << " " << q_icp.y() << " " << q_icp.z() << " " << q_icp.w() << " ";
					
					if (useGt)
					{
						const Vector3 t_gt(T_d_gt_acc.topRightCorner(3,1));
						const Quaternion<Scalar> q_gt(Matrix3(T_d_gt_acc.topLeftCorner(3,3)));
						dtfofs << t_gt(0) << " " << t_gt(1) << " " << t_gt(2) << " " << q_gt.x() << " " << q_gt.y() << " " << q_gt.z() << " " << q_gt.w() << "\n";
					}
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
		if (statofs.good())
		{
			statofs << data.size() - 1 << " " << failCount << " * ";
			if (useGt)
			{
				// error on each dt
				e_x.dumpStats(statofs); statofs << " * ";
				e_y.dumpStats(statofs); statofs << " * ";
				e_z.dumpStats(statofs); statofs << " * ";
				e_a.dumpStats(statofs); statofs << " * ";
				// error every sect
				e_acc_x.dumpStats(statofs); statofs << " * ";
				e_acc_y.dumpStats(statofs); statofs << " * ";
				e_acc_z.dumpStats(statofs); statofs << " * ";
				e_acc_a.dumpStats(statofs); statofs << " * ";
			}
			// timing
			statofs << icpTotalDuration << " * ";
			icp.keyFrameDuration.dumpStats(statofs); statofs << " * ";
			icp.convergenceDuration.dumpStats(statofs); statofs << " * ";
			// algo stats
			icp.iterationsCount.dumpStats(statofs); statofs << " * ";
			icp.pointCountIn.dumpStats(statofs); statofs << " * ";
			icp.pointCountReading.dumpStats(statofs); statofs << " * ";
			icp.pointCountKeyFrame.dumpStats(statofs); statofs << " * ";
			icp.pointCountTouched.dumpStats(statofs); statofs << " * ";
			icp.overlapRatio.dumpStats(statofs); statofs << " # ";
			// params for info
			for (Params::const_iterator it(params.begin()); it != params.end(); ++it)
				statofs << it->first << "=" << it->second << " ";
			// finish results
			statofs << "\n";
		}
	}

	return 0;
}
