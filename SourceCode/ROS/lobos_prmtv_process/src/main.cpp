
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <lobos_cloud_pubsub/CloudPublisher.hpp>
#include <lobos_cloud_pubsub/CloudSubscriber.hpp>
#include <string>
#include <iostream>

#include "prmtvProcess.hpp"
#include "MyGraph.hpp"

int main (int argc, char **argv) {

  // ROS initialization
  ros::init (argc, argv, "lobos_prmtv_process");
  ros::NodeHandle nh;
  CloudPublisher<pcl::PointXYZRGB>  my_cloudPublisher(nh, std::string("prmtv_processed"));
  CloudSubscriber<pcl::PointXYZRGB> my_cloudSubscriber(nh, std::string("planes_segmented"));

  ros::AsyncSpinner spinner(1); 
  spinner.start();
  ros::Rate r(1);
  while (ros::ok()) {

    if (my_cloudSubscriber.getIsThereNewData()) {
      pcl::PointCloud<PointXYZRGB> ri = my_cloudSubscriber.getCurrentPointclout();
      std::vector<PointCloud<PointXYZRGB> > segCloudVec;

      segCloudVec = getColorSegment(ri);
      cout<<"Number of segment:"<<segCloudVec.size()<<endl;

      pcl::PointCloud<PointXYZ> tmpCloud;

      vector<PointCloud<PointXYZ> > horPlanes;
      vector<PointCloud<PointXYZ> > verPlanes;
      vector<PointCloud<PointXYZ> > finPlanes;

      for (size_t j = 0; j < segCloudVec.size(); ++j)
	{
	  PointCloud<PointXYZ> plane;
	  copyPointCloud(segCloudVec[j],plane);

	  	  if(plane.points.size()<ri.size()/10){
	  //	  if(plane.points.size()<10000){	   
 if(getDirection(plane))
	      horPlanes.push_back(plane);
	    else
	      verPlanes.push_back(plane);
	    finPlanes.push_back(plane);
	    }

	}

      ////////////////////////////////////////

      PointCloud<PointXYZRGB> verSeg;
      PointCloud<PointXYZRGB> horSeg;
      PointCloud<PointXYZRGB> finSeg;

      // temporary recognition part
      createPtCloud(horPlanes, horSeg);
      createPtCloud(verPlanes, verSeg);
      //      createPtCloud(finPlanes, finSeg);

      vector<PointCloud<PointXYZRGB>::Ptr > *verVector = new vector<PointCloud<PointXYZRGB>::Ptr >();
      vector<PointCloud<PointXYZRGB>::Ptr > *horVector = new vector<PointCloud<PointXYZRGB>::Ptr >();

      PointCloud<PointXYZRGB>::Ptr verCloud(new PointCloud<PointXYZRGB>(verSeg));
      PointCloud<PointXYZRGB>::Ptr horCloud(new PointCloud<PointXYZRGB>(horSeg));

      getObjClusters(verCloud, *verVector);
      getObjClusters(horCloud, *horVector);

      cout<<"Planes"<<horVector->size()<<endl;
	
      vector<PointCloud<PointXYZ> > verSup;	
      vector<PointCloud<PointXYZ> > horSup;	
      PointCloud<PointXYZ> tmpPtCl;

      for(vector<PointCloud<PointXYZRGB>::Ptr >::iterator it = verVector->begin(); it!= verVector->end();++it){
	copyPointCloud(**it, tmpPtCl);
	verSup.push_back(tmpPtCl);
      }

      for(vector<PointCloud<PointXYZRGB>::Ptr >::iterator it = horVector->begin(); it!= horVector->end();++it){
	copyPointCloud(**it, tmpPtCl);
	horSup.push_back(tmpPtCl);
      }

      createPtCloud(verSup, verSeg);
      createPtCloud(horSup, horSeg);

      // MATCHING
      MyGraph<string> g(horVector->size()+verVector->size());

      int count = 0;
      float tmpDist = 0;

      for(size_t i = 0; i < verSup.size(); i++){	  
	g.setVertexData(count,string("vertical"));
	PointCloud<PointXYZ>::Ptr tmpCloud1(new PointCloud<PointXYZ>(verSup[i]));	

	for(size_t j = 0; j < horSup.size(); j++){
	  PointCloud<PointXYZ>::Ptr tmpCloud2(new PointCloud<PointXYZ>(horSup[j]));
	  tmpDist = getMinDist(tmpCloud1,tmpCloud2);

	  if(tmpDist<0.1 && (tmpCloud1->size() < tmpCloud2->size()))
	    g.addEdge(count,verSup.size()+j);	    
	}
	count++;
      }
	
      for(size_t i = 0; i<horSup.size(); i++){	  	  
	g.setVertexData(count,string("horizontal"));
	count++;
      }
	
      g.printGraph();

      //Our pattern
      MyGraph<string> ourPattern(4);

      ourPattern.addEdge(0,1);
      ourPattern.addEdge(1,2);
      ourPattern.addEdge(3,1);
	
      ourPattern.setVertexData(0,string("vertical"));
      ourPattern.setVertexData(1,string("horizontal"));
      ourPattern.setVertexData(2,string("vertical"));
      ourPattern.setVertexData(3,string("vertical"));
      ourPattern.printGraph();

      vector<int> resultIdxList;
      bool resultOut;
      	
      resultOut = g.containsSubgraph(ourPattern, resultIdxList);
	
      vector<PointCloud<PointXYZ> > resultDisplay;	

      if (resultOut) {
	cout << "===========================" << endl << "MATCH FOUND" <<endl<< "===========================" << endl;
	for(size_t i = 0; i<resultIdxList.size(); i++){
	  cout<<resultIdxList[i]<<endl;
	  if(resultIdxList[i]<verSup.size()){
	    cout<<"here1"<<endl;
	    resultDisplay.push_back(verSup[resultIdxList[i]]);
	  }
	  else{
	    cout<<"here2"<<endl;
	    resultDisplay.push_back(horSup[resultIdxList[i]-verSup.size()]);
	  }
	}

      } else {
	cout << "Not found" << endl;
      }
      createPtCloud(resultDisplay, finSeg);
      my_cloudPublisher.publishPointcloud(finSeg);

    } else {
      std::cout << "Waiting for data to be published" << std::endl;
    }

    r.sleep();
  }

  ros::waitForShutdown();
  return 0;
}

