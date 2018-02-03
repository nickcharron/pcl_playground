#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

//using pcl::visualization::PointCloudColorHandlerGenericField;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// Global point cloud containers:
  PointCloud::Ptr  cloud_out (new PointCloud);
  PointCloud::Ptr  cloud_prev (new PointCloud);
  PointCloud::Ptr  cloud_new (new PointCloud); // Cannot define these in for loop
  PointCloud::Ptr  cloud_new_initialized (new PointCloud);
  PointCloud::Ptr  cloud_new_transformed (new PointCloud);
  PointCloud::Ptr  cloud_result (new PointCloud);

// Global matrix containers
  Eigen::Matrix4f globalTransform = Eigen::Matrix4f::Identity ();

  int main (int argc, char** argv)
  {
      // Initialize first cloud
      pcl::PCLPointCloud2 cloud_intermediate;
      pcl::io::loadPCDFile ("/home/nick/bag_files/calibration/extracted_PCDs/L2L_calibration_02-02-2018-20_10_26/horzPCD1.pcd", cloud_intermediate);
      pcl::fromPCLPointCloud2 (cloud_intermediate, *cloud_out);
      *cloud_new = *cloud_out;

      Eigen::Matrix4f newInitToOut;
      Eigen::Matrix4f globalTransform = Eigen::Matrix4f::Identity ();


      // Initialize visualizer
      pcl::visualization::PCLVisualizer *p;
      int vp1, vp2;
      p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration Testing");
      p->createViewPort (0.0, 0, 0.5, 1.0, vp1);
      p->createViewPort (0.5, 0, 1.0, 1.0, vp2);
      p->addCoordinateSystem (1.0);
                                                                                //R,  G,  B
      pcl::visualization::PointCloudColorHandlerCustom<PointT> prev_h(cloud_prev, 255, 0, 0);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> new_h (cloud_new, 0, 255, 0);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> new_transformed_h (cloud_new_transformed, 0, 0, 255);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> out_h (cloud_out, 255, 255, 255);

      // Iterate over all point clouds - hard code
      for (int i=2; i < 100; i++)
      {
          // Set prev cloud to new cloud before assigning another new cloud
          *cloud_prev = *cloud_new;

          // read and convert point cloud from pcd file
          pcl::PCLPointCloud2 cloud_intermediate;
          char pcdFileName[100];
          //sprintf(pcdFileName, "../pcds/bag3/pcd%d.pcd", i);
          sprintf(pcdFileName, "/home/nick/bag_files/calibration/extracted_PCDs/L2L_calibration_02-02-2018-20_10_26/horzPCD%d.pcd", i);
          PCL_INFO ("Read PCD File: %s\n", pcdFileName);
          pcl::io::loadPCDFile (pcdFileName, cloud_intermediate);
          pcl::fromPCLPointCloud2 (cloud_intermediate, *cloud_new); //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>

          /*
          // Create initial transformation estimate
          Eigen::Affine3f transform_estimate = Eigen::Affine3f::Identity();
          float incre_rot = 5*M_PI/180; //5 degree rotation
          transform_estimate.translation() << 0, 0, 0;  // no translation
          transform_estimate.rotate (Eigen::AngleAxisf (incre_rot, Eigen::Vector3f::UnitY())); //about y axis (pitch)
          // ***** try changing the rotation vector to somewhere near the center
          // of the sensor mount *******

          // align new Scan
          */

          // perform icp

            pcl::IterativeClosestPoint<PointT, PointT> icp;

            // we want to register the new cloud against the combined cloud that
            // we are building. Registering against prev cloud causes drift.
            // First we want to initialize which we can do with the last scan's
            // pose:
            pcl::transformPointCloud (*cloud_new, *cloud_new_initialized, globalTransform);
            icp.setInputSource(cloud_new_initialized);
            icp.setInputTarget(cloud_out);
            icp.setMaximumIterations(20);
            //icp.setInputTarget(cloud_prev);
            icp.align(*cloud_result);

          // Get transforms
          newInitToOut = icp.getFinalTransformation ();

          // Calculate transform between new point cloud and original
          globalTransform = globalTransform * newInitToOut;

          // Transform new cloud back in first cloud frame (original cloud)
          pcl::transformPointCloud (*cloud_new, *cloud_new_transformed, globalTransform);

          // Now that the new scan is in the frame of the first scan, add to map
          *cloud_out += *cloud_new_transformed;

          // visualize
          p->removePointCloud ("vp_new");
          p->removePointCloud ("vp_prev");
          p->removePointCloud ("vp_out");
          p->removePointCloud ("vp_new_transformed");

          p->addPointCloud (cloud_new, new_h , "vp_new", vp1);
          p->addPointCloud (cloud_prev, prev_h, "vp_prev", vp1);
          p->addPointCloud (cloud_new_transformed, new_transformed_h, "vp_new_transformed", vp1);

          p->addPointCloud (cloud_out, out_h, "vp_out", vp2);

          PCL_INFO ("Press q to continue the registration.\n");
          p-> spin();

      }

  }
