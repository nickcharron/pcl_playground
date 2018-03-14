#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <vector>
#include <math.h>
#include <iomanip>

//using pcl::visualization::PointCloudColorHandlerGenericField;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// All the objects needed
pcl::PassThrough<PointT> pass;
pcl::NormalEstimation<PointT, pcl::Normal> ne;
pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
pcl::PCDWriter writer;
pcl::ExtractIndices<PointT> extract;
pcl::ExtractIndices<pcl::Normal> extract_normals;
pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

// Datasets
pcl::PointCloud<PointT>::Ptr cloud_input (new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_remainder (new pcl::PointCloud<pcl::Normal>);
pcl::ModelCoefficients::Ptr coefficients_cylinder1 (new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers_cylinder1 (new pcl::PointIndices);
pcl::ModelCoefficients::Ptr coefficients_cylinder2 (new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers_cylinder2 (new pcl::PointIndices);


  int main (int argc, char** argv)
  {
      // Read point cloud
        pcl::PCLPointCloud2 cloud_intermediate;
        pcl::io::loadPCDFile ("/home/nick/projects/pcl_playground/models/LidarsModel4.pcd", cloud_intermediate);
        pcl::fromPCLPointCloud2 (cloud_intermediate, *cloud_input);
        std::cout << "Model PointCloud has: " << cloud_input->points.size () << " data points." << std::endl;

      // Build a passthrough filter to remove spurious NaNs
        pass.setInputCloud (cloud_input);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0, 1.5);
        pass.filter (*cloud_filtered);
        std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

      // Estimate point normals
        ne.setSearchMethod (tree);
        ne.setInputCloud (cloud_filtered);
        ne.setKSearch (50);
        ne.compute (*cloud_normals);


      // Create the segmentation object for cylinder segmentation and set all the parameters
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_CYLINDER);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight (0.1);
        seg.setMaxIterations (10000);
        seg.setDistanceThreshold (0.05);
        seg.setRadiusLimits (0, 0.1);

      // Segment the first cylinder

        // Obtain the cylinder1 inliers and coefficients
          seg.setInputCloud (cloud_filtered);
          seg.setInputNormals (cloud_normals);
          seg.segment (*inliers_cylinder1, *coefficients_cylinder1);
          std::cout << "Cylinder 1 coefficients: " << *coefficients_cylinder1 << std::endl;
          extract.setInputCloud (cloud_filtered);
          extract.setIndices (inliers_cylinder1);
          extract.setNegative (false);
          pcl::PointCloud<PointT>::Ptr cloud_cylinder1 (new pcl::PointCloud<PointT> ());
          extract.filter (*cloud_cylinder1);

        // now let's repeat with the leftover points

          // extract remainder of points
            extract.setInputCloud (cloud_filtered);
            extract.setIndices (inliers_cylinder1);
            extract.setNegative (true);
            pcl::PointCloud<PointT>::Ptr cloud_remainder (new pcl::PointCloud<PointT> ());
            extract.filter (*cloud_remainder);

          // Recalculate normals
            ne.setInputCloud (cloud_remainder);
            ne.compute (*cloud_normals_remainder);

          // Obtain the cylinder2 inliers and coefficients
            seg.setInputCloud (cloud_remainder);
            seg.setInputNormals (cloud_normals_remainder);
            seg.segment (*inliers_cylinder2, *coefficients_cylinder2);
            std::cout << "Cylinder 2 coefficients: " << *coefficients_cylinder2 << std::endl;
            extract.setInputCloud (cloud_remainder);
            extract.setIndices (inliers_cylinder2);
            extract.setNegative (false);
            pcl::PointCloud<PointT>::Ptr cloud_cylinder2 (new pcl::PointCloud<PointT> ());
            extract.filter (*cloud_cylinder2);

        // Visualize input cloud
          pcl::visualization::PCLVisualizer *p;
          int vp1, vp2;
          p = new pcl::visualization::PCLVisualizer (argc, argv, "Lidar to Lidar Calibration - Visualization");
          p->createViewPort (0.0, 0, 0.5, 1.0, vp1);
          p->addCoordinateSystem (0.5);
          p->setBackgroundColor (255, 255, 255);
          pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_input1_h(cloud_input, 255, 0, 0);
          p->addPointCloud (cloud_input, cloud_input1_h , "vp1_input", vp1);

        // Plot extracted cylinders
          p->createViewPort (0.5, 0, 1.0, 1.0, vp2);
          p->setBackgroundColor (255, 255, 255);
          p->addCoordinateSystem (0.5);
          pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_cylinder1_h(cloud_cylinder1, 0, 255, 0);
          pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_cylinder2_h(cloud_cylinder2, 0, 0, 255);
          p->addPointCloud (cloud_cylinder1, cloud_cylinder1_h , "vp2_cylinder1", vp2);
          p->addPointCloud (cloud_cylinder2, cloud_cylinder2_h , "vp2_cylinder2", vp2);
          //p->spin();

        /* TURNS OUT THAT THE AXES VECTOR ARE ALREADY NORMALIZED
        // store normalized axis vectors from cylinders
          std::vector<double> axis_norm_1 (3,0), axis_norm_2 (3,0);

          std::cout << "calculating magnitude of axes..." << std::endl;

          double mag1 = sqrt(pow(coefficients_cylinder1->values[3],2)+
                              pow(coefficients_cylinder1->values[4],2) +
                              pow(coefficients_cylinder1->values[5],2));
          double mag2 = sqrt(pow(coefficients_cylinder2->values[3],2)+
                              pow(coefficients_cylinder2->values[4],2)+
                              pow(coefficients_cylinder2->values[5],2));

          std::cout << "calculating norm of axis vectors..." << std::endl;
          std::cout << "------------------------" << std::endl;
          std::cout << "Normalized Axis Vectors:" << std::endl;
          std::cout << "------------------------" << std::endl;
          std::cout << " Cyl. No.1  |  Cyl. No.2" << std::endl;
          std::cout << "------------------------ " << std::endl;
          for (int i=0; i<3; i++)
          {
              axis_norm_1[i] = coefficients_cylinder1->values[i+3]/mag1;
              std::cout << std::fixed;
              std::cout << std::setprecision(6) << axis_norm_1[i] << "   |   ";
              axis_norm_2[i] = coefficients_cylinder2->values[i+3]/mag2;
              std::cout << std::setprecision(6) << axis_norm_2[i] << std::endl;
          }
          std::cout << std::endl;
          std::cout << "calculating the the projection of the points onto the axes norm... " << std::endl;
        */

        // calculate the center of the axes
          double proj_i, proj1_sum = 0, proj1_avg, proj2_sum = 0, proj2_avg;

          for (size_t i = 0; i < cloud_cylinder1->points.size (); ++i)
          {
            proj_i = cloud_cylinder1->points[i].x * coefficients_cylinder1->values[3]
                    + cloud_cylinder1->points[i].y * coefficients_cylinder1->values[4]
                    + cloud_cylinder1->points[i].z * coefficients_cylinder1->values[5];
            proj1_sum=+proj_i;
          }

          for (size_t i = 0; i < cloud_cylinder2->points.size (); ++i)
          {
            proj_i = cloud_cylinder2->points[i].x * coefficients_cylinder2->values[3]
                    + cloud_cylinder2->points[i].y * coefficients_cylinder2->values[4]
                    + cloud_cylinder2->points[i].z * coefficients_cylinder2->values[5];
            proj2_sum=+proj_i;
          }

          proj1_avg = proj1_sum/cloud_cylinder1->points.size ();
          proj2_avg = proj2_sum/cloud_cylinder2->points.size ();

          std::cout << "calculating the cente of the axes..." << std::endl;
          std::vector<double> axis_center_1 (3,0), axis_center_2 (3,0);

          for (int i=0; i<3; i++)
          {
            axis_center_1[i] = coefficients_cylinder1->values[i+3] * proj1_avg;
            axis_center_2[i] = coefficients_cylinder2->values[i+3] * proj2_avg;
          }

          std::cout << "creating plot arrows..." << std::endl;

          // Create an end point for the arrow to plot
            double arrow_length = 0.3;
            std::vector<double> arrow_end_1(3,0), arrow_end_2(3,0);
            for (int i=0; i<3; i++)
            {
              arrow_end_1[i] = axis_center_1[i] + arrow_length;
              arrow_end_2[i] = axis_center_2[i] + arrow_length;
            }

            std::cout << "storing arrow end points..." << std::endl;

            // add to point object
             PointT arrow_start1, arrow_end1,arrow_start2, arrow_end2;

             arrow_start1.x = axis_center_1[0];
             arrow_start1.y = axis_center_1[1];
             arrow_start1.z = axis_center_1[2];
             arrow_end1.x = arrow_end_1[0];
             arrow_end1.y = arrow_end_1[1];
             arrow_end1.z = arrow_end_1[2];

             arrow_start2.x = axis_center_2[0];
             arrow_start2.y = axis_center_2[1];
             arrow_start2.z = axis_center_2[2];
             arrow_end2.x = arrow_end_2[0];
             arrow_end2.y = arrow_end_2[1];
             arrow_end2.z = arrow_end_2[2];

             std::cout << "plotting arrows..." << std::endl;

          // plot arrows
            p->addArrow(arrow_start1, arrow_end1, 0, 255, 0, false, "arrow1", vp2);
            p->addArrow(arrow_start2, arrow_end2, 0, 0, 255, false, "arrow2", vp2);
            p->spin();

  }
