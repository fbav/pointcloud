#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_traits.h>
#include <pcl/point_types.h>



class FindCorrespondence {
	Eigen::Matrix3f returnCovariance;
	public:
		typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
		typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

		Eigen::Matrix3f returnMatrix(){
			return returnCovariance;
		}

		void search(PointCloud::Ptr inputCloud, PointCloud::Ptr templateCloud, float xPoint, float yPoint, float zPoint, int iterations) {
			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
			kdtree.setInputCloud(inputCloud);
			pcl::PointXYZ searchPoint;
			std::cout << "point cloud size -- " << templateCloud->size() << std::endl;
			searchPoint.x = xPoint;
			searchPoint.y = yPoint;
			searchPoint.z = zPoint;

			pcl::PointCloud<pcl::PointXYZ>::Ptr tempcloud1(new pcl::PointCloud<pcl::PointXYZ>());
			pcl::PointCloud<pcl::PointXYZ>::Ptr tempcloud2(new pcl::PointCloud<pcl::PointXYZ>());

			tempcloud1 = templateCloud;

			for (int j = 0; j < iterations; j++) {
				

				int K = 3;

				std::vector<int> pointIdxNKNSearch(K);
				std::vector<float> pointNKNSquaredDistance(K);

				std::map< size_t, size_t > corresp;

				std::cout << "K nearest neighbor search at (" << searchPoint.x
					<< " " << searchPoint.y
					<< " " << searchPoint.z
					<< ") with K=" << K << std::endl;

				auto minDist = std::numeric_limits<float>::max();
				Eigen::Vector4f pointCentroid;
				if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
				{
					for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i){
						std::cout << "    " << inputCloud->points[pointIdxNKNSearch[i]].x
							<< " " << inputCloud->points[pointIdxNKNSearch[i]].y
							<< " " << inputCloud->points[pointIdxNKNSearch[i]].z
							<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
						searchPoint.x = inputCloud->points[pointIdxNKNSearch[i]].x;
						searchPoint.y = inputCloud->points[pointIdxNKNSearch[i]].y;
						searchPoint.z = inputCloud->points[pointIdxNKNSearch[i]].z;

						
					}
				}
				
				Eigen::Vector4f inputCloudCentroid = findCentroid(tempcloud1);
				Eigen::Matrix3f covariance_matrix;
				pcl::computeCovarianceMatrix(*tempcloud1, inputCloudCentroid, covariance_matrix);
				returnCovariance = covariance_matrix;

				Eigen::Matrix3f rotation_m = returnMatrix().block<3, 3>(0, 0);
				Eigen::Vector3f translation_v = returnMatrix().block<3, 1>(0, 3);
				Eigen::Affine3f transformObj = Eigen::Affine3f::Identity();

				transformObj.translation() << translation_v(0), translation_v(1), translation_v(2);
				transformObj.rotate(Eigen::AngleAxisf(rotation_m(1), Eigen::Vector3f::UnitY()));
				pcl::transformPointCloud(*tempcloud1, *tempcloud2, transformObj);

				tempcloud1 = tempcloud2;
				
			}

			//Also looked at  Neighbors within radius search

			/*std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;

			float radius = 256.0f * rand() / (RAND_MAX + 1.0f);

			std::cout << "Neighbors within radius search at (" << searchPoint.x
				<< " " << searchPoint.y
				<< " " << searchPoint.z
				<< ") with radius=" << radius << std::endl;


			if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
			{
				for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
					std::cout << "    " << inputCloud->points[pointIdxRadiusSearch[i]].x
					<< " " << inputCloud->points[pointIdxRadiusSearch[i]].y
					<< " " << inputCloud->points[pointIdxRadiusSearch[i]].z
					<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
			}*/

		}

		Eigen::Vector4f findCentroid(PointCloud::Ptr cloud) {
			Eigen::Vector4f centroid;

			pcl::compute3DCentroid(*cloud, centroid);

			std::cout << "The XYZ coordinates of the centroid are: ("
				<< centroid[0] << ", "
				<< centroid[1] << ", "
				<< centroid[2] << ")." << std::endl;

			return centroid;
		}

		~FindCorrespondence(){}
};

class FeatureCloud
{
  public:
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

    FeatureCloud () :
      search_method_xyz_ (new SearchMethod),
      normal_radius_ (0.02f),
      feature_radius_ (0.02f)
    {}

    ~FeatureCloud () {}

    // Process the given cloud
    void
    setInputCloud (PointCloud::Ptr xyz)
    {
      xyz_ = xyz;
      processInput ();
    }

    // Load and process the cloud in the given PCD file
    void
    loadInputCloud (const std::string &plyfile)
    {
      xyz_ = PointCloud::Ptr (new PointCloud);
     // pcl::io::loadPCDFile (pcd_file, *xyz_);

	  pcl::PolygonMesh::Ptr mesh_(new pcl::PolygonMesh());
	  pcl::io::loadPLYFile(plyfile, *mesh_);
	  //pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_(new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::fromPCLPointCloud2(mesh_->cloud, *xyz_);

      //processInput ();
    }

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud () const
    {
      return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
      return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
      return (features_);
    }

  protected:
    // Compute the surface normals and local features
    void
    processInput ()
    {
      computeSurfaceNormals ();
      computeLocalFeatures ();
    }

    // Compute the surface normals
    void
    computeSurfaceNormals ()
    {
      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
      norm_est.setInputCloud (xyz_);
      norm_est.setSearchMethod (search_method_xyz_);
      norm_est.setRadiusSearch (normal_radius_);
      norm_est.compute (*normals_);
    }

    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);

      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (xyz_);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }

  private:
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

class TemplateAlignment
{
  public:

    // A struct for storing alignment results
    struct Result
    {
      float fitness_score;
      Eigen::Matrix4f final_transformation;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignment () :
      min_sample_distance_ (0.05f),
      max_correspondence_distance_ (0.01f*0.01f),
      nr_iterations_ (500)
    {
      // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
      sac_ia_.setMinSampleDistance (min_sample_distance_);
      sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
      sac_ia_.setMaximumIterations (nr_iterations_);
    }

    ~TemplateAlignment () {}

    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud (FeatureCloud &target_cloud)
    {
      target_ = target_cloud;
      sac_ia_.setInputTarget (target_cloud.getPointCloud ());
      sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
    }

    // Add the given cloud to the list of template clouds
    void
    addTemplateCloud (FeatureCloud &template_cloud)
    {
      templates_.push_back (template_cloud);
    }

    // Align the given template cloud to the target specified by setTargetCloud ()
    void
    align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
    {
      sac_ia_.setInputCloud (template_cloud.getPointCloud ());
      sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

      pcl::PointCloud<pcl::PointXYZ> registration_output;
      sac_ia_.align (registration_output);

      result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
      result.final_transformation = sac_ia_.getFinalTransformation ();
    }

    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void
    alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
    {
      results.resize (templates_.size ());
      for (size_t i = 0; i < templates_.size (); ++i)
      {
        align (templates_[i], results[i]);
      }
    }

    // Align all of template clouds to the target cloud to find the one with best alignment score
    int
    findBestAlignment (TemplateAlignment::Result &result)
    {
      // Align all of the templates to the target cloud
      std::vector<Result, Eigen::aligned_allocator<Result> > results;
      alignAll (results);

      // Find the template with the best (lowest) fitness score
      float lowest_score = std::numeric_limits<float>::infinity ();
      int best_template = 0;
      for (size_t i = 0; i < results.size (); ++i)
      {
        const Result &r = results[i];
        if (r.fitness_score < lowest_score)
        {
          lowest_score = r.fitness_score;
          best_template = (int) i;
        }
      }

      // Output the best alignment
      result = results[best_template];
      return (best_template);
    }

  private:
    // A list of template clouds and the target to which they will be aligned
    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
};

// Align a collection of object templates to a sample point cloud
int
main (int argc, char **argv)
{
  if (argc < 3)
  {
    printf ("No target files given!\n");
    return (-1);
  }

  //Load objects specified in the object.txt file
  std::vector<FeatureCloud> object_templates;
  std::ifstream input_stream (argv[1]);
  object_templates.resize (0);
  std::string objFilename;
  while (input_stream.good ())
  {
    std::getline (input_stream, objFilename);
    if (objFilename.empty () || objFilename.at (0) == '#') 
      continue;

    FeatureCloud template_cloud;
    template_cloud.loadInputCloud (objFilename);
    object_templates.push_back (template_cloud);
  }
  input_stream.close ();

  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh());
  pcl::io::loadPLYFile(argv[2], *mesh);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);;
  pcl::fromPCLPointCloud2(mesh->cloud, *cloud);

  //Removing distant points...
  const float depth_limit = 1.0;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, depth_limit);
  pass.filter (*cloud);

  //Downsample
  const float voxel_grid_size = 0.005f;
  pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
  vox_grid.setInputCloud (cloud);
  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>); 
  vox_grid.filter (*tempCloud);
  cloud = tempCloud; 

  // Assign to the target FeatureCloud
  FeatureCloud target_cloud;
  target_cloud.setInputCloud (cloud);

  // Set the TemplateAlignment inputs
  TemplateAlignment template_align;
  for (size_t i = 0; i < object_templates.size (); ++i)
  {
    template_align.addTemplateCloud (object_templates[i]);
  }
  template_align.setTargetCloud (target_cloud);

  // Find the best template alignment
  TemplateAlignment::Result best_alignment;
  int best_index = template_align.findBestAlignment (best_alignment);
  const FeatureCloud &best_template = object_templates[best_index];


  pcl::PolygonMesh::Ptr mesh1(new pcl::PolygonMesh());
  pcl::io::loadPLYFile("../data/bun000.ply", *mesh1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr clouda(new pcl::PointCloud<pcl::PointXYZ>);;
  pcl::fromPCLPointCloud2(mesh1->cloud, *clouda);

  pcl::PolygonMesh::Ptr mesh2(new pcl::PolygonMesh());
  pcl::io::loadPLYFile("../data/bun180.ply", *mesh2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudb(new pcl::PointCloud<pcl::PointXYZ>);;
  pcl::fromPCLPointCloud2(mesh2->cloud, *cloudb);


  // Save the aligned template for visualization
/*  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  pcl::transformPointCloud (*best_template.getPointCloud (), transformed_cloud, best_alignment.final_transformation);
  pcl::io::savePCDFileBinary ("../data/dataoutput.pcd", transformed_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr personloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile("../data/bun01.pcd", *personloud);*/


  pcl::PointCloud<pcl::PointXYZ>::Ptr coutputloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile("../data/dataoutput.pcd", *coutputloud);

  FindCorrespondence findCorrespondence;
  findCorrespondence.search(cloudb, cloud, 0.0f, 0.0f, 0.0f, 50);
  Eigen::Matrix3f rotation_m = findCorrespondence.returnMatrix().block<3, 3>(0, 0);
  Eigen::Vector3f translation_v = findCorrespondence.returnMatrix().block<3, 1>(0, 3);

  Eigen::Affine3f transformObj = Eigen::Affine3f::Identity();

  printf("Rotation\n");
  printf("    | %6.3f %6.3f %6.3f | \n", rotation_m(0, 0), rotation_m(0, 1), rotation_m(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n", rotation_m(1, 0), rotation_m(1, 1), rotation_m(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", rotation_m(2, 0), rotation_m(2, 1), rotation_m(2, 2));
  printf("Translation\n");
  printf("t = < %0.3f, %0.3f, %0.3f >\n", translation_v(0), translation_v(1), translation_v(2));

  transformObj.translation() << translation_v(0), translation_v(1), translation_v(2);
 // transformObj.rotate(Eigen::AngleAxisf(rotation_m(0), Eigen::Vector3f::UnitX()));
  transformObj.rotate(Eigen::AngleAxisf(rotation_m(1), Eigen::Vector3f::UnitY()));
  // transformObj.rotate(Eigen::AngleAxisf(rotation_m(2), Eigen::Vector3f::UnitY()));
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*cloudb, *transformed_cloud, transformObj);

  //Create a viewer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(clouda, "person cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "person cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(transformed_cloud, 200, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(transformed_cloud, single_color, "output cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output cloud");

  //onfigure the camera
  std::vector<pcl::visualization::Camera> cam;
  viewer->initCameraParameters();
  viewer->getCameras(cam);
  cam[0].pos[2] = 5.0;
  viewer->resetCamera();

  while (!viewer->wasStopped())
  {
	  viewer->spinOnce(100);
	  boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }

  return (0);
}