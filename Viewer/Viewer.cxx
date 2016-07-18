/**
Viewer derived from VTK examples
**/

#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>
#include <vtkIterativeClosestPointTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkLandmarkTransform.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <vtkCellArray.h>
#include <vtkIdList.h>
#include <vtkKdTreePointLocator.h>
#include <vtkPointSource.h>

#include <opencv\cv.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/flann.hpp>
#include <opencv2\flann\matrix.h>

#include <stdio.h>
#include <fstream>
#include <vector>
#include <math.h>
#include <sstream>

using namespace std;
using namespace cv;
namespace
{
	void PerturbPolyData(vtkSmartPointer<vtkPolyData> polydata);
	void TranslatePolyData(vtkSmartPointer<vtkPolyData> polydata);
	void FindPoints(vtkSmartPointer<vtkPolyData> source, vtkSmartPointer<vtkPolyData> target);
}

struct SearchParams {
	SearchParams(int checks = 32);
};

int main(int argc, char* argv[])
{
	/*
	Grabbed points from .ply file (bunnies, teapots etc) for use as the ply i/o/ and parsing in these libraries don't work with point cloud only files.
	Looked at some ply parsers, but... 
	*/
	if (argc != 3) {
		std::cout << "Usage: " << argv[0]
			<< "Two files required -  Filename1(.xyz) & Filename2(.xyz)" << std::endl;

		return EXIT_FAILURE;
	}
	
	Mat dataset;
	Mat query;

	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();

	vtkSmartPointer<vtkRenderer> backgroundRenderer =
		vtkSmartPointer<vtkRenderer>::New();
	backgroundRenderer->SetLayer(0);
	renderWindow->AddRenderer(backgroundRenderer);

	vtkSmartPointer<vtkPolyData> source =
		vtkSmartPointer<vtkPolyData>::New();

	vtkSmartPointer<vtkPolyData> target =
		vtkSmartPointer<vtkPolyData>::New();
	

	double xmins[3] = { 0,0,.3 };
	double xmaxs[3] = { 0,0.7,1 };
	double ymins[3] = { 0,0,0, };
	double ymaxs[3] = { 0,1,1 };

	double r[4] = { 0.7,0.5,0.3,0 };
	double g[4] = { 0.1,0.2,0.3,0.4 };
	double b[4] = { 0.1,0.2,0.4,0.4 };

	for (unsigned i = 1; i < argc; i++){
		std::string filename = argv[i];
		std::ifstream filestream(filename.c_str());

		std::string line;
		vtkSmartPointer<vtkPoints> points =
			vtkSmartPointer<vtkPoints>::New();

		while (std::getline(filestream, line))
		{
			double x, y, z;
			std::stringstream linestream;
			linestream << line;
			linestream >> x >> y >> z;

			points->InsertNextPoint(x, y, z);
		}

		filestream.close();

		vtkSmartPointer<vtkPolyData> polyData =
			vtkSmartPointer<vtkPolyData>::New();

		polyData->SetPoints(points);
		
		if (i == 1) {
			source->SetPoints(points);
			
		}

		if (i == 2) {
			//TranslatePolyData(polyData);
			PerturbPolyData(polyData);
			target->SetPoints(points);
			
		}

		vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
			vtkSmartPointer<vtkVertexGlyphFilter>::New();

		glyphFilter->SetInputData(polyData);
		glyphFilter->Update();

		vtkSmartPointer<vtkPolyDataMapper> mapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputConnection(glyphFilter->GetOutputPort());

		vtkSmartPointer<vtkActor> actor =
			vtkSmartPointer<vtkActor>::New();
		actor->GetProperty()->SetColor(r[i], g[i], b[i]);
		actor->GetProperty()->SetPointSize(2);
		actor->SetMapper(mapper);		

		vtkSmartPointer<vtkRenderer> renderer =
			vtkSmartPointer<vtkRenderer>::New();
		
		renderer->AddActor(actor);
		renderer->SetViewport(xmins[i], ymins[i], xmaxs[i], ymaxs[i]);
		renderer->SetLayer(i);
		renderWindow->AddRenderer(renderer);
		
	}
	
	/*
	ICP block
		FindPoints(source, target);

		take returned corresponding points
		Find centroid
		Claculate matrix
		Perform transform
		Render result.....
	
		

	vtkSmartPointer<vtkPolyDataMapper> resultMapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	resultMapper->SetInputConnection(source->GetOutputPort());

	vtkSmartPointer<vtkActor> resultActor =
		vtkSmartPointer<vtkActor>::New();
	resultActor->SetMapper(resultMapper);
	resultActor->GetProperty()->SetColor(0, 0, 1);
	resultActor->GetProperty()->SetPointSize(3);

	vtkSmartPointer<vtkRenderer> resultRenderer =
		vtkSmartPointer<vtkRenderer>::New();
	resultRenderer->SetLayer(argc);
	renderWindow->AddRenderer(resultRenderer);
	resultRenderer->AddActor(resultActor);*/

	renderWindow->SetSize(800, 600);
	renderWindow->SetNumberOfLayers(argc);

	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);
	renderWindow->SetWindowName("Tea pots");

	renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}

namespace
{
	//Ads noise for testing
	void PerturbPolyData(vtkSmartPointer<vtkPolyData> polydata){
		vtkSmartPointer<vtkPoints> points =
			vtkSmartPointer<vtkPoints>::New();
		points->ShallowCopy(polydata->GetPoints());

		for (vtkIdType i = 0; i < points->GetNumberOfPoints(); i++)
		{
			double p[3];
			points->GetPoint(i, p);
			double perturb[3];
			if (i % 3 == 0)
			{
				perturb[0] = .1; perturb[1] = 0; perturb[2] = 0;
			}
			else if (i % 3 == 1)
			{
				perturb[0] = 0; perturb[1] = .1; perturb[2] = 0;
			}
			else
			{
				perturb[0] = 0; perturb[1] = 0; perturb[2] = .1;
			}

			for (unsigned int j = 0; j < 3; j++)
			{
				p[j] += perturb[j];
			}
			points->SetPoint(i, p);
		}

		polydata->SetPoints(points);

	}

	//Transforms an output model so that we can see a result
	void TranslatePolyData(vtkSmartPointer<vtkPolyData> polydata) {
		vtkSmartPointer<vtkTransform> transform =
			vtkSmartPointer<vtkTransform>::New();
		transform->Translate(0, .3, 0);

		vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter =
			vtkSmartPointer<vtkTransformPolyDataFilter>::New();
		transformFilter->SetInputData(polydata);
		transformFilter->SetTransform(transform);
		transformFilter->Update();

		polydata->ShallowCopy(transformFilter->GetOutput());
	}

	void FindPoints(vtkSmartPointer<vtkPolyData> source, 
				vtkSmartPointer<vtkPolyData> target) {


		vtkSmartPointer<vtkKdTreePointLocator> pointTree =
			vtkSmartPointer<vtkKdTreePointLocator>::New();
		pointTree->SetDataSet(source);
		pointTree->BuildLocator();

		vtkIdType k = target->GetNumberOfPoints();
		double testPoint[3] = { 0.0, 0.0, 0.0 }; //Actually iterate through source points and return each row as tests
		vtkSmartPointer<vtkIdList> result =
			vtkSmartPointer<vtkIdList>::New();

		pointTree->FindPointsWithinRadius(1.0, testPoint, result);

		for (vtkIdType i = 0; i < k; i++)
		{
			vtkIdType point_ind = result->GetId(i);
			double p[3];
			source->GetPoint(point_ind, p);
			std::cout << "Closest point " << i << ": Point "
				<< point_ind << ": (" << p[0] << ", " << p[1] << ", " << p[2] << ")" << std::endl;
		}

	}

	float flann_knn(Mat& m_destinations, Mat& m_object, vector<int>& ptpairs, vector<float>& dists = vector<float>()) {
		// find nearest neighbors using FLANN
		cv::Mat m_indices(m_object.rows, 1, CV_32S);
		cv::Mat m_dists(m_object.rows, 1, CV_32F);

		Mat dest_32f; m_destinations.convertTo(dest_32f, CV_32FC2);
		Mat obj_32f; m_object.convertTo(obj_32f, CV_32FC2);
		assert(dest_32f.type() == CV_32F);
		cv::flann::Index flann_index(dest_32f, cv::flann::KDTreeIndexParams(2));  // using 4 randomized kdtrees
		flann_index.knnSearch(obj_32f, m_indices, m_dists, 1, cv::flann::SearchParams(64)); // maximum number of leafs checked

		int* indices_ptr = m_indices.ptr<int>(0);
		//float* dists_ptr = m_dists.ptr<float>(0);
		for (int i = 0; i<m_indices.rows; ++i) {
			ptpairs.push_back(indices_ptr[i]);
		}

		dists.resize(m_dists.rows);
		m_dists.copyTo(Mat(dists));

		return cv::sum(m_dists)[0];
	}
}
