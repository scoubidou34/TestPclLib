/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   main.cpp
 * Author: tom
 *
 * Created on April 6, 2020, 12:15 PM
 */

#include <cstdlib>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/point_cloud_image_extractors.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkPNGWriter.h>
#include <vtkPoints.h>
#include <vtkRenderLargeImage.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
using namespace std;

/*
 *
 */
int main(int argc, char** argv)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudData(new pcl::PointCloud<pcl::PointXYZ>);

    if(pcl::io::loadPCDFile<pcl::PointXYZ>("rops_cloud.pcd", *cloudData) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file bunny.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << cloudData->width * cloudData->height
              << " data points from test_pcd.pcd with the following fields: " << std::endl;

    vtkSmartPointer<vtkPoints> ptsCloud = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> verticesCloud = vtkSmartPointer<vtkCellArray>::New();
    //ptsCloud->SetNumberOfPoints(cloudData->points.size());
    for(std::size_t i = 0; i < cloudData->points.size(); ++i) {
        // ptsCloud->SetPoint(i, );
        vtkIdType id(ptsCloud->InsertNextPoint ( cloudData->points[i].x, cloudData->points[i].y, cloudData->points[i].z )); 
       // vtkIdType id(ptsCloud->InsertNextPoint ( i,i,i ));       
        verticesCloud->InsertNextCell(1,&id);
    }
    // std::cout << "    " << cloudData->points[i].x << " " << cloudData->points[i].y << " " << cloudData->points[i].z
    //         << std::endl;

    //... populate cloud
    /*pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
      viewer.showCloud (cloudData);
    while (!viewer.wasStopped ())
      {

      }*/
    std::cout << vtkVersion::GetVTKSourceVersion() << std::endl;
    std::cout << vtkVersion::GetVTKMajorVersion() << std::endl;
    std::cout << vtkVersion::GetVTKMinorVersion() << std::endl;
    /*  vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
      sphere->SetThetaResolution(10);
      sphere->SetPhiResolution(5);
  */
    vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    // sphereMapper->SetInputConnection(sphere->GetOutputPort());
    vtkSmartPointer<vtkPolyData> pointPolyData = vtkSmartPointer<vtkPolyData>::New();
    pointPolyData->SetPoints(ptsCloud);
    pointPolyData->SetVerts(verticesCloud);

    sphereMapper->SetInputData(pointPolyData);

    vtkSmartPointer<vtkActor> sphere1 = vtkSmartPointer<vtkActor>::New();
    sphere1->SetMapper(sphereMapper);
    sphere1->GetProperty()->SetColor(1, 0, 0);
    sphere1->GetProperty()->SetAmbient(0.3);
    sphere1->GetProperty()->SetDiffuse(0.0);
    sphere1->GetProperty()->SetSpecular(1.0);
    sphere1->GetProperty()->SetSpecularPower(5.0);
    sphere1->GetProperty()->SetPointSize(0.1);
    

    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();

    renderer->AddActor(sphere1);

    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();

    renderWindow->AddRenderer(renderer);
    // renderWindow->SetOffScreenRendering( 1 );

    // Let the renderer compute good position and focal point
    /*renderer->GetActiveCamera()->Azimuth(30);
    renderer->GetActiveCamera()->Elevation(30);*/
    renderer->ResetCamera();
    //renderer->GetActiveCamera()->Dolly(1.4);
    renderer->ResetCameraClippingRange();
    renderer->SetBackground(.3, .4, .5);

    renderWindow->SetSize(640, 480);

    vtkSmartPointer<vtkRenderLargeImage> renderLarge = vtkSmartPointer<vtkRenderLargeImage>::New();
    renderLarge->SetInput(renderer);

    vtkSmartPointer<vtkPNGWriter> pImageWriter = vtkSmartPointer<vtkPNGWriter>::New();
    //  pImageWriter->SetInput(pWindowImageFilter->GetOutput());

    /* pImageWriter->SetFileName("coons_image.png");
     pImageWriter->SetInputConnection(renderLarge->GetOutputPort());
     pImageWriter->Write();*/
    // pcl::PCLImage image;
    // Create PointCloudImageExtractor subclass that can handle "label" field
    //  pcl::io::PointCloudImageExtractor<pcl::XYZ> pcie;
    // Set it up if not happy with the defaults
    //    pcie.setColorMode(pcie.COLORS_RGB_RANDOM);
    // Try to extract an image
    //  bool success = pcie.extract(cloudData, image);
    // Save to file if succeeded
    // if (success)
    // pcl::io::savePNGFile("filename.png", image);
    vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor->SetRenderWindow(renderWindow);
    interactor->Initialize();
    interactor->Start();
    return (0);
}
