#include "PclRendererImpl.h"

#include <cstdlib>
#include <iostream>

#include <pcl/io/point_cloud_image_extractors.h>

#include <pcl/visualization/cloud_viewer.h>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkPNGWriter.h>
#include <vtkWindowToImageFilter.h> 
#include <vtkRenderLargeImage.h>

#include <vtkRenderWindowInteractor.h>


 
  

bool PclRendererImpl::LoadPcdFile( const std::string& _file)
{
    pcl::PointCloud<pcl::PointXYZ> cloudData;

    if(pcl::io::loadPCDFile<pcl::PointXYZ>(_file, cloudData) == -1) //* load the file
    {
        PCL_ERROR("oucou" /*std::string("Couldn't read file ")+_file+std::string(" \n")*/);
        return false;
    }
    std::cout << "Loaded " << cloudData.width * cloudData.height
              << " data points from test_pcd.pcd with the following fields: " << std::endl;
    if (Creategeometry(cloudData))
        return InitRenderer();
    return false;
}

 

bool PclRendererImpl::Creategeometry(const pcl::PointCloud<pcl::PointXYZ>& _cloudData)
{
    // ptsCloud->SetNumberOfPoints(cloudData->points.size());
    for(std::size_t i = 0; i < _cloudData.points.size(); ++i) {
        // ptsCloud->SetPoint(i, );
        vtkIdType id(
            m_ptsCloud->InsertNextPoint(_cloudData.points[i].x, _cloudData.points[i].y, _cloudData.points[i].z));
        // vtkIdType id(ptsCloud->InsertNextPoint ( i,i,i ));
        m_verticesCloud->InsertNextCell(1, &id);
    }
    return true;
}

bool PclRendererImpl::InitRenderer()
{
    // sphereMapper->SetInputConnection(sphere->GetOutputPort());
    m_pointPolyData->SetPoints(m_ptsCloud);
    m_pointPolyData->SetVerts(m_verticesCloud);

    m_pointPolyDataMapper->SetInputData(m_pointPolyData);

    m_actor->SetMapper(m_pointPolyDataMapper);
    m_actor->GetProperty()->SetColor(1, 0, 0);
    m_actor->GetProperty()->SetAmbient(0.3);
    m_actor->GetProperty()->SetDiffuse(0.0);
    m_actor->GetProperty()->SetSpecular(1.0);
    m_actor->GetProperty()->SetSpecularPower(5.0);
    m_actor->GetProperty()->SetPointSize(0.1);

    m_renderer->AddActor(m_actor);
    m_renderWindow->AddRenderer(m_renderer);
    // renderWindow->SetOffScreenRendering( 1 );

    // Let the renderer compute good position and focal point
    /*renderer->GetActiveCamera()->Azimuth(30);
    renderer->GetActiveCamera()->Elevation(30);*/
    m_renderer->ResetCamera();
    // renderer->GetActiveCamera()->Dolly(1.4);
    m_renderer->ResetCameraClippingRange();
    m_renderer->SetBackground(.3, .4, .5);
}

void PclRendererImpl::Render()
{
    m_renderWindow->SetSize(m_width, m_height);
    m_renderWindow->SetOffScreenRendering(0);
    vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor->SetRenderWindow(m_renderWindow);
    interactor->Initialize();
    interactor->Start();
}

void PclRendererImpl::RenderOffScreen()
{
    m_renderWindow->SetSize(m_width, m_height);
    m_renderWindow->SetOffScreenRendering(1);
   /* vtkSmartPointer<vtkRenderLargeImage> renderLarge = vtkSmartPointer<vtkRenderLargeImage>::New();
    renderLarge->SetInput(m_renderer);
     * */
    vtkSmartPointer<vtkWindowToImageFilter> renderLarge = vtkSmartPointer<vtkWindowToImageFilter>::New();
    renderLarge->SetInput(m_renderWindow);
    
    vtkSmartPointer<vtkPNGWriter> pImageWriter = vtkSmartPointer<vtkPNGWriter>::New();
    //pImageWriter->SetInput(pWindowImageFilter->GetOutput());
    pImageWriter->SetFileName("image.png");
    pImageWriter->SetInputConnection(renderLarge->GetOutputPort());
    pImageWriter->Write();
}