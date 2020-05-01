#include "PclRendererImpl.h"

#include <cstdlib>
#include <iostream>

#include <pcl/io/point_cloud_image_extractors.h>

#include <pcl/visualization/cloud_viewer.h>
#include <vtkActor.h>
#include <vtkBMPWriter.h>
#include <vtkCamera.h>
#include <vtkJPEGWriter.h>
#include <vtkPNGWriter.h>
#include <vtkRenderLargeImage.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkTransform.h>
#include <vtkWindowToImageFilter.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <turbojpeg.h>


std::string Display(const double* _vec){
    std::string out;
    for(auto i=0;i<3;++i)out+=std::to_string(_vec[i]);
    return out;
}

bool PclRendererImpl::LoadPcdFile(const std::string& _file)
{
    pcl::PointCloud<pcl::PointXYZ> cloudData;

    if(pcl::io::loadPCDFile<pcl::PointXYZ>(_file, cloudData) == -1) //* load the file
    {
        PCL_ERROR("oucou" /*std::string("Couldn't read file ")+_file+std::string(" \n")*/);
        return false;
    }
    std::cout << "Loaded " << cloudData.width * cloudData.height
              << " data points from test_pcd.pcd with the following fields: " << std::endl;
    if(Creategeometry(cloudData))
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
    m_pointCloud->m_data->SetPoints(m_ptsCloud);
    m_pointCloud->m_data->SetVerts(m_verticesCloud);

    m_pointCloud->m_polyDataMapper->SetInputData(m_pointCloud->m_data);

    {
        auto actor(m_pointCloud->m_actor);
        actor->GetProperty()->SetColor(1, 0, 0);
        actor->GetProperty()->SetAmbient(0.3);
        actor->GetProperty()->SetDiffuse(0.0);
        actor->GetProperty()->SetSpecular(1.0);
        actor->GetProperty()->SetSpecularPower(5.0);
        actor->GetProperty()->SetPointSize(0.1);

        m_renderer->AddActor(actor);
    }

    // show bbox
    {
        m_bbox.reset(new BBox3D());
        m_bbox->m_data->SetInputData(m_pointCloud->m_data);
        m_bbox->m_polyDataMapper->SetInputConnection(m_bbox->m_data->GetOutputPort());
        auto actor(m_bbox->m_actor);
        actor->GetProperty()->SetColor(0, 0, 0);
        m_renderer->AddActor(actor);
    }

    m_renderWindow->AddRenderer(m_renderer);
    // renderWindow->SetOffScreenRendering( 1 );

    // Let the renderer compute good position and focal point
    /*renderer->GetActiveCamera()->Azimuth(30);
    renderer->GetActiveCamera()->Elevation(30);*/
    //  m_renderer->ResetCamera();
    // renderer->GetActiveCamera()->Dolly(1.4);
    //  m_renderer->ResetCameraClippingRange();
    m_renderer->SetBackground(.3, .4, .5);
    return true;
}

void PclRendererImpl::Render()
{
    m_renderWindow->SetSize(m_width, m_height);
    m_renderWindow->SetOffScreenRendering(0);
    vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor->SetRenderWindow(m_renderWindow);
    interactor->Initialize();
    /*
     for(auto i=0;i<100;i++)
     {
         std::cout<<i<<std::endl;
         interactor->SetScale(11);
     }*/
    interactor->Start();
}

bool PclRendererImpl::_CompressToJpeg(const std::string& _imgPath)const{
    
    const int JPEG_QUALITY = 85;
    const int COLOR_COMPONENTS = 3;
    long unsigned int _jpegSize = 0;
    unsigned char* _compressedImage = nullptr;
    unsigned char* buffer = nullptr;
 
    {
        ifstream myfile;
        myfile.open(_imgPath+"image.bmp", ios::binary);
        if(!myfile.is_open()) {
            std::cout << "Error reading";
            return false;
        }
        myfile.seekg(0, ios::end);
        const int sizeF(myfile.tellg());
        buffer = new unsigned char[sizeF];
        myfile.read(reinterpret_cast<char*>(buffer), sizeF);
        myfile.close();
    }

    tjhandle _jpegCompressor = tjInitCompress();

    tjCompress2(_jpegCompressor, buffer, m_width, 0, m_height, TJPF_RGB, &_compressedImage, &_jpegSize, TJSAMP_444,
        JPEG_QUALITY, TJFLAG_FASTDCT | TJFLAG_BOTTOMUP);

    delete[] buffer;
    
    {
        ofstream myfile;
        myfile.open(_imgPath+"image.jpeg", ios::out);
        if (!myfile.is_open()){
            std::cout<<"Error writing";
            return false;
        }
        myfile.write(reinterpret_cast<char*>(_compressedImage), _jpegSize);
        myfile.close();
    }
    
    tjDestroy(_jpegCompressor);
    tjFree(_compressedImage);
    
    /*       auto strCmd(std::string("cjpeg -quality 85 -outfile /run/user/1000/image.jpeg  /run/user/1000/image.bmp"));
       if (!std::system(strCmd.c_str()))
           cerr<<"error cjpeg"<<std::endl;
       return;*/
       return true;
}

void PclRendererImpl::RenderOffScreen(const std::string& _imgName)
{

    // RenderOffScreenInMemory();
    // return;
    m_renderWindow->SetSize(m_width, m_height);
    m_renderWindow->SetOffScreenRendering(1);
    /* vtkSmartPointer<vtkRenderLargeImage> renderLarge = vtkSmartPointer<vtkRenderLargeImage>::New();
     renderLarge->SetInput(m_renderer);
      * */
    vtkSmartPointer<vtkWindowToImageFilter> renderLarge = vtkSmartPointer<vtkWindowToImageFilter>::New();
    renderLarge->SetInput(m_renderWindow);
    {
        vtkSmartPointer<vtkBMPWriter> pImageWriter = vtkSmartPointer<vtkBMPWriter>::New();
        // pImageWriter->SetInput(pWindowImageFilter->GetOutput());
        // pImageWriter->SetWriteToMemory(1);
        pImageWriter->SetFileName((_imgName+std::string("image.bmp")).c_str());
        //  pImageWriter->SetQuality	(50)	;

        pImageWriter->SetInputConnection(renderLarge->GetOutputPort());
        pImageWriter->Write();
    }
    _CompressToJpeg(_imgName);
}

void PclRendererImpl::RenderOffScreenInMemory()
{
    m_renderWindow->SetSize(m_width, m_height);
    m_renderWindow->SetOffScreenRendering(1);

    vtkSmartPointer<vtkWindowToImageFilter> renderLarge = vtkSmartPointer<vtkWindowToImageFilter>::New();
    renderLarge->SetInput(m_renderWindow);

    vtkSmartPointer<vtkJPEGWriter> pImageWriter = vtkSmartPointer<vtkJPEGWriter>::New();
    // pImageWriter->SetInput(pWindowImageFilter->GetOutput());
    pImageWriter->SetInputConnection(renderLarge->GetOutputPort());
    // pImageWriter->SetWriteToMemory(1);
    pImageWriter->SetFileName("/home/tom/toto1.png");
    pImageWriter->Write();
    // auto res(pImageWriter->GetResult());
    // std::cout<<res->GetNumberOfTuples	(	)<<std::endl;
}

void PclRendererImpl::RPYCamera(double _R, double _P, double _Y)
{
    auto start = std::chrono::steady_clock::now();
    vtkCamera* ptrCamera = m_renderer->GetActiveCamera();

    assert(ptrCamera);
 
    std::cout << "RPY:" << _R << "   " << _P << "  " << _Y << std::endl;
    ptrCamera->Roll(_R);
    ptrCamera->Yaw(_Y);
    ptrCamera->Pitch(_P);
    RenderOffScreen();

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
}


void PclRendererImpl::ZoomCamera(double _scale)
{
        auto start = std::chrono::steady_clock::now();
    vtkCamera* ptrCamera = m_renderer->GetActiveCamera();
    auto mat(ptrCamera->GetViewTransformMatrix());
    mat->Print(std::cout);

    auto center(m_pointCloud->m_actor->GetCenter());
    ptrCamera->SetFocalPoint(center);
    
    std::cout << "Center" << center[0] << " " << center[1] << " " << center[2] << std::endl;
    std::cout << "ViewUp" << Display(m_renderer->GetActiveCamera()->GetViewUp()) << std::endl;
    std::cout << "Focal" << Display(m_renderer->GetActiveCamera()->GetFocalPoint()) << std::endl;

   std::cout << "Camera position" <<Display(ptrCamera->GetPosition()) << std::endl;
   
    double distance(ptrCamera->GetViewAngle());
   
    distance+=_scale;
      std::cout << "DIST:" << distance<< " " << std::endl;
    ptrCamera->SetViewAngle(distance);
    m_renderer->ResetCameraClippingRange();
    
    RenderOffScreen();

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
}

void PclRendererImpl::MoveCamera(double _shiftX, double _shiftY, double _shiftZ)
{
    auto start = std::chrono::steady_clock::now();
    vtkCamera* ptrCamera = m_renderer->GetActiveCamera();
    auto mat(ptrCamera->GetViewTransformMatrix());
    mat->Print(std::cout);

    auto center(m_pointCloud->m_actor->GetCenter());
    ptrCamera->SetFocalPoint(center);
  
    std::cout << "ViewUp" << Display(m_renderer->GetActiveCamera()->GetViewUp()) << std::endl;
    std::cout << "Focal" << Display(m_renderer->GetActiveCamera()->GetFocalPoint()) << std::endl;
   std::cout << "Camera position" <<Display(ptrCamera->GetPosition()) << std::endl;

    double* pVec(ptrCamera->GetPosition());
    std::cout << "SHIFT:" << _shiftZ << " " << std::endl
              << "MOVE_BEFORE:" << pVec[0] << "   " << pVec[1] << " " << pVec[2] << std::endl;
   
    auto centerScene(m_pointCloud->m_actor->GetCenter());
    vtkSmartPointer<vtkTransform> rot = vtkSmartPointer<vtkTransform>::New();
    rot->Identity();
    rot->Translate(centerScene);
    rot->RotateY(_shiftX);
    rot->RotateX(_shiftY);
    for(auto i = 0; i < 3; ++i)
        centerScene[i] *= -1;
    rot->Translate(centerScene);
    double pVecOut[3] = { 0, 0, 0 };
    rot->TransformPoint(pVec, pVecOut);
    std::cout << "MOVE_AFTER:" << pVecOut[0] << "   " << pVecOut[1] << " " << pVecOut[2] << std::endl;
    ptrCamera->SetPosition(pVecOut);
    m_renderer->ResetCameraClippingRange();
    // ptrCamera->Update();
    RenderOffScreen();

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
}