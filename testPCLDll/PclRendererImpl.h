#pragma once

#include "export.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkOutlineFilter.h>
#include <vtkDataSetMapper.h>

class  Meshed3D{
   public: 
     Meshed3D():    m_actor(vtkSmartPointer<vtkActor>::New()),                            
                                  m_polyDataMapper(vtkSmartPointer<vtkDataSetMapper>::New()){
                                m_actor->SetMapper(m_polyDataMapper);
                                  }
 vtkSmartPointer<vtkActor> m_actor;
 vtkSmartPointer<vtkDataSetMapper> m_polyDataMapper;
    
    void Print(){
        m_actor->Print(std::cout);
    }
};

template <class T>  class Object3D{
public:
    Object3D():    m_actor(vtkSmartPointer<vtkActor>::New()),
                                  m_data(vtkSmartPointer<T>::New()),
                                  m_polyDataMapper(vtkSmartPointer<vtkPolyDataMapper>::New()){
                                m_actor->SetMapper(m_polyDataMapper);
                                  }
 vtkSmartPointer<vtkActor> m_actor;
 vtkSmartPointer<vtkPolyDataMapper> m_polyDataMapper;
 vtkSmartPointer<T> m_data ;
    
    void Print(){
        m_actor->Print(std::cout);
        m_data->Print(std::cout);
    }
};

using PointCloud3D = Object3D<vtkPolyData>;
using BBox3D = Object3D<vtkOutlineFilter>;


 
struct PclRendererInterface  {
    
public:
PclRendererInterface(): m_ptsCloud(vtkSmartPointer<vtkPoints>::New()),
               m_renderer(vtkSmartPointer<vtkRenderer>::New()),
               m_renderWindow(vtkSmartPointer<vtkRenderWindow>::New()),
                m_pointPolyData(vtkSmartPointer<vtkPolyData>::New()),
                m_verticesCloud(vtkSmartPointer<vtkCellArray>::New())
                {
                    m_renderWindow->AddRenderer(m_renderer);
               }
    bool CreateGeometry(const pcl::PointCloud<pcl::PointXYZ>& _pcdData);
    bool CreatePointsCloud();
    bool CreateMeshing();
    virtual void Render();
    virtual void RenderOffScreen(const std::string& _imgName="/run/user/1000/");
   
    virtual bool LoadPcdFile(const std::string&);
    virtual std::string GetVersion()const {return "1.0";}
    virtual void MoveCamera(double _shiftX,double _shiftY,double _Z);
    virtual void RPYCamera(double _R,double _P,double _Y);
    virtual void ZoomCamera(double _scale);
    virtual void ResetCamera();
    
private:
    bool _CompressToJpeg(const std::string& _imgName,const vtkUnsignedCharArray* _data)const;
    
    void   _DestroyPointsCloud();
    void _DestroyMeshing();
    vtkActor* _GetCurrentActor()const;


private:
    vtkSmartPointer<vtkPoints> m_ptsCloud;
    vtkSmartPointer<vtkCellArray> m_verticesCloud;
    vtkSmartPointer<vtkPolyData> m_pointPolyData;
  
   std::unique_ptr<PointCloud3D> m_pointCloud;
    std::unique_ptr<BBox3D> m_bbox;
    
    std::unique_ptr<Meshed3D> m_mesh;
    


    vtkSmartPointer<vtkRenderer> m_renderer;
    vtkSmartPointer<vtkRenderWindow> m_renderWindow;
    

     int m_width=640;
     int m_height=480;

     
     const std::string sDefaultPath="/home/tom/Documents/TomCodeliteWs/pclData/";
};

