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
    
};

using PointCloud3D = Object3D<vtkPolyData>;
using BBox3D = Object3D<vtkOutlineFilter>;


 
struct PclRendererImpl : public PclRendererInterface {
    
public:
PclRendererImpl(): m_ptsCloud(vtkSmartPointer<vtkPoints>::New()),
               m_renderer(vtkSmartPointer<vtkRenderer>::New()),
               m_renderWindow(vtkSmartPointer<vtkRenderWindow>::New()),
               m_pointCloud(new PointCloud3D),
               m_verticesCloud(vtkSmartPointer<vtkCellArray>::New()){
                   ;
               }
    bool Creategeometry(const pcl::PointCloud<pcl::PointXYZ>& _pcdData);
    bool InitRenderer();
    virtual void Render()override;
    virtual void RenderOffScreen(const std::string& _imgName="/run/user/1000/")override;
   
    virtual bool LoadPcdFile(const std::string&)override;
    virtual std::string GetVersion()const override{return "1.0";}
    virtual void MoveCamera(double _shiftX,double _shiftY,double _Z)override;
    virtual void RPYCamera(double _R,double _P,double _Y)override;
    virtual void ZoomCamera(double _scale);
    
private:
    bool _CompressToJpeg(const std::string& _imgName,const vtkUnsignedCharArray* _data)const;
    
private:
    vtkSmartPointer<vtkPoints> m_ptsCloud;
    vtkSmartPointer<vtkCellArray> m_verticesCloud;

  
   std::unique_ptr<PointCloud3D> m_pointCloud;
    std::unique_ptr<BBox3D> m_bbox;
    //vtkSmartPointer<vtkPolyData> m_pointPolyData ;
    
     //vtkSmartPointer<vtkOutlineFilter> m_bbox;
     //vtkSmartPointer<vtkPolyDataMapper> m_bboxMapper ;

    vtkSmartPointer<vtkRenderer> m_renderer;
    vtkSmartPointer<vtkRenderWindow> m_renderWindow;
    

     int m_width=640;
     int m_height=480;
     int curr=1;
};

