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



struct PclRendererImpl : public PclRendererInterface {
    
public:
PclRendererImpl(): m_ptsCloud(vtkSmartPointer<vtkPoints>::New()),
               m_renderer(vtkSmartPointer<vtkRenderer>::New()),
               m_renderWindow(vtkSmartPointer<vtkRenderWindow>::New()),
               m_pointPolyDataMapper(vtkSmartPointer<vtkPolyDataMapper>::New()),
               m_pointPolyData(vtkSmartPointer<vtkPolyData>::New()),
               m_actor(vtkSmartPointer<vtkActor>::New()),
               m_verticesCloud(vtkSmartPointer<vtkCellArray>::New()){
                   ;
               }
    bool Creategeometry(const pcl::PointCloud<pcl::PointXYZ>& _pcdData);
    bool InitRenderer();
    virtual void Render()override;
    virtual void RenderOffScreen(const std::string& _imgName="/run/user/1000/image.png")override;
    void RenderOffScreenInMemory();
    virtual bool LoadPcdFile(const std::string&)override;
    virtual std::string GetVersion()const override{return "1.0";}
    virtual void MoveCamera(double _shiftX,double _shiftY)override;
    
private:
    vtkSmartPointer<vtkPoints> m_ptsCloud;
    vtkSmartPointer<vtkCellArray> m_verticesCloud;

    vtkSmartPointer<vtkPolyDataMapper> m_pointPolyDataMapper;

    vtkSmartPointer<vtkPolyData> m_pointPolyData ;

    vtkSmartPointer<vtkRenderer> m_renderer;
    vtkSmartPointer<vtkRenderWindow> m_renderWindow;
    
    vtkSmartPointer<vtkActor> m_actor;
    


     int m_width=640;
     int m_height=480;
     int curr=1;
};

