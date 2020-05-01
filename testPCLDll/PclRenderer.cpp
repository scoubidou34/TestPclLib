
#include "export.h"
#include "PclRendererImpl.h"

#include <cstdlib>
#include <iostream>

 

PclRendererInterface* Init()
{
    return new PclRendererImpl();
    //return nullptr;
}

void Destroy(PclRendererInterface* _obj){
    delete _obj;
}

bool LoadPcdFile(PclRendererInterface* _obj, const std::string& _s){
    return _obj->LoadPcdFile(_s);  
}

bool LoadPcdFile2(PclRendererInterface* _obj){
    return _obj->LoadPcdFile("rops_cloud.pcd");  
}

char* GetVersion(PclRendererInterface* _obj){
    const std::string& val(_obj->GetVersion());
    char* res=new char[val.length()+1];
     strcpy(res, val.c_str()); 
    return res;
}

void Render(PclRendererInterface* _obj){
    _obj->Render();
}

void RenderOffScreen(PclRendererInterface* _obj){
    _obj->RenderOffScreen();
    
}

void MoveCamera(PclRendererInterface* _obj,double _x,double _y,double _z){
    _obj->MoveCamera(_x,_y,_z);
}
 
 void RPYCamera(PclRendererInterface* _obj,double _R,double _P,double _Y){
    _obj->RPYCamera(_R,_P,_Y);
}
 
 void ZoomCamera(PclRendererInterface* _obj,double _scale){
     _obj->ZoomCamera(_scale);
 }