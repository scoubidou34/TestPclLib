#pragma once

#include <iostream>
#include <memory>
#include <string>

extern "C" {

struct PclRendererInterface {
public:
    virtual void Render() = 0;
    virtual void RenderOffScreen(const std::string& _imgName="/run/user/1000/") = 0;
    virtual bool LoadPcdFile(const std::string&) = 0;    
    virtual std::string GetVersion()const=0;
    virtual void MoveCamera(double _shiftX,double _shiftY,double _Z)=0;
    virtual void RPYCamera(double _R,double _P,double _Y)=0;
    virtual void ZoomCamera(double _scale)=0;
 
};

PclRendererInterface* Init();

void Destroy(PclRendererInterface* _obj);

bool LoadPcdFile(PclRendererInterface* _obj, const std::string&);
bool LoadPcdFile2(PclRendererInterface* _obj);
void Render(PclRendererInterface* _obj);
void RenderOffScreen(PclRendererInterface* _obj);
char* GetVersion(PclRendererInterface*);
void MoveCamera(PclRendererInterface* _obj,double _shiftX,double _shiftY,double _Z);
void RPYCamera(PclRendererInterface* _obj,double _R,double _P,double _Y);
void ZoomCamera(PclRendererInterface* ,double _scale);
 
};