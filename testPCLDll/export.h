#pragma once

#include <iostream>
#include <memory>
#include <string>

extern "C" {

struct PclRendererInterface {
public:
    virtual void Render() = 0;
    virtual void RenderOffScreen(const std::string& _imgName="image.png") = 0;
    virtual bool LoadPcdFile(const std::string&) = 0;    
    virtual std::string GetVersion()const=0;
    virtual void MoveCamera(double _shiftX,double _shiftY)=0;
};

PclRendererInterface* Init();

void Destroy(PclRendererInterface* _obj);

bool LoadPcdFile(PclRendererInterface* _obj, const std::string&);
bool LoadPcdFile2(PclRendererInterface* _obj);
void Render(PclRendererInterface* _obj);
void RenderOffScreen(PclRendererInterface* _obj);
char* GetVersion(PclRendererInterface*);
void MoveCamera(PclRendererInterface* _obj,double _shiftX,double _shiftY);
};