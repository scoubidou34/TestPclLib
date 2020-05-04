#pragma once

#include <iostream>
#include <memory>
#include <string>

extern "C" {

    struct PclRendererInterface;
    


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
 void ResetCamera(PclRendererInterface*);
 void CreateMeshing(PclRendererInterface*);
 void CreatePointsCloud(PclRendererInterface*);
};