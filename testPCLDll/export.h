#pragma once

#include <iostream>
#include <string>
#include <memory>



class PclRendererInterface
{
public:
    virtual bool LoadPcdFile(const std::string&)=0;
    virtual void Render()=0;
    virtual void RenderOffScreen()=0;

};

PclRendererInterface* Init();

void Destroy(PclRendererInterface* _obj);

 
