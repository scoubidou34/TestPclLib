#pragma once

#include <iostream>
#include <memory>
#include <string>

extern "C" {

struct PclRendererInterface {
public:
    virtual void Render() = 0;
    virtual void RenderOffScreen() = 0;
    virtual bool LoadPcdFile(const std::string&) = 0;    
    virtual std::string GetVersion()const=0;
};

PclRendererInterface* Init();

void Destroy(PclRendererInterface* _obj);

bool LoadPcdFile(PclRendererInterface* _obj, const std::string&);
bool LoadPcdFile2(PclRendererInterface* _obj);
void Render(PclRendererInterface* _obj);
void RenderOffScreen(PclRendererInterface* _obj);
char* GetVersion(PclRendererInterface*);
};