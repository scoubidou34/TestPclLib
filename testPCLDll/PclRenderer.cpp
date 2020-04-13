
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