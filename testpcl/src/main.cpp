
#include "export.h"
#include <memory>

/*
 *
 */
int main(int argc, char** argv)
{

   //  void* handle = dlopen("myclass.so", RTLD_LAZY);
    PclRendererInterface* pclPtr(Init());
    pclPtr->LoadPcdFile("rops_cloud.pcd");
    pclPtr->Render();   
    Destroy(pclPtr);
    return (0);
}
