
#include "export.h"
#include <memory>
 #include <chrono>

/*
 *
 */
int main(int argc, char** argv)
{

    //  void* handle = dlopen("myclass.so", RTLD_LAZY);
    PclRendererInterface* pclPtr(Init());
    pclPtr->LoadPcdFile("rops_cloud.pcd");

    pclPtr->RenderOffScreen();
    Destroy(pclPtr);
    return (0);
}
