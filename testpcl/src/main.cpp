
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
    for(auto i = 0; i < 100; i++) {
        pclPtr->MoveCamera(double(i)*0.05, double(i)*0.05);
        pclPtr->RenderOffScreen("image" + std::to_string(i) + ".png");
    }
Destroy(pclPtr);
return (0);
}
