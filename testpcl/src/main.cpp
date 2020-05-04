
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
   LoadPcdFile(pclPtr,"rops_cloud.pcd");
   CreatePointsCloud(pclPtr);
   CreateMeshing(pclPtr);
   Render(pclPtr);

    Destroy(pclPtr);
    return (0);
}
