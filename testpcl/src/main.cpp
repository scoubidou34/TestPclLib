
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
    for(auto i = 0; i < 100; i++) {
       pclPtr->MoveCamera(0.05, 0);
        auto start = std::chrono::steady_clock::now();
   //    pclPtr->RenderOffScreen("image" + std::to_string(i) + ".png");      
        auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
  //  std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
    }
Destroy(pclPtr);
return (0);
}
