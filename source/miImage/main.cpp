#include "miImage.h"
#include "miImageProcessing.h"
#include "miDepthProcessing.h"

#include <iostream>

int main(int argc, char* argv[]) {

    int arr[10] = {1,2,3,4,5,6,7,8,9,10};
    int ary[10] = {0};

    std::copy(&arr[5],&arr[5]+5, ary);


    for(int i=0; i<10; i++)
    {
        std::cout<<ary[i]<<" "<<arr[i]<<std::endl;
    }

    return 0;
}