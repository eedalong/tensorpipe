#include <iostream>
#include <stdio.h>
#include <chrono>
#include <ctime>
#include <cstring>
int main(){

    size_t data_size1 = 128 * 1024 * 1024;
    uint8_t * src1 = (uint8_t *)malloc(data_size1);
    uint8_t * dst1 = (uint8_t *)malloc(data_size1);

    size_t data_size2 = 8;
    uint8_t * src2 = (uint8_t *)malloc(data_size2);
    uint8_t * dst2 = (uint8_t *)malloc(data_size2);
    // warm up

    
    
    for(int i=0; i < 10; i++){
        memcpy(dst1, src1, data_size1);
    }

    auto start = std::chrono::system_clock::now();

    memcpy(dst1, src1, data_size1);
    memcpy(dst2, src2, data_size2);
    memcpy(dst2, src2, data_size2);
    memcpy(dst2, src2, data_size2);


    //memcpy(dst1, src1, data_size1);

    //memcpy(dst2, src2, data_size2);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    std::cout << "Memory Copy Bandwidth " << float(data_size1) / elapsed_seconds.count() / 1024 / 1024 / 1024 << " GB/s" <<std::endl;




}