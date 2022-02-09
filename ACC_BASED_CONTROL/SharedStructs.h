#ifndef SHAREDSTRUCT_H
#define SHAREDSTRUCT_H
// File for structs definitions:

#include <stdio.h>
#include <thread>
#include <mutex>

typedef struct shared_struct {
    float sampletime_;
    int     exectime_;
    float      *data_;
    std::mutex  *mtx_;
    float *datavec_[18];
    // useful parameters:
    short param00_; // thread priority
    short param01_;
    short param02_;
    short param03_;
    short param04_;
    short param05_;
    short param06_;
    short param07_;
    short param08_;
    short param09_;
    short param0A_;
    short param0B_;
    short param0C_;
    short param0D_;
    short param0E_;
    short param0F_;
} ThrdStruct;

#endif // SHAREDSTRUCT_H