#include <stdint.h> 

typedef struct {
    double A[2];
    double B[2];
    double des_pos[2];
    uint8_t index;
    double base_norm;
    double t_marker;
} TrackData;