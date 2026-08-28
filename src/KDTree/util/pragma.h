#pragma once
#ifdef _OPENMP
#include <omp.h>
#else
inline void omp_set_num_threads(int) {
}
inline int omp_get_max_threads() {
    return 1;
}
inline int omp_get_thread_num() {
    return 0;
}
#endif
