#if not defined(omp_set_num_threads)
    #define omp_set_num_threads(num_threads) (void) 0
#endif
