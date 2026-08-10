#if not defined(omp_set_num_threads)
    #define omp_set_num_threads(num_threads) (void) 0
    #define omp_get_max_threads() 1
    #define omp_get_thread_num() 0
#endif
