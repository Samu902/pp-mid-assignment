#ifdef _OPENMP
#include <omp.h> // for OpenMP library functions
#endif
#include <cstdio>
#include <iostream>
#include <cstdlib>

int main(int argc, char* argv[]) {
    int nthreads, tid;
    tid = -1;

    #ifdef _OPENMP
    std::cout << "_OPENMP defined" << std::endl;
    std::cout << "Num processors (Phys+HT): " << omp_get_num_procs() << std::endl;
    #endif

    printf("Global copy of tid before threads execution = %d\n", tid);

    /* Fork a team of threads giving them their own copies of variables */
    #pragma omp parallel private(nthreads, tid)
    {

        #ifdef _OPENMP
        /* Obtain thread number */
        tid = omp_get_thread_num();
        printf("Hello World from thread = %d\n", tid);

        /* Only master thread does this */
        if (tid == 0) {
            nthreads = omp_get_num_threads();
            printf("Master thread - Number of threads = %d\n", nthreads);
        }
        #endif

        tid += 100;
        printf("Local copy of tid = %d\n", tid);

    }  /* All threads join master thread and are destroyed */
    printf("Global copy of tid after threads execution = %d\n", tid);

    // Bernstein's conditions and OpenMP
    {
        const long n = 100;
        int A[n];
        int histo[n];
        #pragma omp parallel for
        for (int i = 0; i < n; i++)
            A[i] = i;
        // XXX the following code will produce bad results !
        #pragma omp parallel for
        for (int i = 1; i < n; i++)
            A[i] = A[i - 1];
        #pragma omp parallel
        {
            #pragma omp for
            for (int i = 0; i < n; i++)
                histo[i] = 0;
            #pragma omp for
            for (int i = 0; i < n; i++)
                histo[A[i]]++;
        }
        for (int i=0; i<n; i++) {
            if (histo[i]>1)
                printf("i: %d - count: %d\n", i, histo[i]);
        }
    }
}