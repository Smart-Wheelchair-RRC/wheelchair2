#include <iostream>
#include <cuda_runtime.h>

// CUDA Kernel for vector multiplication
__global__ void multiply(float *a, float *b, float *c, int n) {
    int i = threadIdx.x + blockIdx.x * blockDim.x;
    if (i < n) {
        c[i] = a[i] * b[i];
    }
}

int main() {
    int n = 1024;
    size_t size = n * sizeof(float);

    // Allocate host memory
    float *h_a = (float *)malloc(size);
    float *h_b = (float *)malloc(size);
    float *h_c = (float *)malloc(size);

    for (int i = 0; i < n; i++) {
        h_a[i] = 2.0f;
        h_b[i] = 3.0f;
    }

    // Allocate device memory
    float *d_a, *d_b, *d_c;
    cudaMalloc(&d_a, size);
    cudaMalloc(&d_b, size);
    cudaMalloc(&d_c, size);

    // Copy data to GPU
    cudaMemcpy(d_a, h_a, size, cudaMemcpyHostToDevice);
    cudaMemcpy(d_b, h_b, size, cudaMemcpyHostToDevice);

    // Launch kernel (1 block of 1024 threads)
    multiply<<<1, n>>>(d_a, d_b, d_c, n);

    // Copy result back to host
    cudaMemcpy(h_c, d_c, size, cudaMemcpyDeviceToHost);

    // Verify
    bool success = true;
    for (int i = 0; i < n; i++) {
        if (h_c[i] != 6.0f) {
            success = false;
            break;
        }
    }

    if (success) {
        std::cout << "🚀 GPU multiplication successful! Result: " << h_c[0] << std::endl;
    } else {
        std::cout << "❌ GPU test failed." << std::endl;
    }

    // Cleanup
    cudaFree(d_a); cudaFree(d_b); cudaFree(d_c);
    free(h_a); free(h_b); free(h_c);

    return 0;
}
