#include <stdio.h>
// conv_size: 2^5
typedef int u8;

// Convolution kernel function
void conv_kernel(u8 *kernel_addr, u8 kernel_size) {
    int res = 0;
    asm volatile ( 
        "addi zero,zero,0\n"
        ".insn r 0x77, 2, 0, %[null], %[ker_addr], %[ker_size]"
        :[null] "=r"(res)
        :[ker_addr] "r" (kernel_addr), [ker_size] "r" (kernel_size) 
     );
}

// Convolution calculation function
void conv_cal(u8 *conv_addr, u8 conv_size) {
    int res = 0;
    asm volatile ( 
        "addi zero,zero,0\n"
        ".insn r 0x77, 1, 0, %[null], %[cv_addr], %[cv_size]"
        :[null] "=r"(res)
        :[cv_addr] "r" (conv_addr), [cv_size] "r" (conv_size) 
);
}

// Write-back function to store results
void conv_wb(u8 *res_addr) {
    int res1 = 0, res2 = 0;
    asm volatile ( 
        "addi zero,zero,0\n"
        ".insn r 0x77, 1, 1, %[null1], %[rs_addr], %[null2]"
        :[null1] "=r"(res1)
        :[rs_addr] "r" (res_addr), [null2] "r" (res2) 
     );
}

// Finish function to indicate the end of processing
void finish() {
    int res1 = 0, res2 = 0, res3 = 0;
    asm volatile ( 
        "addi zero,zero,0\n"
        ".insn r 0x77, 2, 1, %[null1], %[null2], %[null3]"
        :[null1] "=r"(res1)
        :[null2] "r" (res2), [null3] "r" (res3) 
     );
     printf("Hello world in finish\n");
     return;
}

// Function to read the cycle count
static inline unsigned long rdcycle() {
    unsigned int cycle;
    __asm__ volatile ("rdcycle %0" : "=r" (cycle));
    return cycle;
}

// 普通卷积计算函数
void conv_cal_normal(u8 *input, u8 *kernel, u8 *output, int input_size, int kernel_size) {
    int k_center = kernel_size / 2;
    
    // 遍历输入矩阵的每个位置
    for (int i = 0; i < input_size; i++) {
        for (int j = 0; j < input_size; j++) {
            int sum = 0;
            
            // 对于每个输出位置，计算卷积
            for (int m = 0; m < kernel_size; m++) {
                for (int n = 0; n < kernel_size; n++) {
                    int x = i + m - k_center;
                    int y = j + n - k_center;
                    
                    // 边界检查
                    if (x >= 0 && x < input_size && y >= 0 && y < input_size) {
                        sum += input[x * input_size + y] * kernel[m * kernel_size + n];
                    }
                }
            }
            // 将结果存储到输出矩阵
            output[i * input_size + j] = sum;
        }
    }
}
static u8 input[4*4]=
{
    1,1,1,1,
    1,1,1,1,
    1,1,1,1,
    1,1,1,1
};

// 3x3 Convolution kernel
static u8 kernel3x3[3 * 3] = {
    0, 1, 0,
    1, 0, 1,
    0, 1, 0,
};

int main() {

    unsigned long start_time, end_time;
    unsigned long total = 0;

    // Convolution for 224x224 using 4x4 blocks
    u8 res1[2 * 2]; // Result array for storing 224x224 result
    u8 res2[2 * 2];
    
    start_time = rdcycle(); // Start cycle count
    
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            // Address of the 4x4 block in input
            u8 *block_addr = &input[(i * 4) * 4 + (j * 4)];
            
            // Apply convolution on the 4x4 block
            //start_time = rdcycle(); 
            
            conv_kernel(kernel3x3, 9); // Apply kernel (3x3)
            conv_cal(block_addr, 4); // Apply convolution on block (224 elements)
            conv_wb(&res1[(i * 4) * 2 + (j * 4)]); // Store result back
            
            //end_time = rdcycle(); // End cycle count
            //total += end_time - start_time;
        }
    }

    end_time = rdcycle(); // End cycle count
    printf("Total cycles taken: %lu\n", end_time - start_time);


    start_time = rdcycle();
    conv_cal_normal(input, kernel3x3, res2, 16, 3);
    end_time = rdcycle();
    printf("Normal conv cycles: %lu\n", end_time - start_time);


    return 0;
}
