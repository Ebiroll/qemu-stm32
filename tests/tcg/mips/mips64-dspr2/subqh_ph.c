#include"io.h"

int main(void)
{
    long long rd, rs, rt;
    long long result;

    rs = 0x12345678;
    rt = 0x87654321;
    result = 0x456709AB;

    __asm
        ("subqh.ph %0, %1, %2\n\t"
         : "=r"(rd)
         : "r"(rs), "r"(rt)
        );
    if (rd != result) {
        printf("subqh.ph error\n");
        return -1;
    }

    return 0;
}
