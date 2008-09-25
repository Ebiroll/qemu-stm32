/* See if various MMX/SSE SSSE3 instructions give expected results */
#include <stdio.h>
#include <string.h>

int main(int argc, char *argv[]) {
	char hello[16];
	const char ehlo[8] = "EHLO    ";
	uint64_t mask = 0x8080800302020001;

	uint64_t a = 0x0000000000090007;
	uint64_t b = 0x0000000000000000;

	const char c[16] = "LLOaaaaaaaaaaaaa";
	const char d[16] = "aaaaaaaaaaaaaaHE";

	/* pshufb mm1/xmm1, mm2/xmm2 */
	asm volatile ("movq    (%0), %%mm0" : : "r" (ehlo) : "mm0", "mm1");
	asm volatile ("movq    %0, %%mm1" : : "m" (mask));
	asm volatile ("pshufb  %mm1, %mm0");
	asm volatile ("movq    %%mm0, %0" : "=m" (hello));
	printf("%s\n", hello);

	/* pshufb mm1/xmm1, m64/m128 */
	asm volatile ("movq    (%0), %%mm0" : : "r" (ehlo) : "mm0");
	asm volatile ("pshufb  %0, %%mm0" : : "m" (mask));
	asm volatile ("movq    %%mm0, %0" : "=m" (hello));
	printf("%s\n", hello);

	/* psubsw mm1/xmm1, m64/m128 */
	asm volatile ("movq    %0, %%mm0" : : "r" (a) : "mm0");
	asm volatile ("phsubsw %0, %%mm0" : : "m" (b));
	asm volatile ("movq    %%mm0, %0" : "=m" (a));
	printf("%i - %i = %i\n", 9, 7, -(int16_t) a);

	/* palignr mm1/xmm1, m64/m128, imm8 */
	asm volatile ("movdqa  (%0), %%xmm0" : : "r" (c) : "xmm0");
	asm volatile ("palignr $14, (%0), %%xmm0" : : "r" (d));
	asm volatile ("movdqa  %%xmm0, (%0)" : : "r" (hello));
	printf("%5.5s\n", hello);

	return 0;
}
