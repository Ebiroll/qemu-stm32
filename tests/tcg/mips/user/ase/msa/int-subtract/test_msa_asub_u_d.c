/*
 *  Test program for MSA instruction ASUB_U.D
 *
 *  Copyright (C) 2019  Wave Computing, Inc.
 *  Copyright (C) 2019  Aleksandar Markovic <amarkovic@wavecomp.com>
 *  Copyright (C) 2019  RT-RK Computer Based Systems LLC
 *  Copyright (C) 2019  Mateja Marjanovic <mateja.marjanovic@rt-rk.com>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include <sys/time.h>
#include <stdint.h>

#include "../../../../include/wrappers_msa.h"
#include "../../../../include/test_inputs_128.h"
#include "../../../../include/test_utils_128.h"

#define TEST_COUNT_TOTAL (                                                \
            (PATTERN_INPUTS_SHORT_COUNT) * (PATTERN_INPUTS_SHORT_COUNT) + \
            (RANDOM_INPUTS_SHORT_COUNT) * (RANDOM_INPUTS_SHORT_COUNT))


int32_t main(void)
{
    char *isa_ase_name = "MSA";
    char *group_name = "Int Subtract";
    char *instruction_name =  "ASUB_U.D";
    int32_t ret;
    uint32_t i, j;
    struct timeval start, end;
    double elapsed_time;

    uint64_t b128_result[TEST_COUNT_TOTAL][2];
    uint64_t b128_expect[TEST_COUNT_TOTAL][2] = {
        { 0x0000000000000000ULL, 0x0000000000000000ULL, },    /*   0  */
        { 0xffffffffffffffffULL, 0xffffffffffffffffULL, },
        { 0x5555555555555555ULL, 0x5555555555555555ULL, },
        { 0xaaaaaaaaaaaaaaaaULL, 0xaaaaaaaaaaaaaaaaULL, },
        { 0x3333333333333333ULL, 0x3333333333333333ULL, },
        { 0xccccccccccccccccULL, 0xccccccccccccccccULL, },
        { 0x1c71c71c71c71c71ULL, 0xc71c71c71c71c71cULL, },
        { 0xe38e38e38e38e38eULL, 0x38e38e38e38e38e3ULL, },
        { 0xffffffffffffffffULL, 0xffffffffffffffffULL, },    /*   8  */
        { 0x0000000000000000ULL, 0x0000000000000000ULL, },
        { 0xaaaaaaaaaaaaaaaaULL, 0xaaaaaaaaaaaaaaaaULL, },
        { 0x5555555555555555ULL, 0x5555555555555555ULL, },
        { 0xccccccccccccccccULL, 0xccccccccccccccccULL, },
        { 0x3333333333333333ULL, 0x3333333333333333ULL, },
        { 0xe38e38e38e38e38eULL, 0x38e38e38e38e38e3ULL, },
        { 0x1c71c71c71c71c71ULL, 0xc71c71c71c71c71cULL, },
        { 0x5555555555555555ULL, 0x5555555555555555ULL, },    /*  16  */
        { 0xaaaaaaaaaaaaaaaaULL, 0xaaaaaaaaaaaaaaaaULL, },
        { 0x0000000000000000ULL, 0x0000000000000000ULL, },
        { 0x5555555555555555ULL, 0x5555555555555555ULL, },
        { 0x2222222222222222ULL, 0x2222222222222222ULL, },
        { 0x7777777777777777ULL, 0x7777777777777777ULL, },
        { 0x38e38e38e38e38e4ULL, 0x71c71c71c71c71c7ULL, },
        { 0x8e38e38e38e38e39ULL, 0x1c71c71c71c71c72ULL, },
        { 0xaaaaaaaaaaaaaaaaULL, 0xaaaaaaaaaaaaaaaaULL, },    /*  24  */
        { 0x5555555555555555ULL, 0x5555555555555555ULL, },
        { 0x5555555555555555ULL, 0x5555555555555555ULL, },
        { 0x0000000000000000ULL, 0x0000000000000000ULL, },
        { 0x7777777777777777ULL, 0x7777777777777777ULL, },
        { 0x2222222222222222ULL, 0x2222222222222222ULL, },
        { 0x8e38e38e38e38e39ULL, 0x1c71c71c71c71c72ULL, },
        { 0x38e38e38e38e38e4ULL, 0x71c71c71c71c71c7ULL, },
        { 0x3333333333333333ULL, 0x3333333333333333ULL, },    /*  32  */
        { 0xccccccccccccccccULL, 0xccccccccccccccccULL, },
        { 0x2222222222222222ULL, 0x2222222222222222ULL, },
        { 0x7777777777777777ULL, 0x7777777777777777ULL, },
        { 0x0000000000000000ULL, 0x0000000000000000ULL, },
        { 0x9999999999999999ULL, 0x9999999999999999ULL, },
        { 0x16c16c16c16c16c2ULL, 0x93e93e93e93e93e9ULL, },
        { 0xb05b05b05b05b05bULL, 0x05b05b05b05b05b0ULL, },
        { 0xccccccccccccccccULL, 0xccccccccccccccccULL, },    /*  40  */
        { 0x3333333333333333ULL, 0x3333333333333333ULL, },
        { 0x7777777777777777ULL, 0x7777777777777777ULL, },
        { 0x2222222222222222ULL, 0x2222222222222222ULL, },
        { 0x9999999999999999ULL, 0x9999999999999999ULL, },
        { 0x0000000000000000ULL, 0x0000000000000000ULL, },
        { 0xb05b05b05b05b05bULL, 0x05b05b05b05b05b0ULL, },
        { 0x16c16c16c16c16c2ULL, 0x93e93e93e93e93e9ULL, },
        { 0x1c71c71c71c71c71ULL, 0xc71c71c71c71c71cULL, },    /*  48  */
        { 0xe38e38e38e38e38eULL, 0x38e38e38e38e38e3ULL, },
        { 0x38e38e38e38e38e4ULL, 0x71c71c71c71c71c7ULL, },
        { 0x8e38e38e38e38e39ULL, 0x1c71c71c71c71c72ULL, },
        { 0x16c16c16c16c16c2ULL, 0x93e93e93e93e93e9ULL, },
        { 0xb05b05b05b05b05bULL, 0x05b05b05b05b05b0ULL, },
        { 0x0000000000000000ULL, 0x0000000000000000ULL, },
        { 0xc71c71c71c71c71dULL, 0x8e38e38e38e38e39ULL, },
        { 0xe38e38e38e38e38eULL, 0x38e38e38e38e38e3ULL, },    /*  56  */
        { 0x1c71c71c71c71c71ULL, 0xc71c71c71c71c71cULL, },
        { 0x8e38e38e38e38e39ULL, 0x1c71c71c71c71c72ULL, },
        { 0x38e38e38e38e38e4ULL, 0x71c71c71c71c71c7ULL, },
        { 0xb05b05b05b05b05bULL, 0x05b05b05b05b05b0ULL, },
        { 0x16c16c16c16c16c2ULL, 0x93e93e93e93e93e9ULL, },
        { 0xc71c71c71c71c71dULL, 0x8e38e38e38e38e39ULL, },
        { 0x0000000000000000ULL, 0x0000000000000000ULL, },
        { 0x0000000000000000ULL, 0x0000000000000000ULL, },    /*  64  */
        { 0x73531997253171c8ULL, 0x386f5044e93c5d10ULL, },
        { 0x23efc7de916d3640ULL, 0x238e445f53508af8ULL, },
        { 0x181bd07eca3072f2ULL, 0x428a7d79aac73294ULL, },
        { 0x73531997253171c8ULL, 0x386f5044e93c5d10ULL, },
        { 0x0000000000000000ULL, 0x0000000000000000ULL, },
        { 0x4f6351b893c43b88ULL, 0x14e10be595ebd218ULL, },
        { 0x8b6eea15ef61e4baULL, 0x7af9cdbe94038fa4ULL, },
        { 0x23efc7de916d3640ULL, 0x238e445f53508af8ULL, },    /*  72  */
        { 0x4f6351b893c43b88ULL, 0x14e10be595ebd218ULL, },
        { 0x0000000000000000ULL, 0x0000000000000000ULL, },
        { 0x3c0b985d5b9da932ULL, 0x6618c1d8fe17bd8cULL, },
        { 0x181bd07eca3072f2ULL, 0x428a7d79aac73294ULL, },
        { 0x8b6eea15ef61e4baULL, 0x7af9cdbe94038fa4ULL, },
        { 0x3c0b985d5b9da932ULL, 0x6618c1d8fe17bd8cULL, },
        { 0x0000000000000000ULL, 0x0000000000000000ULL, },
};

    reset_msa_registers();

    gettimeofday(&start, NULL);

    for (i = 0; i < PATTERN_INPUTS_SHORT_COUNT; i++) {
        for (j = 0; j < PATTERN_INPUTS_SHORT_COUNT; j++) {
            do_msa_ASUB_U_D(b128_pattern[i], b128_pattern[j],
                           b128_result[PATTERN_INPUTS_SHORT_COUNT * i + j]);
        }
    }

    for (i = 0; i < RANDOM_INPUTS_SHORT_COUNT; i++) {
        for (j = 0; j < RANDOM_INPUTS_SHORT_COUNT; j++) {
            do_msa_ASUB_U_D(b128_random[i], b128_random[j],
                           b128_result[((PATTERN_INPUTS_SHORT_COUNT) *
                                        (PATTERN_INPUTS_SHORT_COUNT)) +
                                       RANDOM_INPUTS_SHORT_COUNT * i + j]);
        }
    }

    gettimeofday(&end, NULL);

    elapsed_time = (end.tv_sec - start.tv_sec) * 1000.0;
    elapsed_time += (end.tv_usec - start.tv_usec) / 1000.0;

    ret = check_results_128(isa_ase_name, group_name, instruction_name,
                            TEST_COUNT_TOTAL, elapsed_time,
                            &b128_result[0][0], &b128_expect[0][0]);

    return ret;
}
