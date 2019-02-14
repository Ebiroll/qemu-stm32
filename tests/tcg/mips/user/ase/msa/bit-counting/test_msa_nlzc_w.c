/*
 *  Test program for MSA instruction NLZC.W
 *
 *  Copyright (C) 2018  Wave Computing, Inc.
 *  Copyright (C) 2018  Aleksandar Markovic <amarkovic@wavecomp.com>
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

#include "../../../include/wrappers_msa.h"
#include "../../../include/test_inputs.h"
#include "../../../include/test_utils.h"

#define TEST_COUNT_TOTAL (PATTERN_INPUTS_COUNT + RANDOM_INPUTS_COUNT)


int32_t main(void)
{
    char *instruction_name = "NLZC.W";
    int32_t ret;
    uint32_t i;
    struct timeval start, end;
    double elapsed_time;

    uint64_t b128_result[TEST_COUNT_TOTAL][2];
    uint64_t b128_expect[TEST_COUNT_TOTAL][2] = {
        { 0x0000000000000000ULL, 0x0000000000000000ULL, },    /*   0  */
        { 0x0000002000000020ULL, 0x0000002000000020ULL, },
        { 0x0000000000000000ULL, 0x0000000000000000ULL, },
        { 0x0000000100000001ULL, 0x0000000100000001ULL, },
        { 0x0000000000000000ULL, 0x0000000000000000ULL, },
        { 0x0000000200000002ULL, 0x0000000200000002ULL, },
        { 0x0000000000000000ULL, 0x0000000200000000ULL, },
        { 0x0000000300000001ULL, 0x0000000000000003ULL, },
        { 0x0000000000000000ULL, 0x0000000000000000ULL, },    /*   8  */
        { 0x0000000400000004ULL, 0x0000000400000004ULL, },
        { 0x0000000000000000ULL, 0x0000000000000004ULL, },
        { 0x0000000500000003ULL, 0x0000000100000000ULL, },
        { 0x0000000000000004ULL, 0x0000000000000000ULL, },
        { 0x0000000600000000ULL, 0x0000000200000006ULL, },
        { 0x0000000000000000ULL, 0x0000000600000002ULL, },
        { 0x0000000700000003ULL, 0x0000000000000000ULL, },
        { 0x0000000000000000ULL, 0x0000000000000000ULL, },    /*  16  */
        { 0x0000000800000008ULL, 0x0000000800000008ULL, },
        { 0x0000000000000004ULL, 0x0000000800000000ULL, },
        { 0x0000000900000000ULL, 0x0000000000000003ULL, },
        { 0x0000000000000008ULL, 0x0000000000000004ULL, },
        { 0x0000000a00000000ULL, 0x0000000600000000ULL, },
        { 0x0000000000000000ULL, 0x0000000200000000ULL, },
        { 0x0000000b00000001ULL, 0x0000000000000003ULL, },
        { 0x0000000000000000ULL, 0x0000000800000000ULL, },    /*  24  */
        { 0x0000000c00000004ULL, 0x000000000000000cULL, },
        { 0x0000000000000000ULL, 0x0000000000000008ULL, },
        { 0x0000000d00000007ULL, 0x0000000100000000ULL, },
        { 0x0000000000000000ULL, 0x0000000000000000ULL, },
        { 0x0000000e0000000aULL, 0x0000000600000002ULL, },
        { 0x0000000000000000ULL, 0x0000000000000000ULL, },
        { 0x0000000f0000000dULL, 0x0000000b00000009ULL, },
        { 0x0000000000000000ULL, 0x0000000000000000ULL, },    /*  32  */
        { 0x0000001000000010ULL, 0x0000001000000010ULL, },
        { 0x0000000000000002ULL, 0x0000000400000006ULL, },
        { 0x0000001100000000ULL, 0x0000000000000000ULL, },
        { 0x0000000000000004ULL, 0x000000080000000cULL, },
        { 0x0000001200000000ULL, 0x0000000000000000ULL, },
        { 0x0000000000000006ULL, 0x0000000c00000012ULL, },
        { 0x0000001300000000ULL, 0x0000000000000000ULL, },
        { 0x0000000000000008ULL, 0x0000001000000000ULL, },    /*  40  */
        { 0x0000001400000000ULL, 0x0000000000000004ULL, },
        { 0x000000000000000aULL, 0x0000001400000000ULL, },
        { 0x0000001500000000ULL, 0x0000000000000009ULL, },
        { 0x000000000000000cULL, 0x0000000000000000ULL, },
        { 0x0000001600000000ULL, 0x000000020000000eULL, },
        { 0x000000000000000eULL, 0x0000000000000000ULL, },
        { 0x0000001700000000ULL, 0x0000000500000013ULL, },
        { 0x0000000000000010ULL, 0x0000000000000000ULL, },    /*  48  */
        { 0x0000001800000000ULL, 0x0000000800000018ULL, },
        { 0x0000000000000012ULL, 0x0000000000000004ULL, },
        { 0x0000001900000000ULL, 0x0000000b00000000ULL, },
        { 0x0000000000000014ULL, 0x0000000000000008ULL, },
        { 0x0000001a00000000ULL, 0x0000000e00000000ULL, },
        { 0x0000000000000016ULL, 0x000000000000000cULL, },
        { 0x0000001b00000000ULL, 0x0000001100000000ULL, },
        { 0x0000000000000018ULL, 0x0000000000000010ULL, },    /*  56  */
        { 0x0000001c00000000ULL, 0x0000001400000000ULL, },
        { 0x000000000000001aULL, 0x0000000000000014ULL, },
        { 0x0000001d00000000ULL, 0x0000001700000000ULL, },
        { 0x000000000000001cULL, 0x0000000000000018ULL, },
        { 0x0000001e00000000ULL, 0x0000001a00000000ULL, },
        { 0x000000000000001eULL, 0x000000000000001cULL, },
        { 0x0000001f00000000ULL, 0x0000001d00000000ULL, },
        { 0x0000000000000002ULL, 0x0000000100000000ULL, },    /*  64  */
        { 0x0000000000000001ULL, 0x0000000300000003ULL, },
        { 0x0000000000000000ULL, 0x0000000200000000ULL, },
        { 0x0000000100000001ULL, 0x0000000000000000ULL, },
        { 0x0000000000000001ULL, 0x0000000000000000ULL, },
        { 0x0000000000000000ULL, 0x0000000300000000ULL, },
        { 0x0000000000000001ULL, 0x0000000000000000ULL, },
        { 0x0000000000000000ULL, 0x0000000000000003ULL, },
        { 0x0000000200000001ULL, 0x0000000000000000ULL, },    /*  72  */
        { 0x0000000000000001ULL, 0x0000000000000000ULL, },
        { 0x0000000000000000ULL, 0x0000000000000001ULL, },
        { 0x0000000000000002ULL, 0x0000000000000001ULL, },
        { 0x0000000000000000ULL, 0x0000000000000001ULL, },
        { 0x0000000000000003ULL, 0x0000000000000001ULL, },
        { 0x0000000100000002ULL, 0x0000000000000000ULL, },
        { 0x0000000000000000ULL, 0x0000000000000001ULL, },
    };

    gettimeofday(&start, NULL);

    for (i = 0; i < TEST_COUNT_TOTAL; i++) {
        if (i < PATTERN_INPUTS_COUNT) {
            do_msa_NLZC_W(b128_pattern[i], b128_result[i]);
        } else {
            do_msa_NLZC_W(b128_random[i - PATTERN_INPUTS_COUNT],
                          b128_result[i]);
        }
    }

    gettimeofday(&end, NULL);

    elapsed_time = (end.tv_sec - start.tv_sec) * 1000.0;
    elapsed_time += (end.tv_usec - start.tv_usec) / 1000.0;

    ret = check_results(instruction_name, TEST_COUNT_TOTAL, elapsed_time,
                        &b128_result[0][0], &b128_expect[0][0]);

    return ret;
}
