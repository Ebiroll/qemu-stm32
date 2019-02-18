/*
 *  Test program for MSA instruction PCNT.D
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

#include "../../../../include/wrappers_msa.h"
#include "../../../../include/test_inputs.h"
#include "../../../../include/test_utils.h"

#define TEST_COUNT_TOTAL (PATTERN_INPUTS_COUNT + RANDOM_INPUTS_COUNT)


int32_t main(void)
{
    char *instruction_name = "PCNT.D";
    int32_t ret;
    uint32_t i;
    struct timeval start, end;
    double elapsed_time;

    uint64_t b128_result[TEST_COUNT_TOTAL][2];
    uint64_t b128_expect[TEST_COUNT_TOTAL][2] = {
        { 0x0000000000000040ULL, 0x0000000000000040ULL, },    /*   0  */
        { 0x0000000000000000ULL, 0x0000000000000000ULL, },
        { 0x0000000000000020ULL, 0x0000000000000020ULL, },
        { 0x0000000000000020ULL, 0x0000000000000020ULL, },
        { 0x0000000000000020ULL, 0x0000000000000020ULL, },
        { 0x0000000000000020ULL, 0x0000000000000020ULL, },
        { 0x0000000000000021ULL, 0x0000000000000020ULL, },
        { 0x000000000000001fULL, 0x0000000000000020ULL, },
        { 0x0000000000000020ULL, 0x0000000000000020ULL, },    /*   8  */
        { 0x0000000000000020ULL, 0x0000000000000020ULL, },
        { 0x0000000000000022ULL, 0x000000000000001fULL, },
        { 0x000000000000001eULL, 0x0000000000000021ULL, },
        { 0x0000000000000022ULL, 0x0000000000000020ULL, },
        { 0x000000000000001eULL, 0x0000000000000020ULL, },
        { 0x0000000000000023ULL, 0x000000000000001eULL, },
        { 0x000000000000001dULL, 0x0000000000000022ULL, },
        { 0x0000000000000020ULL, 0x0000000000000020ULL, },    /*  16  */
        { 0x0000000000000020ULL, 0x0000000000000020ULL, },
        { 0x0000000000000024ULL, 0x000000000000001dULL, },
        { 0x000000000000001cULL, 0x0000000000000023ULL, },
        { 0x0000000000000022ULL, 0x0000000000000022ULL, },
        { 0x000000000000001eULL, 0x000000000000001eULL, },
        { 0x0000000000000021ULL, 0x0000000000000021ULL, },
        { 0x000000000000001fULL, 0x000000000000001fULL, },
        { 0x0000000000000024ULL, 0x0000000000000020ULL, },    /*  24  */
        { 0x000000000000001cULL, 0x0000000000000020ULL, },
        { 0x0000000000000026ULL, 0x000000000000001bULL, },
        { 0x000000000000001aULL, 0x0000000000000025ULL, },
        { 0x0000000000000024ULL, 0x0000000000000022ULL, },
        { 0x000000000000001cULL, 0x000000000000001eULL, },
        { 0x0000000000000022ULL, 0x0000000000000022ULL, },
        { 0x000000000000001eULL, 0x000000000000001eULL, },
        { 0x0000000000000020ULL, 0x0000000000000020ULL, },    /*  32  */
        { 0x0000000000000020ULL, 0x0000000000000020ULL, },
        { 0x0000000000000022ULL, 0x0000000000000022ULL, },
        { 0x000000000000001eULL, 0x000000000000001eULL, },
        { 0x0000000000000024ULL, 0x0000000000000024ULL, },
        { 0x000000000000001cULL, 0x000000000000001cULL, },
        { 0x0000000000000026ULL, 0x0000000000000021ULL, },
        { 0x000000000000001aULL, 0x000000000000001fULL, },
        { 0x0000000000000028ULL, 0x000000000000001cULL, },    /*  40  */
        { 0x0000000000000018ULL, 0x0000000000000024ULL, },
        { 0x000000000000002aULL, 0x0000000000000017ULL, },
        { 0x0000000000000016ULL, 0x0000000000000029ULL, },
        { 0x000000000000002aULL, 0x0000000000000018ULL, },
        { 0x0000000000000016ULL, 0x0000000000000028ULL, },
        { 0x0000000000000029ULL, 0x000000000000001cULL, },
        { 0x0000000000000017ULL, 0x0000000000000024ULL, },
        { 0x0000000000000028ULL, 0x0000000000000020ULL, },    /*  48  */
        { 0x0000000000000018ULL, 0x0000000000000020ULL, },
        { 0x0000000000000027ULL, 0x0000000000000024ULL, },
        { 0x0000000000000019ULL, 0x000000000000001cULL, },
        { 0x0000000000000026ULL, 0x0000000000000026ULL, },
        { 0x000000000000001aULL, 0x000000000000001aULL, },
        { 0x0000000000000025ULL, 0x0000000000000025ULL, },
        { 0x000000000000001bULL, 0x000000000000001bULL, },
        { 0x0000000000000024ULL, 0x0000000000000024ULL, },    /*  56  */
        { 0x000000000000001cULL, 0x000000000000001cULL, },
        { 0x0000000000000023ULL, 0x0000000000000023ULL, },
        { 0x000000000000001dULL, 0x000000000000001dULL, },
        { 0x0000000000000022ULL, 0x0000000000000022ULL, },
        { 0x000000000000001eULL, 0x000000000000001eULL, },
        { 0x0000000000000021ULL, 0x0000000000000021ULL, },
        { 0x000000000000001fULL, 0x000000000000001fULL, },
        { 0x0000000000000019ULL, 0x0000000000000023ULL, },    /*  64  */
        { 0x000000000000001fULL, 0x0000000000000024ULL, },
        { 0x0000000000000021ULL, 0x0000000000000022ULL, },
        { 0x000000000000001fULL, 0x000000000000001bULL, },
        { 0x0000000000000023ULL, 0x0000000000000022ULL, },
        { 0x0000000000000024ULL, 0x000000000000001cULL, },
        { 0x000000000000001eULL, 0x000000000000001cULL, },
        { 0x0000000000000022ULL, 0x0000000000000022ULL, },
        { 0x000000000000001dULL, 0x0000000000000022ULL, },    /*  72  */
        { 0x0000000000000025ULL, 0x000000000000001eULL, },
        { 0x0000000000000024ULL, 0x000000000000001dULL, },
        { 0x000000000000001fULL, 0x0000000000000024ULL, },
        { 0x000000000000001dULL, 0x000000000000001eULL, },
        { 0x000000000000001dULL, 0x0000000000000021ULL, },
        { 0x000000000000001eULL, 0x0000000000000022ULL, },
        { 0x0000000000000024ULL, 0x0000000000000022ULL, },
    };

    gettimeofday(&start, NULL);

    for (i = 0; i < TEST_COUNT_TOTAL; i++) {
        if (i < PATTERN_INPUTS_COUNT) {
            do_msa_PCNT_D(b128_pattern[i], b128_result[i]);
        } else {
            do_msa_PCNT_D(b128_random[i - PATTERN_INPUTS_COUNT],
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
