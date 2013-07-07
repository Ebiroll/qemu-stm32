/*
 * CRIS gdb server stub
 *
 * Copyright (c) 2003-2005 Fabrice Bellard
 * Copyright (c) 2013 SUSE LINUX Products GmbH
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

static int
read_register_crisv10(CPUCRISState *env, uint8_t *mem_buf, int n)
{
    if (n < 15) {
        return gdb_get_reg32(mem_buf, env->regs[n]);
    }

    if (n == 15) {
        return gdb_get_reg32(mem_buf, env->pc);
    }

    if (n < 32) {
        switch (n) {
        case 16:
            return gdb_get_reg8(mem_buf, env->pregs[n - 16]);
        case 17:
            return gdb_get_reg8(mem_buf, env->pregs[n - 16]);
        case 20:
        case 21:
            return gdb_get_reg16(mem_buf, env->pregs[n - 16]);
        default:
            if (n >= 23) {
                return gdb_get_reg32(mem_buf, env->pregs[n - 16]);
            }
            break;
        }
    }
    return 0;
}

static int cpu_gdb_read_register(CPUCRISState *env, uint8_t *mem_buf, int n)
{
    uint8_t srs;

    if (env->pregs[PR_VR] < 32) {
        return read_register_crisv10(env, mem_buf, n);
    }

    srs = env->pregs[PR_SRS];
    if (n < 16) {
        return gdb_get_reg32(mem_buf, env->regs[n]);
    }

    if (n >= 21 && n < 32) {
        return gdb_get_reg32(mem_buf, env->pregs[n - 16]);
    }
    if (n >= 33 && n < 49) {
        return gdb_get_reg32(mem_buf, env->sregs[srs][n - 33]);
    }
    switch (n) {
    case 16:
        return gdb_get_reg8(mem_buf, env->pregs[0]);
    case 17:
        return gdb_get_reg8(mem_buf, env->pregs[1]);
    case 18:
        return gdb_get_reg32(mem_buf, env->pregs[2]);
    case 19:
        return gdb_get_reg8(mem_buf, srs);
    case 20:
        return gdb_get_reg16(mem_buf, env->pregs[4]);
    case 32:
        return gdb_get_reg32(mem_buf, env->pc);
    }

    return 0;
}

static int cpu_gdb_write_register(CPUCRISState *env, uint8_t *mem_buf, int n)
{
    uint32_t tmp;

    if (n > 49) {
        return 0;
    }

    tmp = ldl_p(mem_buf);

    if (n < 16) {
        env->regs[n] = tmp;
    }

    if (n >= 21 && n < 32) {
        env->pregs[n - 16] = tmp;
    }

    /* FIXME: Should support function regs be writable?  */
    switch (n) {
    case 16:
        return 1;
    case 17:
        return 1;
    case 18:
        env->pregs[PR_PID] = tmp;
        break;
    case 19:
        return 1;
    case 20:
        return 2;
    case 32:
        env->pc = tmp;
        break;
    }

    return 4;
}
