/*
 *  Generic Dynamic compiler generator
 * 
 *  Copyright (c) 2003 Fabrice Bellard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>
#include <unistd.h>
#include <fcntl.h>

#include "config.h"

/* elf format definitions. We use these macros to test the CPU to
   allow cross compilation (this tool must be ran on the build
   platform) */
#if defined(HOST_I386)

#define ELF_CLASS	ELFCLASS32
#define ELF_ARCH	EM_386
#define elf_check_arch(x) ( ((x) == EM_386) || ((x) == EM_486) )
#undef ELF_USES_RELOCA

#elif defined(HOST_PPC)

#define ELF_CLASS	ELFCLASS32
#define ELF_ARCH	EM_PPC
#define elf_check_arch(x) ((x) == EM_PPC)
#define ELF_USES_RELOCA

#elif defined(HOST_S390)

#define ELF_CLASS	ELFCLASS32
#define ELF_ARCH	EM_S390
#define elf_check_arch(x) ((x) == EM_S390)
#define ELF_USES_RELOCA

#elif defined(HOST_ALPHA)

#define ELF_CLASS	ELFCLASS64
#define ELF_ARCH	EM_ALPHA
#define elf_check_arch(x) ((x) == EM_ALPHA)
#define ELF_USES_RELOCA

#elif defined(HOST_IA64)

#define ELF_CLASS	ELFCLASS64
#define ELF_ARCH	EM_IA_64
#define elf_check_arch(x) ((x) == EM_IA_64)
#define ELF_USES_RELOCA

#elif defined(HOST_SPARC)

#define ELF_CLASS	ELFCLASS32
#define ELF_ARCH	EM_SPARC
#define elf_check_arch(x) ((x) == EM_SPARC || (x) == EM_SPARC32PLUS)
#define ELF_USES_RELOCA

#elif defined(HOST_SPARC64)

#define ELF_CLASS	ELFCLASS64
#define ELF_ARCH	EM_SPARCV9
#define elf_check_arch(x) ((x) == EM_SPARCV9)
#define ELF_USES_RELOCA

#else
#error unsupported CPU - please update the code
#endif

#include "elf.h"

#if ELF_CLASS == ELFCLASS32
typedef int32_t host_long;
typedef uint32_t host_ulong;
#define swabls(x) swab32s(x)
#else
typedef int64_t host_long;
typedef uint64_t host_ulong;
#define swabls(x) swab64s(x)
#endif

#include "thunk.h"

/* all dynamically generated functions begin with this code */
#define OP_PREFIX "op_"

int elf_must_swap(struct elfhdr *h)
{
  union {
      uint32_t i;
      uint8_t b[4];
  } swaptest;

  swaptest.i = 1;
  return (h->e_ident[EI_DATA] == ELFDATA2MSB) != 
      (swaptest.b[0] == 0);
}
  
void swab16s(uint16_t *p)
{
    *p = bswap16(*p);
}

void swab32s(uint32_t *p)
{
    *p = bswap32(*p);
}

void swab64s(uint64_t *p)
{
    *p = bswap64(*p);
}

void elf_swap_ehdr(struct elfhdr *h)
{
    swab16s(&h->e_type);			/* Object file type */
    swab16s(&h->	e_machine);		/* Architecture */
    swab32s(&h->	e_version);		/* Object file version */
    swabls(&h->	e_entry);		/* Entry point virtual address */
    swabls(&h->	e_phoff);		/* Program header table file offset */
    swabls(&h->	e_shoff);		/* Section header table file offset */
    swab32s(&h->	e_flags);		/* Processor-specific flags */
    swab16s(&h->	e_ehsize);		/* ELF header size in bytes */
    swab16s(&h->	e_phentsize);		/* Program header table entry size */
    swab16s(&h->	e_phnum);		/* Program header table entry count */
    swab16s(&h->	e_shentsize);		/* Section header table entry size */
    swab16s(&h->	e_shnum);		/* Section header table entry count */
    swab16s(&h->	e_shstrndx);		/* Section header string table index */
}

void elf_swap_shdr(struct elf_shdr *h)
{
  swab32s(&h->	sh_name);		/* Section name (string tbl index) */
  swab32s(&h->	sh_type);		/* Section type */
  swabls(&h->	sh_flags);		/* Section flags */
  swabls(&h->	sh_addr);		/* Section virtual addr at execution */
  swabls(&h->	sh_offset);		/* Section file offset */
  swabls(&h->	sh_size);		/* Section size in bytes */
  swab32s(&h->	sh_link);		/* Link to another section */
  swab32s(&h->	sh_info);		/* Additional section information */
  swabls(&h->	sh_addralign);		/* Section alignment */
  swabls(&h->	sh_entsize);		/* Entry size if section holds table */
}

void elf_swap_phdr(struct elf_phdr *h)
{
    swab32s(&h->p_type);			/* Segment type */
    swabls(&h->p_offset);		/* Segment file offset */
    swabls(&h->p_vaddr);		/* Segment virtual address */
    swabls(&h->p_paddr);		/* Segment physical address */
    swabls(&h->p_filesz);		/* Segment size in file */
    swabls(&h->p_memsz);		/* Segment size in memory */
    swab32s(&h->p_flags);		/* Segment flags */
    swabls(&h->p_align);		/* Segment alignment */
}

int do_swap;

uint16_t get16(uint16_t *p)
{
    uint16_t val;
    val = *p;
    if (do_swap)
        val = bswap16(val);
    return val;
}

uint32_t get32(uint32_t *p)
{
    uint32_t val;
    val = *p;
    if (do_swap)
        val = bswap32(val);
    return val;
}

void put16(uint16_t *p, uint16_t val)
{
    if (do_swap)
        val = bswap16(val);
    *p = val;
}

void put32(uint32_t *p, uint32_t val)
{
    if (do_swap)
        val = bswap32(val);
    *p = val;
}

void __attribute__((noreturn)) __attribute__((format (printf, 1, 2))) error(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "dyngen: ");
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    va_end(ap);
    exit(1);
}


struct elf_shdr *find_elf_section(struct elf_shdr *shdr, int shnum, const char *shstr, 
                                  const char *name)
{
    int i;
    const char *shname;
    struct elf_shdr *sec;

    for(i = 0; i < shnum; i++) {
        sec = &shdr[i];
        if (!sec->sh_name)
            continue;
        shname = shstr + sec->sh_name;
        if (!strcmp(shname, name))
            return sec;
    }
    return NULL;
}

void *load_data(int fd, long offset, unsigned int size)
{
    char *data;

    data = malloc(size);
    if (!data)
        return NULL;
    lseek(fd, offset, SEEK_SET);
    if (read(fd, data, size) != size) {
        free(data);
        return NULL;
    }
    return data;
}

int strstart(const char *str, const char *val, const char **ptr)
{
    const char *p, *q;
    p = str;
    q = val;
    while (*q != '\0') {
        if (*p != *q)
            return 0;
        p++;
        q++;
    }
    if (ptr)
        *ptr = p;
    return 1;
}

#define MAX_ARGS 3

/* generate op code */
void gen_code(const char *name, host_ulong offset, host_ulong size, 
              FILE *outfile, uint8_t *text, ELF_RELOC *relocs, int nb_relocs, int reloc_sh_type,
              ElfW(Sym) *symtab, char *strtab, int gen_switch)
{
    int copy_size = 0;
    uint8_t *p_start, *p_end;
    int nb_args, i, n;
    uint8_t args_present[MAX_ARGS];
    const char *sym_name, *p;
    ELF_RELOC *rel;

    /* compute exact size excluding return instruction */
    p_start = text + offset;
    p_end = p_start + size;
    switch(ELF_ARCH) {
    case EM_386:
        {
            uint8_t *p;
            p = p_end - 1;
            if (p == p_start)
                error("empty code for %s", name);
            if (p[0] != 0xc3)
                error("ret expected at the end of %s", name);
            copy_size = p - p_start;
        }
        break;
    case EM_PPC:
        {
            uint8_t *p;
            p = (void *)(p_end - 4);
            if (p == p_start)
                error("empty code for %s", name);
            if (get32((uint32_t *)p) != 0x4e800020)
                error("blr expected at the end of %s", name);
            copy_size = p - p_start;
        }
        break;
    case EM_S390:
	{
	    uint8_t *p;
	    p = (void *)(p_end - 2);
	    if (p == p_start)
		error("empty code for %s", name);
	    if (get16((uint16_t *)p) != 0x07fe && get16((uint16_t *)p) != 0x07f4)
		error("br %%r14 expected at the end of %s", name);
	    copy_size = p - p_start;
	}
        break;
    case EM_ALPHA:
        {
	    uint8_t *p;
	    p = p_end - 4;
	    if (p == p_start)
		error("empty code for %s", name);
            if (get32((uint32_t *)p) != 0x6bfa8001)
		error("ret expected at the end of %s", name);
	    copy_size = p - p_start;	    
	}
	break;
    case EM_IA_64:
	{
            uint8_t *p;
            p = (void *)(p_end - 4);
            if (p == p_start)
                error("empty code for %s", name);
	    /* br.ret.sptk.many b0;; */
	    /* 08 00 84 00 */
            if (get32((uint32_t *)p) != 0x00840008)
                error("br.ret.sptk.many b0;; expected at the end of %s", name);
            copy_size = p - p_start;
	}
        break;
    case EM_SPARC:
    case EM_SPARC32PLUS:
	{
            uint8_t *p;
            p = (void *)(p_end - 8);
            if (p <= p_start)
                error("empty code for %s", name);
	    if (get32((uint32_t *)(p_start + 0x0)) != 0x9de3bf98)
                error("save %%sp,-104,%%sp expected at the start of %s "
		      "found [%08x]",
		      name, get32((uint32_t *)(p_start + 0x0)));
            if (get32((uint32_t *)(p + 0x0)) != 0x81c7e008 ||
		get32((uint32_t *)(p + 0x4)) != 0x81e80000)
                error("ret; restore; expected at the end of %s found [%08x:%08x]",
		      name,
		      get32((uint32_t *)(p + 0x0)),
		      get32((uint32_t *)(p + 0x4)));

            copy_size = p - p_start;
	}
	break;
    case EM_SPARCV9:
	{
            uint8_t *p;
            p = (void *)(p_end - 8);
            if (p <= p_start)
                error("empty code for %s", name);
	    if (get32((uint32_t *)(p_start + 0x0)) != 0x9de3bf40)
                error("save %%sp,-192,%%sp expected at the start of %s "
		      "found [%08x]",
		      name, get32((uint32_t *)(p_start + 0x0)));
            if (get32((uint32_t *)(p + 0x0)) != 0x81cfe008 ||
		get32((uint32_t *)(p + 0x4)) != 0x01000000)
                error("rett %%i7+8; nop; expected at the end of %s "
		      "found [%08x:%08x]",
		      name,
		      get32((uint32_t *)(p + 0x0)),
		      get32((uint32_t *)(p + 0x4)));
            copy_size = p - p_start;
	}
	break;
    default:
	error("unknown ELF architecture");
    }

    /* compute the number of arguments by looking at the relocations */
    for(i = 0;i < MAX_ARGS; i++)
        args_present[i] = 0;

    for(i = 0, rel = relocs;i < nb_relocs; i++, rel++) {
        if (rel->r_offset >= offset && rel->r_offset < offset + copy_size) {
            sym_name = strtab + symtab[ELFW(R_SYM)(rel->r_info)].st_name;
            if (strstart(sym_name, "__op_param", &p)) {
                n = strtoul(p, NULL, 10);
                if (n >= MAX_ARGS)
                    error("too many arguments in %s", name);
                args_present[n - 1] = 1;
            }
        }
    }
    
    nb_args = 0;
    while (nb_args < MAX_ARGS && args_present[nb_args])
        nb_args++;
    for(i = nb_args; i < MAX_ARGS; i++) {
        if (args_present[i])
            error("inconsistent argument numbering in %s", name);
    }

    if (gen_switch == 2) {
        fprintf(outfile, "DEF(%s, %d)\n", name + 3, nb_args);
    } else if (gen_switch == 1) {

        /* output C code */
        fprintf(outfile, "case INDEX_%s: {\n", name);
        if (nb_args > 0) {
            fprintf(outfile, "    long ");
            for(i = 0; i < nb_args; i++) {
                if (i != 0)
                    fprintf(outfile, ", ");
                fprintf(outfile, "param%d", i + 1);
            }
            fprintf(outfile, ";\n");
        }
        fprintf(outfile, "    extern void %s();\n", name);

        for(i = 0, rel = relocs;i < nb_relocs; i++, rel++) {
            if (rel->r_offset >= offset && rel->r_offset < offset + copy_size) {
                sym_name = strtab + symtab[ELFW(R_SYM)(rel->r_info)].st_name;
                if (*sym_name && !strstart(sym_name, "__op_param", &p)) {
#if defined(HOST_SPARC)
		    if (sym_name[0] == '.') {
			fprintf(outfile,
				"extern char __dot_%s __asm__(\"%s\");\n",
				sym_name+1, sym_name);
			continue;
		    }
#endif
                    fprintf(outfile, "extern char %s;\n", sym_name);
                }
            }
        }

        fprintf(outfile, "    memcpy(gen_code_ptr, &%s, %d);\n", name, copy_size);
        for(i = 0; i < nb_args; i++) {
            fprintf(outfile, "    param%d = *opparam_ptr++;\n", i + 1);
        }

        /* patch relocations */
#if defined(HOST_I386)
            {
                char name[256];
                int type;
                int addend;
                for(i = 0, rel = relocs;i < nb_relocs; i++, rel++) {
                if (rel->r_offset >= offset && rel->r_offset < offset + copy_size) {
                    sym_name = strtab + symtab[ELFW(R_SYM)(rel->r_info)].st_name;
                    if (strstart(sym_name, "__op_param", &p)) {
                        snprintf(name, sizeof(name), "param%s", p);
                    } else {
                        snprintf(name, sizeof(name), "(long)(&%s)", sym_name);
                    }
                    type = ELF32_R_TYPE(rel->r_info);
                    addend = get32((uint32_t *)(text + rel->r_offset));
                    switch(type) {
                    case R_386_32:
                        fprintf(outfile, "    *(uint32_t *)(gen_code_ptr + %d) = %s + %d;\n", 
                                rel->r_offset - offset, name, addend);
                        break;
                    case R_386_PC32:
                        fprintf(outfile, "    *(uint32_t *)(gen_code_ptr + %d) = %s - (long)(gen_code_ptr + %d) + %d;\n", 
                                rel->r_offset - offset, name, rel->r_offset - offset, addend);
                        break;
                    default:
                        error("unsupported i386 relocation (%d)", type);
                    }
                }
                }
            }
#elif defined(HOST_PPC)
            {
                char name[256];
                int type;
                int addend;
                for(i = 0, rel = relocs;i < nb_relocs; i++, rel++) {
                    if (rel->r_offset >= offset && rel->r_offset < offset + copy_size) {
                        sym_name = strtab + symtab[ELFW(R_SYM)(rel->r_info)].st_name;
                        if (strstart(sym_name, "__op_param", &p)) {
                            snprintf(name, sizeof(name), "param%s", p);
                        } else {
                            snprintf(name, sizeof(name), "(long)(&%s)", sym_name);
                        }
                        type = ELF32_R_TYPE(rel->r_info);
                        addend = rel->r_addend;
                        switch(type) {
                        case R_PPC_ADDR32:
                            fprintf(outfile, "    *(uint32_t *)(gen_code_ptr + %d) = %s + %d;\n", 
                                    rel->r_offset - offset, name, addend);
                            break;
                        case R_PPC_ADDR16_LO:
                            fprintf(outfile, "    *(uint16_t *)(gen_code_ptr + %d) = (%s + %d);\n", 
                                    rel->r_offset - offset, name, addend);
                            break;
                        case R_PPC_ADDR16_HI:
                            fprintf(outfile, "    *(uint16_t *)(gen_code_ptr + %d) = (%s + %d) >> 16;\n", 
                                    rel->r_offset - offset, name, addend);
                            break;
                        case R_PPC_ADDR16_HA:
                            fprintf(outfile, "    *(uint16_t *)(gen_code_ptr + %d) = (%s + %d + 0x8000) >> 16;\n", 
                                    rel->r_offset - offset, name, addend);
                            break;
                        case R_PPC_REL24:
                            /* warning: must be at 32 MB distancy */
                            fprintf(outfile, "    *(uint32_t *)(gen_code_ptr + %d) = (*(uint32_t *)(gen_code_ptr + %d) & ~0x03fffffc) | ((%s - (long)(gen_code_ptr + %d) + %d) & 0x03fffffc);\n", 
                                    rel->r_offset - offset, rel->r_offset - offset, name, rel->r_offset - offset, addend);
                            break;
                        default:
                            error("unsupported powerpc relocation (%d)", type);
                        }
                    }
                }
            }
#elif defined(HOST_S390)
            {
                char name[256];
                int type;
                int addend;
                for(i = 0, rel = relocs;i < nb_relocs; i++, rel++) {
                    if (rel->r_offset >= offset && rel->r_offset < offset + copy_size) {
                        sym_name = strtab + symtab[ELFW(R_SYM)(rel->r_info)].st_name;
                        if (strstart(sym_name, "__op_param", &p)) {
                            snprintf(name, sizeof(name), "param%s", p);
                        } else {
                            snprintf(name, sizeof(name), "(long)(&%s)", sym_name);
                        }
                        type = ELF32_R_TYPE(rel->r_info);
                        addend = rel->r_addend;
                        switch(type) {
                        case R_390_32:
                            fprintf(outfile, "    *(uint32_t *)(gen_code_ptr + %d) = %s + %d;\n", 
                                    rel->r_offset - offset, name, addend);
                            break;
                        case R_390_16:
                            fprintf(outfile, "    *(uint16_t *)(gen_code_ptr + %d) = %s + %d;\n", 
                                    rel->r_offset - offset, name, addend);
                            break;
                        case R_390_8:
                            fprintf(outfile, "    *(uint8_t *)(gen_code_ptr + %d) = %s + %d;\n", 
                                    rel->r_offset - offset, name, addend);
                            break;
                        default:
                            error("unsupported s390 relocation (%d)", type);
                        }
                    }
                }
            }
#elif defined(HOST_ALPHA)
            {
                for (i = 0, rel = relocs; i < nb_relocs; i++, rel++) {
		    if (rel->r_offset >= offset && rel->r_offset < offset + copy_size) {
			int type;

			type = ELF64_R_TYPE(rel->r_info);
			sym_name = strtab + symtab[ELF64_R_SYM(rel->r_info)].st_name;
			switch (type) {
			case R_ALPHA_GPDISP:
			    /* The gp is just 32 bit, and never changes, so it's easiest to emit it
			       as an immediate instead of constructing it from the pv or ra.  */
			    fprintf(outfile, "    immediate_ldah(gen_code_ptr + %ld, gp);\n",
				    rel->r_offset - offset);
			    fprintf(outfile, "    immediate_lda(gen_code_ptr + %ld, gp);\n",
				    rel->r_offset - offset + rel->r_addend);
			    break;
			case R_ALPHA_LITUSE:
			    /* jsr to literal hint. Could be used to optimize to bsr. Ignore for
			       now, since some called functions (libc) need pv to be set up.  */
			    break;
			case R_ALPHA_HINT:
			    /* Branch target prediction hint. Ignore for now.  Should be already
			       correct for in-function jumps.  */
			    break;
			case R_ALPHA_LITERAL:
			    /* Load a literal from the GOT relative to the gp.  Since there's only a
			       single gp, nothing is to be done.  */
			    break;
			case R_ALPHA_GPRELHIGH:
			    /* Handle fake relocations against __op_param symbol.  Need to emit the
			       high part of the immediate value instead.  Other symbols need no
			       special treatment.  */
			    if (strstart(sym_name, "__op_param", &p))
				fprintf(outfile, "    immediate_ldah(gen_code_ptr + %ld, param%s);\n",
					rel->r_offset - offset, p);
			    break;
			case R_ALPHA_GPRELLOW:
			    if (strstart(sym_name, "__op_param", &p))
				fprintf(outfile, "    immediate_lda(gen_code_ptr + %ld, param%s);\n",
					rel->r_offset - offset, p);
			    break;
			case R_ALPHA_BRSGP:
			    /* PC-relative jump. Tweak offset to skip the two instructions that try to
			       set up the gp from the pv.  */
			    fprintf(outfile, "    fix_bsr(gen_code_ptr + %ld, (uint8_t *) &%s - (gen_code_ptr + %ld) + 4);\n",
				    rel->r_offset - offset, sym_name, rel->r_offset - offset);
			    break;
			default:
			    error("unsupported Alpha relocation (%d)", type);
			}
		    }
                }
            }
#elif defined(HOST_IA64)
            {
                char name[256];
                int type;
                int addend;
                for(i = 0, rel = relocs;i < nb_relocs; i++, rel++) {
                    if (rel->r_offset >= offset && rel->r_offset < offset + copy_size) {
                        sym_name = strtab + symtab[ELF64_R_SYM(rel->r_info)].st_name;
                        if (strstart(sym_name, "__op_param", &p)) {
                            snprintf(name, sizeof(name), "param%s", p);
                        } else {
                            snprintf(name, sizeof(name), "(long)(&%s)", sym_name);
                        }
                        type = ELF64_R_TYPE(rel->r_info);
                        addend = rel->r_addend;
                        switch(type) {
			case R_IA64_LTOFF22:
			    error("must implemnt R_IA64_LTOFF22 relocation");
			case R_IA64_PCREL21B:
			    error("must implemnt R_IA64_PCREL21B relocation");
                        default:
                            error("unsupported ia64 relocation (%d)", type);
                        }
                    }
                }
            }
#elif defined(HOST_SPARC)
            {
                char name[256];
                int type;
                int addend;
                for(i = 0, rel = relocs;i < nb_relocs; i++, rel++) {
                    if (rel->r_offset >= offset && rel->r_offset < offset + copy_size) {
                        sym_name = strtab + symtab[ELF32_R_SYM(rel->r_info)].st_name;
                        if (strstart(sym_name, "__op_param", &p)) {
                            snprintf(name, sizeof(name), "param%s", p);
                        } else {
				if (sym_name[0] == '.')
					snprintf(name, sizeof(name),
						 "(long)(&__dot_%s)",
						 sym_name + 1);
				else
					snprintf(name, sizeof(name),
						 "(long)(&%s)", sym_name);
                        }
                        type = ELF32_R_TYPE(rel->r_info);
                        addend = rel->r_addend;
                        switch(type) {
                        case R_SPARC_32:
                            fprintf(outfile, "    *(uint32_t *)(gen_code_ptr + %d) = %s + %d;\n", 
                                    rel->r_offset - offset, name, addend);
			    break;
			case R_SPARC_HI22:
                            fprintf(outfile,
				    "    *(uint32_t *)(gen_code_ptr + %d) = "
				    "((*(uint32_t *)(gen_code_ptr + %d)) "
				    " & ~0x3fffff) "
				    " | ((%s + %d) & 0x3fffff);\n",
                                    rel->r_offset - offset,
				    rel->r_offset - offset,
				    name, addend);
			    break;
			case R_SPARC_LO10:
                            fprintf(outfile,
				    "    *(uint32_t *)(gen_code_ptr + %d) = "
				    "((*(uint32_t *)(gen_code_ptr + %d)) "
				    " & ~0x3ff) "
				    " | ((%s + %d) & 0x3ff);\n",
                                    rel->r_offset - offset,
				    rel->r_offset - offset,
				    name, addend);
			    break;
			case R_SPARC_WDISP30:
			    fprintf(outfile,
				    "    *(uint32_t *)(gen_code_ptr + %d) = "
				    "((*(uint32_t *)(gen_code_ptr + %d)) "
				    " & ~0x3fffffff) "
				    " | ((((%s + %d) - (long)gen_code_ptr)>>2) "
				    "    & 0x3fffffff);\n",
				    rel->r_offset - offset,
				    rel->r_offset - offset,
				    name, addend);
			    break;
                        default:
                            error("unsupported sparc relocation (%d)", type);
                        }
                    }
                }
            }
#elif defined(HOST_SPARC64)
            {
                char name[256];
                int type;
                int addend;
                for(i = 0, rel = relocs;i < nb_relocs; i++, rel++) {
                    if (rel->r_offset >= offset && rel->r_offset < offset + copy_size) {
                        sym_name = strtab + symtab[ELF64_R_SYM(rel->r_info)].st_name;
                        if (strstart(sym_name, "__op_param", &p)) {
                            snprintf(name, sizeof(name), "param%s", p);
                        } else {
                            snprintf(name, sizeof(name), "(long)(&%s)", sym_name);
                        }
                        type = ELF64_R_TYPE(rel->r_info);
                        addend = rel->r_addend;
                        switch(type) {
                        case R_SPARC_32:
                            fprintf(outfile, "    *(uint32_t *)(gen_code_ptr + %d) = %s + %d;\n",
                                    rel->r_offset - offset, name, addend);
			    break;
			case R_SPARC_HI22:
                            fprintf(outfile,
				    "    *(uint32_t *)(gen_code_ptr + %d) = "
				    "((*(uint32_t *)(gen_code_ptr + %d)) "
				    " & ~0x3fffff) "
				    " | ((%s + %d) & 0x3fffff);\n",
                                    rel->r_offset - offset,
				    rel->r_offset - offset,
				    name, addend);
			    break;
			case R_SPARC_LO10:
                            fprintf(outfile,
				    "    *(uint32_t *)(gen_code_ptr + %d) = "
				    "((*(uint32_t *)(gen_code_ptr + %d)) "
				    " & ~0x3ff) "
				    " | ((%s + %d) & 0x3ff);\n",
                                    rel->r_offset - offset,
				    rel->r_offset - offset,
				    name, addend);
			    break;
			case R_SPARC_WDISP30:
			    fprintf(outfile,
				    "    *(uint32_t *)(gen_code_ptr + %d) = "
				    "((*(uint32_t *)(gen_code_ptr + %d)) "
				    " & ~0x3fffffff) "
				    " | ((((%s + %d) - (long)gen_code_ptr)>>2) "
				    "    & 0x3fffffff);\n",
				    rel->r_offset - offset,
				    rel->r_offset - offset,
				    name, addend);
			    break;
                        default:
			    error("unsupported sparc64 relocation (%d)", type);
                        }
                    }
                }
            }
#else
#error unsupported CPU
#endif
        fprintf(outfile, "    gen_code_ptr += %d;\n", copy_size);
        fprintf(outfile, "}\n");
        fprintf(outfile, "break;\n\n");
    } else {
        fprintf(outfile, "static inline void gen_%s(", name);
        if (nb_args == 0) {
            fprintf(outfile, "void");
        } else {
            for(i = 0; i < nb_args; i++) {
                if (i != 0)
                    fprintf(outfile, ", ");
                fprintf(outfile, "long param%d", i + 1);
            }
        }
        fprintf(outfile, ")\n");
        fprintf(outfile, "{\n");
        for(i = 0; i < nb_args; i++) {
            fprintf(outfile, "    *gen_opparam_ptr++ = param%d;\n", i + 1);
        }
        fprintf(outfile, "    *gen_opc_ptr++ = INDEX_%s;\n", name);
        fprintf(outfile, "}\n\n");
    }
}

/* load an elf object file */
int load_elf(const char *filename, FILE *outfile, int do_print_enum)
{
    int fd;
    struct elfhdr ehdr;
    struct elf_shdr *sec, *shdr, *symtab_sec, *strtab_sec, *text_sec;
    int i, j, nb_syms;
    ElfW(Sym) *symtab, *sym;
    char *shstr, *strtab;
    uint8_t *text;
    void *relocs;
    int nb_relocs, reloc_sh_type;
    
    fd = open(filename, O_RDONLY);
    if (fd < 0) 
        error("can't open file '%s'", filename);
    
    /* Read ELF header.  */
    if (read(fd, &ehdr, sizeof (ehdr)) != sizeof (ehdr))
        error("unable to read file header");

    /* Check ELF identification.  */
    if (ehdr.e_ident[EI_MAG0] != ELFMAG0
     || ehdr.e_ident[EI_MAG1] != ELFMAG1
     || ehdr.e_ident[EI_MAG2] != ELFMAG2
     || ehdr.e_ident[EI_MAG3] != ELFMAG3
     || ehdr.e_ident[EI_VERSION] != EV_CURRENT) {
        error("bad ELF header");
    }

    do_swap = elf_must_swap(&ehdr);
    if (do_swap)
        elf_swap_ehdr(&ehdr);
    if (ehdr.e_ident[EI_CLASS] != ELF_CLASS)
        error("Unsupported ELF class");
    if (ehdr.e_type != ET_REL)
        error("ELF object file expected");
    if (ehdr.e_version != EV_CURRENT)
        error("Invalid ELF version");
    if (!elf_check_arch(ehdr.e_machine))
        error("Unsupported CPU (e_machine=%d)", ehdr.e_machine);

    /* read section headers */
    shdr = load_data(fd, ehdr.e_shoff, ehdr.e_shnum * sizeof(struct elf_shdr));
    if (do_swap) {
        for(i = 0; i < ehdr.e_shnum; i++) {
            elf_swap_shdr(&shdr[i]);
        }
    }

    sec = &shdr[ehdr.e_shstrndx];
    shstr = load_data(fd, sec->sh_offset, sec->sh_size);

    /* text section */

    text_sec = find_elf_section(shdr, ehdr.e_shnum, shstr, ".text");
    if (!text_sec)
        error("could not find .text section");
    text = load_data(fd, text_sec->sh_offset, text_sec->sh_size);

    /* find text relocations, if any */
    nb_relocs = 0;
    relocs = NULL;
    reloc_sh_type = 0;
    for(i = 0; i < ehdr.e_shnum; i++) {
        sec = &shdr[i];
        if ((sec->sh_type == SHT_REL || sec->sh_type == SHT_RELA) &&
            sec->sh_info == (text_sec - shdr)) {
            reloc_sh_type = sec->sh_type;
            relocs = load_data(fd, sec->sh_offset, sec->sh_size);
            nb_relocs = sec->sh_size / sec->sh_entsize;
            if (do_swap) {
                if (sec->sh_type == SHT_REL) {
                    ElfW(Rel) *rel = relocs;
                    for(j = 0, rel = relocs; j < nb_relocs; j++, rel++) {
                        swabls(&rel->r_offset);
                        swabls(&rel->r_info);
                    }
                } else {
                    ElfW(Rela) *rel = relocs;
                    for(j = 0, rel = relocs; j < nb_relocs; j++, rel++) {
                        swabls(&rel->r_offset);
                        swabls(&rel->r_info);
                        swabls(&rel->r_addend);
                    }
                }
            }
            break;
        }
    }

    symtab_sec = find_elf_section(shdr, ehdr.e_shnum, shstr, ".symtab");
    if (!symtab_sec)
        error("could not find .symtab section");
    strtab_sec = &shdr[symtab_sec->sh_link];

    symtab = load_data(fd, symtab_sec->sh_offset, symtab_sec->sh_size);
    strtab = load_data(fd, strtab_sec->sh_offset, strtab_sec->sh_size);
    
    nb_syms = symtab_sec->sh_size / sizeof(ElfW(Sym));
    if (do_swap) {
        for(i = 0, sym = symtab; i < nb_syms; i++, sym++) {
            swab32s(&sym->st_name);
            swabls(&sym->st_value);
            swabls(&sym->st_size);
            swab16s(&sym->st_shndx);
        }
    }

    if (do_print_enum) {
        fprintf(outfile, "DEF(end, 0)\n");
        for(i = 0, sym = symtab; i < nb_syms; i++, sym++) {
            const char *name, *p;
            name = strtab + sym->st_name;
            if (strstart(name, OP_PREFIX, &p)) {
                gen_code(name, sym->st_value, sym->st_size, outfile, 
                         text, relocs, nb_relocs, reloc_sh_type, symtab, strtab, 2);
            }
        }
    } else {
        /* generate big code generation switch */
#ifdef HOST_ALPHA
fprintf(outfile,
"register int gp asm(\"$29\");\n"
"static inline void immediate_ldah(void *p, int val) {\n"
"    uint32_t *dest = p;\n"
"    long high = ((val >> 16) + ((val >> 15) & 1)) & 0xffff;\n"
"\n"
"    *dest &= ~0xffff;\n"
"    *dest |= high;\n"
"    *dest |= 31 << 16;\n"
"}\n"
"static inline void immediate_lda(void *dest, int val) {\n"
"    *(uint16_t *) dest = val;\n"
"}\n"
"void fix_bsr(void *p, int offset) {\n"
"    uint32_t *dest = p;\n"
"    *dest &= ~((1 << 21) - 1);\n"
"    *dest |= (offset >> 2) & ((1 << 21) - 1);\n"
"}\n");
#endif
fprintf(outfile,
"int dyngen_code(uint8_t *gen_code_buf,\n"
"                const uint16_t *opc_buf, const uint32_t *opparam_buf)\n"
"{\n"
"    uint8_t *gen_code_ptr;\n"
"    const uint16_t *opc_ptr;\n"
"    const uint32_t *opparam_ptr;\n"
"    gen_code_ptr = gen_code_buf;\n"
"    opc_ptr = opc_buf;\n"
"    opparam_ptr = opparam_buf;\n"
"    for(;;) {\n"
"        switch(*opc_ptr++) {\n"
);

        for(i = 0, sym = symtab; i < nb_syms; i++, sym++) {
            const char *name;
            name = strtab + sym->st_name;
            if (strstart(name, OP_PREFIX, NULL)) {
#if 0
                printf("%4d: %s pos=0x%08x len=%d\n", 
                       i, name, sym->st_value, sym->st_size);
#endif
                if (sym->st_shndx != (text_sec - shdr))
                    error("invalid section for opcode (0x%x)", sym->st_shndx);
                gen_code(name, sym->st_value, sym->st_size, outfile, 
                         text, relocs, nb_relocs, reloc_sh_type, symtab, strtab, 1);
            }
        }

fprintf(outfile,
"        default:\n"
"            goto the_end;\n"
"        }\n"
"    }\n"
" the_end:\n"
);

/* generate a return */ 
    switch(ELF_ARCH) {
    case EM_386:
        fprintf(outfile, "*gen_code_ptr++ = 0xc3; /* ret */\n");
        break;
    case EM_PPC:
        fprintf(outfile, "*((uint32_t *)gen_code_ptr)++ = 0x4e800020; /* blr */\n");
        break;
    case EM_S390:
        fprintf(outfile, "*((uint16_t *)gen_code_ptr)++ = 0x07fe; /* br %%r14 */\n");
        break;
    case EM_ALPHA:
        fprintf(outfile, "*((uint32_t *)gen_code_ptr)++ = 0x6bfa8001; /* ret */\n");
        break;
    case EM_IA_64:
        fprintf(outfile, "*((uint32_t *)gen_code_ptr)++ = 0x00840008; /* br.ret.sptk.many b0;; */\n");
        break;
    case EM_SPARC:
    case EM_SPARC32PLUS:
    case EM_SPARCV9:
	/* Fill the delay slot. */
	fprintf(outfile, "*((uint32_t *)gen_code_ptr) = *((uint32_t *)gen_code_ptr - 1); /* delay slot */\n");
	fprintf(outfile, "*((uint32_t *)gen_code_ptr - 1) = 0x81c3e008; /* retl */\n");
	fprintf(outfile, "gen_code_ptr++;\n");
        break;
    default:
	error("unknown ELF architecture");
    }
    
    fprintf(outfile, "return gen_code_ptr -  gen_code_buf;\n");
    fprintf(outfile, "}\n\n");

/* generate gen_xxx functions */
/* XXX: suppress the use of these functions to simplify code */
        for(i = 0, sym = symtab; i < nb_syms; i++, sym++) {
            const char *name;
            name = strtab + sym->st_name;
            if (strstart(name, OP_PREFIX, NULL)) {
                if (sym->st_shndx != (text_sec - shdr))
                    error("invalid section for opcode (0x%x)", sym->st_shndx);
                gen_code(name, sym->st_value, sym->st_size, outfile, 
                         text, relocs, nb_relocs, reloc_sh_type, symtab, strtab, 0);
            }
        }
    }

    close(fd);
    return 0;
}

void usage(void)
{
    printf("dyngen (c) 2003 Fabrice Bellard\n"
           "usage: dyngen [-o outfile] [-c] objfile\n"
           "Generate a dynamic code generator from an object file\n"
           "-c     output enum of operations\n"
           );
    exit(1);
}

int main(int argc, char **argv)
{
    int c, do_print_enum;
    const char *filename, *outfilename;
    FILE *outfile;

    outfilename = "out.c";
    do_print_enum = 0;
    for(;;) {
        c = getopt(argc, argv, "ho:c");
        if (c == -1)
            break;
        switch(c) {
        case 'h':
            usage();
            break;
        case 'o':
            outfilename = optarg;
            break;
        case 'c':
            do_print_enum = 1;
            break;
        }
    }
    if (optind >= argc)
        usage();
    filename = argv[optind];
    outfile = fopen(outfilename, "w");
    if (!outfile)
        error("could not open '%s'", outfilename);
    load_elf(filename, outfile, do_print_enum);
    fclose(outfile);
    return 0;
}
