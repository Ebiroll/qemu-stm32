/*
 * Copyright (c) 1995 Danny Gasparovski.
 *
 * Please read the file COPYRIGHT for the
 * terms and conditions of the copyright.
 */

#ifndef MISC_H
#define MISC_H

struct gfwd_list {
	void *ex_chardev;
	struct in_addr ex_addr;		/* Server address */
	int ex_fport;                   /* Port to telnet to */
	char *ex_exec;                  /* Command line of what to exec */
	struct gfwd_list *ex_next;
};

#define EMU_NONE 0x0

/* TCP emulations */
#define EMU_CTL 0x1
#define EMU_FTP 0x2
#define EMU_KSH 0x3
#define EMU_IRC 0x4
#define EMU_REALAUDIO 0x5
#define EMU_RLOGIN 0x6
#define EMU_IDENT 0x7

#define EMU_NOCONNECT 0x10	/* Don't connect */

struct tos_t {
    uint16_t lport;
    uint16_t fport;
    uint8_t tos;
    uint8_t emu;
};

struct emu_t {
    uint16_t lport;
    uint16_t fport;
    uint8_t tos;
    uint8_t emu;
    struct emu_t *next;
};

struct slirp_quehead {
    struct slirp_quehead *qh_link;
    struct slirp_quehead *qh_rlink;
};

void slirp_insque(void *, void *);
void slirp_remque(void *);
int add_exec(struct gfwd_list **, void *, const char *, struct in_addr, int);
int fork_exec(struct socket *so, const char *ex);

#endif
