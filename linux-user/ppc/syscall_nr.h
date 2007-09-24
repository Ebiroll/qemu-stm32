/*
 * This file contains the system call numbers.
 */
#define TARGET_NR_restart_syscall          0
#define TARGET_NR_exit                     1
#define TARGET_NR_fork                     2
#define TARGET_NR_read                     3
#define TARGET_NR_write                    4
#define TARGET_NR_open                     5
#define TARGET_NR_close                    6
#define TARGET_NR_waitpid                  7
#define TARGET_NR_creat                    8
#define TARGET_NR_link                     9
#define TARGET_NR_unlink                  10
#define TARGET_NR_execve                  11
#define TARGET_NR_chdir                   12
#define TARGET_NR_time                    13
#define TARGET_NR_mknod                   14
#define TARGET_NR_chmod                   15
#define TARGET_NR_lchown32                16
#define TARGET_NR_break                   17
#define TARGET_NR_oldstat                 18
#define TARGET_NR_lseek                   19
#define TARGET_NR_getpid                  20
#define TARGET_NR_mount                   21
#define TARGET_NR_umount                  22
#define TARGET_NR_setuid32                23
#define TARGET_NR_getuid32                24
#define TARGET_NR_stime                   25
#define TARGET_NR_ptrace                  26
#define TARGET_NR_alarm                   27
#define TARGET_NR_oldfstat                28
#define TARGET_NR_pause                   29
#define TARGET_NR_utime                   30
#define TARGET_NR_stty                    31
#define TARGET_NR_gtty                    32
#define TARGET_NR_access                  33
#define TARGET_NR_nice                    34
#define TARGET_NR_ftime                   35
#define TARGET_NR_sync                    36
#define TARGET_NR_kill                    37
#define TARGET_NR_rename                  38
#define TARGET_NR_mkdir                   39
#define TARGET_NR_rmdir                   40
#define TARGET_NR_dup                     41
#define TARGET_NR_pipe                    42
#define TARGET_NR_times                   43
#define TARGET_NR_prof                    44
#define TARGET_NR_brk                     45
#define TARGET_NR_setgid32                46
#define TARGET_NR_getgid32                47
#define TARGET_NR_signal                  48
#define TARGET_NR_geteuid32               49
#define TARGET_NR_getegid32               50
#define TARGET_NR_acct                    51
#define TARGET_NR_umount2                 52
#define TARGET_NR_lock                    53
#define TARGET_NR_ioctl                   54
#define TARGET_NR_fcntl                   55
#define TARGET_NR_mpx                     56
#define TARGET_NR_setpgid                 57
#define TARGET_NR_ulimit                  58
#define TARGET_NR_oldolduname             59
#define TARGET_NR_umask                   60
#define TARGET_NR_chroot                  61
#define TARGET_NR_ustat                   62
#define TARGET_NR_dup2                    63
#define TARGET_NR_getppid                 64
#define TARGET_NR_getpgrp                 65
#define TARGET_NR_setsid                  66
#define TARGET_NR_sigaction               67
#define TARGET_NR_sgetmask                68
#define TARGET_NR_ssetmask                69
#define TARGET_NR_setreuid32              70
#define TARGET_NR_setregid32              71
#define TARGET_NR_sigsuspend              72
#define TARGET_NR_sigpending              73
#define TARGET_NR_sethostname             74
#define TARGET_NR_setrlimit               75
#define TARGET_NR_getrlimit               76
#define TARGET_NR_getrusage               77
#define TARGET_NR_gettimeofday            78
#define TARGET_NR_settimeofday            79
#define TARGET_NR_getgroups32             80
#define TARGET_NR_setgroups32             81
#define TARGET_NR_select                  82
#define TARGET_NR_symlink                 83
#define TARGET_NR_oldlstat                84
#define TARGET_NR_readlink                85
#define TARGET_NR_uselib                  86
#define TARGET_NR_swapon                  87
#define TARGET_NR_reboot                  88
#define TARGET_NR_readdir                 89
#define TARGET_NR_mmap                    90
#define TARGET_NR_munmap                  91
#define TARGET_NR_truncate                92
#define TARGET_NR_ftruncate               93
#define TARGET_NR_fchmod                  94
#define TARGET_NR_fchown32                95
#define TARGET_NR_getpriority             96
#define TARGET_NR_setpriority             97
#define TARGET_NR_profil                  98
#define TARGET_NR_statfs                  99
#define TARGET_NR_fstatfs                100
#define TARGET_NR_ioperm                 101
#define TARGET_NR_socketcall             102
#define TARGET_NR_syslog                 103
#define TARGET_NR_setitimer              104
#define TARGET_NR_getitimer              105
#define TARGET_NR_stat                   106
#define TARGET_NR_lstat                  107
#define TARGET_NR_fstat                  108
#define TARGET_NR_olduname               109
#define TARGET_NR_iopl                   110
#define TARGET_NR_vhangup                111
#define TARGET_NR_idle                   112
#define TARGET_NR_vm86                   113
#define TARGET_NR_wait4                  114
#define TARGET_NR_swapoff                115
#define TARGET_NR_sysinfo                116
#define TARGET_NR_ipc                    117
#define TARGET_NR_fsync                  118
#define TARGET_NR_sigreturn              119
#define TARGET_NR_clone                  120
#define TARGET_NR_setdomainname          121
#define TARGET_NR_uname                  122
#define TARGET_NR_modify_ldt             123
#define TARGET_NR_adjtimex               124
#define TARGET_NR_mprotect               125
#define TARGET_NR_sigprocmask            126
#define TARGET_NR_create_module          127
#define TARGET_NR_init_module            128
#define TARGET_NR_delete_module          129
#define TARGET_NR_get_kernel_syms        130
#define TARGET_NR_quotactl               131
#define TARGET_NR_getpgid                132
#define TARGET_NR_fchdir                 133
#define TARGET_NR_bdflush                134
#define TARGET_NR_sysfs                  135
#define TARGET_NR_personality            136
#define TARGET_NR_afs_syscall            137 /* Syscall for Andrew File System */
#define TARGET_NR_setfsuid32             138
#define TARGET_NR_setfsgid32             139
#define TARGET_NR__llseek                140
#define TARGET_NR_getdents               141
#define TARGET_NR__newselect             142
#define TARGET_NR_flock                  143
#define TARGET_NR_msync                  144
#define TARGET_NR_readv                  145
#define TARGET_NR_writev                 146
#define TARGET_NR_getsid                 147
#define TARGET_NR_fdatasync              148
#define TARGET_NR__sysctl                149
#define TARGET_NR_mlock                  150
#define TARGET_NR_munlock                151
#define TARGET_NR_mlockall               152
#define TARGET_NR_munlockall             153
#define TARGET_NR_sched_setparam         154
#define TARGET_NR_sched_getparam         155
#define TARGET_NR_sched_setscheduler     156
#define TARGET_NR_sched_getscheduler     157
#define TARGET_NR_sched_yield            158
#define TARGET_NR_sched_get_priority_max 159
#define TARGET_NR_sched_get_priority_min 160
#define TARGET_NR_sched_rr_get_interval  161
#define TARGET_NR_nanosleep              162
#define TARGET_NR_mremap                 163
#define TARGET_NR_setresuid32            164
#define TARGET_NR_getresuid32            165
#define TARGET_NR_query_module           166
#define TARGET_NR_poll                   167
#define TARGET_NR_nfsservctl             168
#define TARGET_NR_setresgid32            169
#define TARGET_NR_getresgid32            170
#define TARGET_NR_prctl                  171
#define TARGET_NR_rt_sigreturn           172
#define TARGET_NR_rt_sigaction           173
#define TARGET_NR_rt_sigprocmask         174
#define TARGET_NR_rt_sigpending          175
#define TARGET_NR_rt_sigtimedwait        176
#define TARGET_NR_rt_sigqueueinfo        177
#define TARGET_NR_rt_sigsuspend          178
#define TARGET_NR_pread64                179
#define TARGET_NR_pwrite64               180
#define TARGET_NR_chown32                181
#define TARGET_NR_getcwd                 182
#define TARGET_NR_capget                 183
#define TARGET_NR_capset                 184
#define TARGET_NR_sigaltstack            185
#define TARGET_NR_sendfile               186
#define TARGET_NR_getpmsg                187     /* some people actually want streams */
#define TARGET_NR_putpmsg                188     /* some people actually want streams */
#define TARGET_NR_vfork                  189
#define TARGET_NR_ugetrlimit             190     /* SuS compliant getrlimit */
#define TARGET_NR_readahead              191
#define TARGET_NR_mmap2                  192
#define TARGET_NR_truncate64             193
#define TARGET_NR_ftruncate64            194
#define TARGET_NR_stat64                 195
#define TARGET_NR_lstat64                196
#define TARGET_NR_fstat64                197
#define TARGET_NR_pciconfig_read         198
#define TARGET_NR_pciconfig_write        199
#define TARGET_NR_pciconfig_iobase       200
#define TARGET_NR_multiplexer            201
#define TARGET_NR_getdents64             202
#define TARGET_NR_pivot_root             203
#define TARGET_NR_fcntl64                204
#define TARGET_NR_madvise                205
#define TARGET_NR_mincore                206
#define TARGET_NR_gettid                 207
#define TARGET_NR_tkill                  208
#define TARGET_NR_setxattr               209
#define TARGET_NR_lsetxattr              210
#define TARGET_NR_fsetxattr              211
#define TARGET_NR_getxattr               212
#define TARGET_NR_lgetxattr              213
#define TARGET_NR_fgetxattr              214
#define TARGET_NR_listxattr              215
#define TARGET_NR_llistxattr             216
#define TARGET_NR_flistxattr             217
#define TARGET_NR_removexattr            218
#define TARGET_NR_lremovexattr           219
#define TARGET_NR_fremovexattr           220
#define TARGET_NR_futex                  221
#define TARGET_NR_sched_setaffinity      222
#define TARGET_NR_sched_getaffinity      223
/* 224 currently unused */
#define TARGET_NR_tuxcall                225
#define TARGET_NR_sendfile64             226
#define TARGET_NR_io_setup               227
#define TARGET_NR_io_destroy             228
#define TARGET_NR_io_getevents           229
#define TARGET_NR_io_submit              230
#define TARGET_NR_io_cancel              231
#define TARGET_NR_set_tid_address        232
#define TARGET_NR_fadvise64              233
#define TARGET_NR_exit_group             234
#define TARGET_NR_lookup_dcookie         235
#define TARGET_NR_epoll_create           236
#define TARGET_NR_epoll_ctl              237
#define TARGET_NR_epoll_wait             238
#define TARGET_NR_remap_file_pages       239
#define TARGET_NR_timer_create           240
#define TARGET_NR_timer_settime          241
#define TARGET_NR_timer_gettime          242
#define TARGET_NR_timer_getoverrun       243
#define TARGET_NR_timer_delete           244
#define TARGET_NR_clock_settime          245
#define TARGET_NR_clock_gettime          246
#define TARGET_NR_clock_getres           247
#define TARGET_NR_clock_nanosleep        248
#define TARGET_NR_swapcontext            249
#define TARGET_NR_tgkill                 250
#define TARGET_NR_utimes                 251
#define TARGET_NR_statfs64               252
#define TARGET_NR_fstatfs64              253
#define TARGET_NR_fadvise64_64           254
#define TARGET_NR_rtas		255
#define TARGET_NR_sys_debug_setcontext 256
/* Number 257 is reserved for vserver */
#define TARGET_NR_migrate_pages	258
#define TARGET_NR_mbind		259
#define TARGET_NR_get_mempolicy	260
#define TARGET_NR_set_mempolicy	261
#define TARGET_NR_mq_open		262
#define TARGET_NR_mq_unlink		263
#define TARGET_NR_mq_timedsend	264
#define TARGET_NR_mq_timedreceive	265
#define TARGET_NR_mq_notify		266
#define TARGET_NR_mq_getsetattr	267
#define TARGET_NR_kexec_load		268
#define TARGET_NR_add_key		269
#define TARGET_NR_request_key	270
#define TARGET_NR_keyctl		271
#define TARGET_NR_waitid		272
#define TARGET_NR_ioprio_set		273
#define TARGET_NR_ioprio_get		274
#define TARGET_NR_inotify_init	275
#define TARGET_NR_inotify_add_watch	276
#define TARGET_NR_inotify_rm_watch	277
#define TARGET_NR_spu_run		278
#define TARGET_NR_spu_create		279
#define TARGET_NR_pselect6		280
#define TARGET_NR_ppoll		281
#define TARGET_NR_unshare		282
#define TARGET_NR_splice		283
#define TARGET_NR_tee		284
#define TARGET_NR_vmsplice		285
#define TARGET_NR_openat		286
#define TARGET_NR_mkdirat		287
#define TARGET_NR_mknodat		288
#define TARGET_NR_fchownat		289
#define TARGET_NR_futimesat		290
#define TARGET_NR_fstatat64		291
#define TARGET_NR_unlinkat		292
#define TARGET_NR_renameat		293
#define TARGET_NR_linkat		294
#define TARGET_NR_symlinkat		295
#define TARGET_NR_readlinkat		296
#define TARGET_NR_fchmodat		297
#define TARGET_NR_faccessat		298
#define TARGET_NR_get_robust_list	299
#define TARGET_NR_set_robust_list	300
#define TARGET_NR_move_pages		301
#define TARGET_NR_getcpu		302
#define TARGET_NR_epoll_pwait	303
#define TARGET_NR_utimensat		304
#define TARGET_NR_signalfd		305
#define TARGET_NR_timerfd		306
#define TARGET_NR_eventfd		307
#define TARGET_NR_sync_file_range2	308
#define TARGET_NR_fallocate		309
