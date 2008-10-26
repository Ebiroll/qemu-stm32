/*
 * System call numbers.
 *
 * $FreeBSD: src/sys/sys/syscall.h,v 1.224 2008/08/24 21:23:08 rwatson Exp $
 * created from FreeBSD: head/sys/kern/syscalls.master 182123 2008-08-24 21:20:35Z rwatson
 */

#define TARGET_FREEBSD_NR_syscall     0
#define TARGET_FREEBSD_NR_exit        1
#define TARGET_FREEBSD_NR_fork        2
#define TARGET_FREEBSD_NR_read        3
#define TARGET_FREEBSD_NR_write       4
#define TARGET_FREEBSD_NR_open        5
#define TARGET_FREEBSD_NR_close       6
#define TARGET_FREEBSD_NR_wait4       7
#define TARGET_FREEBSD_NR_link        9
#define TARGET_FREEBSD_NR_unlink      10
#define TARGET_FREEBSD_NR_chdir       12
#define TARGET_FREEBSD_NR_fchdir      13
#define TARGET_FREEBSD_NR_mknod       14
#define TARGET_FREEBSD_NR_chmod       15
#define TARGET_FREEBSD_NR_chown       16
#define TARGET_FREEBSD_NR_break       17
#define TARGET_FREEBSD_NR_freebsd4_getfsstat  18
#define TARGET_FREEBSD_NR_getpid      20
#define TARGET_FREEBSD_NR_mount       21
#define TARGET_FREEBSD_NR_unmount     22
#define TARGET_FREEBSD_NR_setuid      23
#define TARGET_FREEBSD_NR_getuid      24
#define TARGET_FREEBSD_NR_geteuid     25
#define TARGET_FREEBSD_NR_ptrace      26
#define TARGET_FREEBSD_NR_recvmsg     27
#define TARGET_FREEBSD_NR_sendmsg     28
#define TARGET_FREEBSD_NR_recvfrom    29
#define TARGET_FREEBSD_NR_accept      30
#define TARGET_FREEBSD_NR_getpeername 31
#define TARGET_FREEBSD_NR_getsockname 32
#define TARGET_FREEBSD_NR_access      33
#define TARGET_FREEBSD_NR_chflags     34
#define TARGET_FREEBSD_NR_fchflags    35
#define TARGET_FREEBSD_NR_sync        36
#define TARGET_FREEBSD_NR_kill        37
#define TARGET_FREEBSD_NR_getppid     39
#define TARGET_FREEBSD_NR_dup 41
#define TARGET_FREEBSD_NR_pipe        42
#define TARGET_FREEBSD_NR_getegid     43
#define TARGET_FREEBSD_NR_profil      44
#define TARGET_FREEBSD_NR_ktrace      45
#define TARGET_FREEBSD_NR_getgid      47
#define TARGET_FREEBSD_NR_getlogin    49
#define TARGET_FREEBSD_NR_setlogin    50
#define TARGET_FREEBSD_NR_acct        51
#define TARGET_FREEBSD_NR_sigaltstack 53
#define TARGET_FREEBSD_NR_ioctl       54
#define TARGET_FREEBSD_NR_reboot      55
#define TARGET_FREEBSD_NR_revoke      56
#define TARGET_FREEBSD_NR_symlink     57
#define TARGET_FREEBSD_NR_readlink    58
#define TARGET_FREEBSD_NR_execve      59
#define TARGET_FREEBSD_NR_umask       60
#define TARGET_FREEBSD_NR_chroot      61
#define TARGET_FREEBSD_NR_msync       65
#define TARGET_FREEBSD_NR_vfork       66
#define TARGET_FREEBSD_NR_sbrk        69
#define TARGET_FREEBSD_NR_sstk        70
#define TARGET_FREEBSD_NR_vadvise     72
#define TARGET_FREEBSD_NR_munmap      73
#define TARGET_FREEBSD_NR_mprotect    74
#define TARGET_FREEBSD_NR_madvise     75
#define TARGET_FREEBSD_NR_mincore     78
#define TARGET_FREEBSD_NR_getgroups   79
#define TARGET_FREEBSD_NR_setgroups   80
#define TARGET_FREEBSD_NR_getpgrp     81
#define TARGET_FREEBSD_NR_setpgid     82
#define TARGET_FREEBSD_NR_setitimer   83
#define TARGET_FREEBSD_NR_swapon      85
#define TARGET_FREEBSD_NR_getitimer   86
#define TARGET_FREEBSD_NR_getdtablesize       89
#define TARGET_FREEBSD_NR_dup2        90
#define TARGET_FREEBSD_NR_fcntl       92
#define TARGET_FREEBSD_NR_select      93
#define TARGET_FREEBSD_NR_fsync       95
#define TARGET_FREEBSD_NR_setpriority 96
#define TARGET_FREEBSD_NR_socket      97
#define TARGET_FREEBSD_NR_connect     98
#define TARGET_FREEBSD_NR_getpriority 100
#define TARGET_FREEBSD_NR_bind        104
#define TARGET_FREEBSD_NR_setsockopt  105
#define TARGET_FREEBSD_NR_listen      106
#define TARGET_FREEBSD_NR_gettimeofday        116
#define TARGET_FREEBSD_NR_getrusage   117
#define TARGET_FREEBSD_NR_getsockopt  118
#define TARGET_FREEBSD_NR_readv       120
#define TARGET_FREEBSD_NR_writev      121
#define TARGET_FREEBSD_NR_settimeofday        122
#define TARGET_FREEBSD_NR_fchown      123
#define TARGET_FREEBSD_NR_fchmod      124
#define TARGET_FREEBSD_NR_setreuid    126
#define TARGET_FREEBSD_NR_setregid    127
#define TARGET_FREEBSD_NR_rename      128
#define TARGET_FREEBSD_NR_flock       131
#define TARGET_FREEBSD_NR_mkfifo      132
#define TARGET_FREEBSD_NR_sendto      133
#define TARGET_FREEBSD_NR_shutdown    134
#define TARGET_FREEBSD_NR_socketpair  135
#define TARGET_FREEBSD_NR_mkdir       136
#define TARGET_FREEBSD_NR_rmdir       137
#define TARGET_FREEBSD_NR_utimes      138
#define TARGET_FREEBSD_NR_adjtime     140
#define TARGET_FREEBSD_NR_setsid      147
#define TARGET_FREEBSD_NR_quotactl    148
#define TARGET_FREEBSD_NR_nlm_syscall 154
#define TARGET_FREEBSD_NR_nfssvc      155
#define TARGET_FREEBSD_NR_freebsd4_statfs     157
#define TARGET_FREEBSD_NR_freebsd4_fstatfs    158
#define TARGET_FREEBSD_NR_lgetfh      160
#define TARGET_FREEBSD_NR_getfh       161
#define TARGET_FREEBSD_NR_getdomainname       162
#define TARGET_FREEBSD_NR_setdomainname       163
#define TARGET_FREEBSD_NR_uname       164
#define TARGET_FREEBSD_NR_sysarch     165
#define TARGET_FREEBSD_NR_rtprio      166
#define TARGET_FREEBSD_NR_semsys      169
#define TARGET_FREEBSD_NR_msgsys      170
#define TARGET_FREEBSD_NR_shmsys      171
#define TARGET_FREEBSD_NR_freebsd6_pread      173
#define TARGET_FREEBSD_NR_freebsd6_pwrite     174
#define TARGET_FREEBSD_NR_setfib      175
#define TARGET_FREEBSD_NR_ntp_adjtime 176
#define TARGET_FREEBSD_NR_setgid      181
#define TARGET_FREEBSD_NR_setegid     182
#define TARGET_FREEBSD_NR_seteuid     183
#define TARGET_FREEBSD_NR_stat        188
#define TARGET_FREEBSD_NR_fstat       189
#define TARGET_FREEBSD_NR_lstat       190
#define TARGET_FREEBSD_NR_pathconf    191
#define TARGET_FREEBSD_NR_fpathconf   192
#define TARGET_FREEBSD_NR_getrlimit   194
#define TARGET_FREEBSD_NR_setrlimit   195
#define TARGET_FREEBSD_NR_getdirentries       196
#define TARGET_FREEBSD_NR_freebsd6_mmap       197
#define TARGET_FREEBSD_NR___syscall   198
#define TARGET_FREEBSD_NR_freebsd6_lseek      199
#define TARGET_FREEBSD_NR_freebsd6_truncate   200
#define TARGET_FREEBSD_NR_freebsd6_ftruncate  201
#define TARGET_FREEBSD_NR___sysctl    202
#define TARGET_FREEBSD_NR_mlock       203
#define TARGET_FREEBSD_NR_munlock     204
#define TARGET_FREEBSD_NR_undelete    205
#define TARGET_FREEBSD_NR_futimes     206
#define TARGET_FREEBSD_NR_getpgid     207
#define TARGET_FREEBSD_NR_poll        209
#define TARGET_FREEBSD_NR___semctl    220
#define TARGET_FREEBSD_NR_semget      221
#define TARGET_FREEBSD_NR_semop       222
#define TARGET_FREEBSD_NR_msgctl      224
#define TARGET_FREEBSD_NR_msgget      225
#define TARGET_FREEBSD_NR_msgsnd      226
#define TARGET_FREEBSD_NR_msgrcv      227
#define TARGET_FREEBSD_NR_shmat       228
#define TARGET_FREEBSD_NR_shmctl      229
#define TARGET_FREEBSD_NR_shmdt       230
#define TARGET_FREEBSD_NR_shmget      231
#define TARGET_FREEBSD_NR_clock_gettime       232
#define TARGET_FREEBSD_NR_clock_settime       233
#define TARGET_FREEBSD_NR_clock_getres        234
#define TARGET_FREEBSD_NR_ktimer_create       235
#define TARGET_FREEBSD_NR_ktimer_delete       236
#define TARGET_FREEBSD_NR_ktimer_settime      237
#define TARGET_FREEBSD_NR_ktimer_gettime      238
#define TARGET_FREEBSD_NR_ktimer_getoverrun   239
#define TARGET_FREEBSD_NR_nanosleep   240
#define TARGET_FREEBSD_NR_ntp_gettime 248
#define TARGET_FREEBSD_NR_minherit    250
#define TARGET_FREEBSD_NR_rfork       251
#define TARGET_FREEBSD_NR_openbsd_poll        252
#define TARGET_FREEBSD_NR_issetugid   253
#define TARGET_FREEBSD_NR_lchown      254
#define TARGET_FREEBSD_NR_aio_read    255
#define TARGET_FREEBSD_NR_aio_write   256
#define TARGET_FREEBSD_NR_lio_listio  257
#define TARGET_FREEBSD_NR_getdents    272
#define TARGET_FREEBSD_NR_lchmod      274
#define TARGET_FREEBSD_NR_netbsd_lchown       275
#define TARGET_FREEBSD_NR_lutimes     276
#define TARGET_FREEBSD_NR_netbsd_msync        277
#define TARGET_FREEBSD_NR_nstat       278
#define TARGET_FREEBSD_NR_nfstat      279
#define TARGET_FREEBSD_NR_nlstat      280
#define TARGET_FREEBSD_NR_preadv      289
#define TARGET_FREEBSD_NR_pwritev     290
#define TARGET_FREEBSD_NR_freebsd4_fhstatfs   297
#define TARGET_FREEBSD_NR_fhopen      298
#define TARGET_FREEBSD_NR_fhstat      299
#define TARGET_FREEBSD_NR_modnext     300
#define TARGET_FREEBSD_NR_modstat     301
#define TARGET_FREEBSD_NR_modfnext    302
#define TARGET_FREEBSD_NR_modfind     303
#define TARGET_FREEBSD_NR_kldload     304
#define TARGET_FREEBSD_NR_kldunload   305
#define TARGET_FREEBSD_NR_kldfind     306
#define TARGET_FREEBSD_NR_kldnext     307
#define TARGET_FREEBSD_NR_kldstat     308
#define TARGET_FREEBSD_NR_kldfirstmod 309
#define TARGET_FREEBSD_NR_getsid      310
#define TARGET_FREEBSD_NR_setresuid   311
#define TARGET_FREEBSD_NR_setresgid   312
#define TARGET_FREEBSD_NR_aio_return  314
#define TARGET_FREEBSD_NR_aio_suspend 315
#define TARGET_FREEBSD_NR_aio_cancel  316
#define TARGET_FREEBSD_NR_aio_error   317
#define TARGET_FREEBSD_NR_oaio_read   318
#define TARGET_FREEBSD_NR_oaio_write  319
#define TARGET_FREEBSD_NR_olio_listio 320
#define TARGET_FREEBSD_NR_yield       321
#define TARGET_FREEBSD_NR_mlockall    324
#define TARGET_FREEBSD_NR_munlockall  325
#define TARGET_FREEBSD_NR___getcwd    326
#define TARGET_FREEBSD_NR_sched_setparam      327
#define TARGET_FREEBSD_NR_sched_getparam      328
#define TARGET_FREEBSD_NR_sched_setscheduler  329
#define TARGET_FREEBSD_NR_sched_getscheduler  330
#define TARGET_FREEBSD_NR_sched_yield 331
#define TARGET_FREEBSD_NR_sched_get_priority_max      332
#define TARGET_FREEBSD_NR_sched_get_priority_min      333
#define TARGET_FREEBSD_NR_sched_rr_get_interval       334
#define TARGET_FREEBSD_NR_utrace      335
#define TARGET_FREEBSD_NR_freebsd4_sendfile   336
#define TARGET_FREEBSD_NR_kldsym      337
#define TARGET_FREEBSD_NR_jail        338
#define TARGET_FREEBSD_NR_sigprocmask 340
#define TARGET_FREEBSD_NR_sigsuspend  341
#define TARGET_FREEBSD_NR_freebsd4_sigaction  342
#define TARGET_FREEBSD_NR_sigpending  343
#define TARGET_FREEBSD_NR_freebsd4_sigreturn  344
#define TARGET_FREEBSD_NR_sigtimedwait        345
#define TARGET_FREEBSD_NR_sigwaitinfo 346
#define TARGET_FREEBSD_NR___acl_get_file      347
#define TARGET_FREEBSD_NR___acl_set_file      348
#define TARGET_FREEBSD_NR___acl_get_fd        349
#define TARGET_FREEBSD_NR___acl_set_fd        350
#define TARGET_FREEBSD_NR___acl_delete_file   351
#define TARGET_FREEBSD_NR___acl_delete_fd     352
#define TARGET_FREEBSD_NR___acl_aclcheck_file 353
#define TARGET_FREEBSD_NR___acl_aclcheck_fd   354
#define TARGET_FREEBSD_NR_extattrctl  355
#define TARGET_FREEBSD_NR_extattr_set_file    356
#define TARGET_FREEBSD_NR_extattr_get_file    357
#define TARGET_FREEBSD_NR_extattr_delete_file 358
#define TARGET_FREEBSD_NR_aio_waitcomplete    359
#define TARGET_FREEBSD_NR_getresuid   360
#define TARGET_FREEBSD_NR_getresgid   361
#define TARGET_FREEBSD_NR_kqueue      362
#define TARGET_FREEBSD_NR_kevent      363
#define TARGET_FREEBSD_NR_extattr_set_fd      371
#define TARGET_FREEBSD_NR_extattr_get_fd      372
#define TARGET_FREEBSD_NR_extattr_delete_fd   373
#define TARGET_FREEBSD_NR___setugid   374
#define TARGET_FREEBSD_NR_nfsclnt     375
#define TARGET_FREEBSD_NR_eaccess     376
#define TARGET_FREEBSD_NR_nmount      378
#define TARGET_FREEBSD_NR___mac_get_proc      384
#define TARGET_FREEBSD_NR___mac_set_proc      385
#define TARGET_FREEBSD_NR___mac_get_fd        386
#define TARGET_FREEBSD_NR___mac_get_file      387
#define TARGET_FREEBSD_NR___mac_set_fd        388
#define TARGET_FREEBSD_NR___mac_set_file      389
#define TARGET_FREEBSD_NR_kenv        390
#define TARGET_FREEBSD_NR_lchflags    391
#define TARGET_FREEBSD_NR_uuidgen     392
#define TARGET_FREEBSD_NR_sendfile    393
#define TARGET_FREEBSD_NR_mac_syscall 394
#define TARGET_FREEBSD_NR_getfsstat   395
#define TARGET_FREEBSD_NR_statfs      396
#define TARGET_FREEBSD_NR_fstatfs     397
#define TARGET_FREEBSD_NR_fhstatfs    398
#define TARGET_FREEBSD_NR_ksem_close  400
#define TARGET_FREEBSD_NR_ksem_post   401
#define TARGET_FREEBSD_NR_ksem_wait   402
#define TARGET_FREEBSD_NR_ksem_trywait        403
#define TARGET_FREEBSD_NR_ksem_init   404
#define TARGET_FREEBSD_NR_ksem_open   405
#define TARGET_FREEBSD_NR_ksem_unlink 406
#define TARGET_FREEBSD_NR_ksem_getvalue       407
#define TARGET_FREEBSD_NR_ksem_destroy        408
#define TARGET_FREEBSD_NR___mac_get_pid       409
#define TARGET_FREEBSD_NR___mac_get_link      410
#define TARGET_FREEBSD_NR___mac_set_link      411
#define TARGET_FREEBSD_NR_extattr_set_link    412
#define TARGET_FREEBSD_NR_extattr_get_link    413
#define TARGET_FREEBSD_NR_extattr_delete_link 414
#define TARGET_FREEBSD_NR___mac_execve        415
#define TARGET_FREEBSD_NR_sigaction   416
#define TARGET_FREEBSD_NR_sigreturn   417
#define TARGET_FREEBSD_NR_getcontext  421
#define TARGET_FREEBSD_NR_setcontext  422
#define TARGET_FREEBSD_NR_swapcontext 423
#define TARGET_FREEBSD_NR_swapoff     424
#define TARGET_FREEBSD_NR___acl_get_link      425
#define TARGET_FREEBSD_NR___acl_set_link      426
#define TARGET_FREEBSD_NR___acl_delete_link   427
#define TARGET_FREEBSD_NR___acl_aclcheck_link 428
#define TARGET_FREEBSD_NR_sigwait     429
#define TARGET_FREEBSD_NR_thr_create  430
#define TARGET_FREEBSD_NR_thr_exit    431
#define TARGET_FREEBSD_NR_thr_self    432
#define TARGET_FREEBSD_NR_thr_kill    433
#define TARGET_FREEBSD_NR__umtx_lock  434
#define TARGET_FREEBSD_NR__umtx_unlock        435
#define TARGET_FREEBSD_NR_jail_attach 436
#define TARGET_FREEBSD_NR_extattr_list_fd     437
#define TARGET_FREEBSD_NR_extattr_list_file   438
#define TARGET_FREEBSD_NR_extattr_list_link   439
#define TARGET_FREEBSD_NR_ksem_timedwait      441
#define TARGET_FREEBSD_NR_thr_suspend 442
#define TARGET_FREEBSD_NR_thr_wake    443
#define TARGET_FREEBSD_NR_kldunloadf  444
#define TARGET_FREEBSD_NR_audit       445
#define TARGET_FREEBSD_NR_auditon     446
#define TARGET_FREEBSD_NR_getauid     447
#define TARGET_FREEBSD_NR_setauid     448
#define TARGET_FREEBSD_NR_getaudit    449
#define TARGET_FREEBSD_NR_setaudit    450
#define TARGET_FREEBSD_NR_getaudit_addr       451
#define TARGET_FREEBSD_NR_setaudit_addr       452
#define TARGET_FREEBSD_NR_auditctl    453
#define TARGET_FREEBSD_NR__umtx_op    454
#define TARGET_FREEBSD_NR_thr_new     455
#define TARGET_FREEBSD_NR_sigqueue    456
#define TARGET_FREEBSD_NR_kmq_open    457
#define TARGET_FREEBSD_NR_kmq_setattr 458
#define TARGET_FREEBSD_NR_kmq_timedreceive    459
#define TARGET_FREEBSD_NR_kmq_timedsend       460
#define TARGET_FREEBSD_NR_kmq_notify  461
#define TARGET_FREEBSD_NR_kmq_unlink  462
#define TARGET_FREEBSD_NR_abort2      463
#define TARGET_FREEBSD_NR_thr_set_name        464
#define TARGET_FREEBSD_NR_aio_fsync   465
#define TARGET_FREEBSD_NR_rtprio_thread       466
#define TARGET_FREEBSD_NR_sctp_peeloff        471
#define TARGET_FREEBSD_NR_sctp_generic_sendmsg        472
#define TARGET_FREEBSD_NR_sctp_generic_sendmsg_iov    473
#define TARGET_FREEBSD_NR_sctp_generic_recvmsg        474
#define TARGET_FREEBSD_NR_pread       475
#define TARGET_FREEBSD_NR_pwrite      476
#define TARGET_FREEBSD_NR_mmap        477
#define TARGET_FREEBSD_NR_lseek       478
#define TARGET_FREEBSD_NR_truncate    479
#define TARGET_FREEBSD_NR_ftruncate   480
#define TARGET_FREEBSD_NR_thr_kill2   481
#define TARGET_FREEBSD_NR_shm_open    482
#define TARGET_FREEBSD_NR_shm_unlink  483
#define TARGET_FREEBSD_NR_cpuset      484
#define TARGET_FREEBSD_NR_cpuset_setid        485
#define TARGET_FREEBSD_NR_cpuset_getid        486
#define TARGET_FREEBSD_NR_cpuset_getaffinity  487
#define TARGET_FREEBSD_NR_cpuset_setaffinity  488
#define TARGET_FREEBSD_NR_faccessat   489
#define TARGET_FREEBSD_NR_fchmodat    490
#define TARGET_FREEBSD_NR_fchownat    491
#define TARGET_FREEBSD_NR_fexecve     492
#define TARGET_FREEBSD_NR_fstatat     493
#define TARGET_FREEBSD_NR_futimesat   494
#define TARGET_FREEBSD_NR_linkat      495
#define TARGET_FREEBSD_NR_mkdirat     496
#define TARGET_FREEBSD_NR_mkfifoat    497
#define TARGET_FREEBSD_NR_mknodat     498
#define TARGET_FREEBSD_NR_openat      499
#define TARGET_FREEBSD_NR_readlinkat  500
#define TARGET_FREEBSD_NR_renameat    501
#define TARGET_FREEBSD_NR_symlinkat   502
#define TARGET_FREEBSD_NR_unlinkat    503
#define TARGET_FREEBSD_NR_posix_openpt        504
