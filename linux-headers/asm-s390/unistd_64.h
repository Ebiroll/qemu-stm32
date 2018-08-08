/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _ASM_S390_UNISTD_64_H
#define _ASM_S390_UNISTD_64_H

#define __NR_exit 1
#define __NR_fork 2
#define __NR_read 3
#define __NR_write 4
#define __NR_open 5
#define __NR_close 6
#define __NR_restart_syscall 7
#define __NR_creat 8
#define __NR_link 9
#define __NR_unlink 10
#define __NR_execve 11
#define __NR_chdir 12
#define __NR_mknod 14
#define __NR_chmod 15
#define __NR_lseek 19
#define __NR_getpid 20
#define __NR_mount 21
#define __NR_umount 22
#define __NR_ptrace 26
#define __NR_alarm 27
#define __NR_pause 29
#define __NR_utime 30
#define __NR_access 33
#define __NR_nice 34
#define __NR_sync 36
#define __NR_kill 37
#define __NR_rename 38
#define __NR_mkdir 39
#define __NR_rmdir 40
#define __NR_dup 41
#define __NR_pipe 42
#define __NR_times 43
#define __NR_brk 45
#define __NR_signal 48
#define __NR_acct 51
#define __NR_umount2 52
#define __NR_ioctl 54
#define __NR_fcntl 55
#define __NR_setpgid 57
#define __NR_umask 60
#define __NR_chroot 61
#define __NR_ustat 62
#define __NR_dup2 63
#define __NR_getppid 64
#define __NR_getpgrp 65
#define __NR_setsid 66
#define __NR_sigaction 67
#define __NR_sigsuspend 72
#define __NR_sigpending 73
#define __NR_sethostname 74
#define __NR_setrlimit 75
#define __NR_getrusage 77
#define __NR_gettimeofday 78
#define __NR_settimeofday 79
#define __NR_symlink 83
#define __NR_readlink 85
#define __NR_uselib 86
#define __NR_swapon 87
#define __NR_reboot 88
#define __NR_readdir 89
#define __NR_mmap 90
#define __NR_munmap 91
#define __NR_truncate 92
#define __NR_ftruncate 93
#define __NR_fchmod 94
#define __NR_getpriority 96
#define __NR_setpriority 97
#define __NR_statfs 99
#define __NR_fstatfs 100
#define __NR_socketcall 102
#define __NR_syslog 103
#define __NR_setitimer 104
#define __NR_getitimer 105
#define __NR_stat 106
#define __NR_lstat 107
#define __NR_fstat 108
#define __NR_lookup_dcookie 110
#define __NR_vhangup 111
#define __NR_idle 112
#define __NR_wait4 114
#define __NR_swapoff 115
#define __NR_sysinfo 116
#define __NR_ipc 117
#define __NR_fsync 118
#define __NR_sigreturn 119
#define __NR_clone 120
#define __NR_setdomainname 121
#define __NR_uname 122
#define __NR_adjtimex 124
#define __NR_mprotect 125
#define __NR_sigprocmask 126
#define __NR_create_module 127
#define __NR_init_module 128
#define __NR_delete_module 129
#define __NR_get_kernel_syms 130
#define __NR_quotactl 131
#define __NR_getpgid 132
#define __NR_fchdir 133
#define __NR_bdflush 134
#define __NR_sysfs 135
#define __NR_personality 136
#define __NR_afs_syscall 137
#define __NR_getdents 141
#define __NR_select 142
#define __NR_flock 143
#define __NR_msync 144
#define __NR_readv 145
#define __NR_writev 146
#define __NR_getsid 147
#define __NR_fdatasync 148
#define __NR__sysctl 149
#define __NR_mlock 150
#define __NR_munlock 151
#define __NR_mlockall 152
#define __NR_munlockall 153
#define __NR_sched_setparam 154
#define __NR_sched_getparam 155
#define __NR_sched_setscheduler 156
#define __NR_sched_getscheduler 157
#define __NR_sched_yield 158
#define __NR_sched_get_priority_max 159
#define __NR_sched_get_priority_min 160
#define __NR_sched_rr_get_interval 161
#define __NR_nanosleep 162
#define __NR_mremap 163
#define __NR_query_module 167
#define __NR_poll 168
#define __NR_nfsservctl 169
#define __NR_prctl 172
#define __NR_rt_sigreturn 173
#define __NR_rt_sigaction 174
#define __NR_rt_sigprocmask 175
#define __NR_rt_sigpending 176
#define __NR_rt_sigtimedwait 177
#define __NR_rt_sigqueueinfo 178
#define __NR_rt_sigsuspend 179
#define __NR_pread64 180
#define __NR_pwrite64 181
#define __NR_getcwd 183
#define __NR_capget 184
#define __NR_capset 185
#define __NR_sigaltstack 186
#define __NR_sendfile 187
#define __NR_getpmsg 188
#define __NR_putpmsg 189
#define __NR_vfork 190
#define __NR_getrlimit 191
#define __NR_lchown 198
#define __NR_getuid 199
#define __NR_getgid 200
#define __NR_geteuid 201
#define __NR_getegid 202
#define __NR_setreuid 203
#define __NR_setregid 204
#define __NR_getgroups 205
#define __NR_setgroups 206
#define __NR_fchown 207
#define __NR_setresuid 208
#define __NR_getresuid 209
#define __NR_setresgid 210
#define __NR_getresgid 211
#define __NR_chown 212
#define __NR_setuid 213
#define __NR_setgid 214
#define __NR_setfsuid 215
#define __NR_setfsgid 216
#define __NR_pivot_root 217
#define __NR_mincore 218
#define __NR_madvise 219
#define __NR_getdents64 220
#define __NR_readahead 222
#define __NR_setxattr 224
#define __NR_lsetxattr 225
#define __NR_fsetxattr 226
#define __NR_getxattr 227
#define __NR_lgetxattr 228
#define __NR_fgetxattr 229
#define __NR_listxattr 230
#define __NR_llistxattr 231
#define __NR_flistxattr 232
#define __NR_removexattr 233
#define __NR_lremovexattr 234
#define __NR_fremovexattr 235
#define __NR_gettid 236
#define __NR_tkill 237
#define __NR_futex 238
#define __NR_sched_setaffinity 239
#define __NR_sched_getaffinity 240
#define __NR_tgkill 241
#define __NR_io_setup 243
#define __NR_io_destroy 244
#define __NR_io_getevents 245
#define __NR_io_submit 246
#define __NR_io_cancel 247
#define __NR_exit_group 248
#define __NR_epoll_create 249
#define __NR_epoll_ctl 250
#define __NR_epoll_wait 251
#define __NR_set_tid_address 252
#define __NR_fadvise64 253
#define __NR_timer_create 254
#define __NR_timer_settime 255
#define __NR_timer_gettime 256
#define __NR_timer_getoverrun 257
#define __NR_timer_delete 258
#define __NR_clock_settime 259
#define __NR_clock_gettime 260
#define __NR_clock_getres 261
#define __NR_clock_nanosleep 262
#define __NR_statfs64 265
#define __NR_fstatfs64 266
#define __NR_remap_file_pages 267
#define __NR_mbind 268
#define __NR_get_mempolicy 269
#define __NR_set_mempolicy 270
#define __NR_mq_open 271
#define __NR_mq_unlink 272
#define __NR_mq_timedsend 273
#define __NR_mq_timedreceive 274
#define __NR_mq_notify 275
#define __NR_mq_getsetattr 276
#define __NR_kexec_load 277
#define __NR_add_key 278
#define __NR_request_key 279
#define __NR_keyctl 280
#define __NR_waitid 281
#define __NR_ioprio_set 282
#define __NR_ioprio_get 283
#define __NR_inotify_init 284
#define __NR_inotify_add_watch 285
#define __NR_inotify_rm_watch 286
#define __NR_migrate_pages 287
#define __NR_openat 288
#define __NR_mkdirat 289
#define __NR_mknodat 290
#define __NR_fchownat 291
#define __NR_futimesat 292
#define __NR_newfstatat 293
#define __NR_unlinkat 294
#define __NR_renameat 295
#define __NR_linkat 296
#define __NR_symlinkat 297
#define __NR_readlinkat 298
#define __NR_fchmodat 299
#define __NR_faccessat 300
#define __NR_pselect6 301
#define __NR_ppoll 302
#define __NR_unshare 303
#define __NR_set_robust_list 304
#define __NR_get_robust_list 305
#define __NR_splice 306
#define __NR_sync_file_range 307
#define __NR_tee 308
#define __NR_vmsplice 309
#define __NR_move_pages 310
#define __NR_getcpu 311
#define __NR_epoll_pwait 312
#define __NR_utimes 313
#define __NR_fallocate 314
#define __NR_utimensat 315
#define __NR_signalfd 316
#define __NR_timerfd 317
#define __NR_eventfd 318
#define __NR_timerfd_create 319
#define __NR_timerfd_settime 320
#define __NR_timerfd_gettime 321
#define __NR_signalfd4 322
#define __NR_eventfd2 323
#define __NR_inotify_init1 324
#define __NR_pipe2 325
#define __NR_dup3 326
#define __NR_epoll_create1 327
#define __NR_preadv 328
#define __NR_pwritev 329
#define __NR_rt_tgsigqueueinfo 330
#define __NR_perf_event_open 331
#define __NR_fanotify_init 332
#define __NR_fanotify_mark 333
#define __NR_prlimit64 334
#define __NR_name_to_handle_at 335
#define __NR_open_by_handle_at 336
#define __NR_clock_adjtime 337
#define __NR_syncfs 338
#define __NR_setns 339
#define __NR_process_vm_readv 340
#define __NR_process_vm_writev 341
#define __NR_s390_runtime_instr 342
#define __NR_kcmp 343
#define __NR_finit_module 344
#define __NR_sched_setattr 345
#define __NR_sched_getattr 346
#define __NR_renameat2 347
#define __NR_seccomp 348
#define __NR_getrandom 349
#define __NR_memfd_create 350
#define __NR_bpf 351
#define __NR_s390_pci_mmio_write 352
#define __NR_s390_pci_mmio_read 353
#define __NR_execveat 354
#define __NR_userfaultfd 355
#define __NR_membarrier 356
#define __NR_recvmmsg 357
#define __NR_sendmmsg 358
#define __NR_socket 359
#define __NR_socketpair 360
#define __NR_bind 361
#define __NR_connect 362
#define __NR_listen 363
#define __NR_accept4 364
#define __NR_getsockopt 365
#define __NR_setsockopt 366
#define __NR_getsockname 367
#define __NR_getpeername 368
#define __NR_sendto 369
#define __NR_sendmsg 370
#define __NR_recvfrom 371
#define __NR_recvmsg 372
#define __NR_shutdown 373
#define __NR_mlock2 374
#define __NR_copy_file_range 375
#define __NR_preadv2 376
#define __NR_pwritev2 377
#define __NR_s390_guarded_storage 378
#define __NR_statx 379
#define __NR_s390_sthyi 380
#define __NR_kexec_file_load 381
#define __NR_io_pgetevents 382
#define __NR_rseq 383

#endif /* _ASM_S390_UNISTD_64_H */
