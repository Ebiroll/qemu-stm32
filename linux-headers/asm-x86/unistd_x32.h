#ifndef _ASM_UNISTD_X32_H
#define _ASM_UNISTD_X32_H

#define __NR_read (__X32_SYSCALL_BIT + 0)
#define __NR_write (__X32_SYSCALL_BIT + 1)
#define __NR_open (__X32_SYSCALL_BIT + 2)
#define __NR_close (__X32_SYSCALL_BIT + 3)
#define __NR_stat (__X32_SYSCALL_BIT + 4)
#define __NR_fstat (__X32_SYSCALL_BIT + 5)
#define __NR_lstat (__X32_SYSCALL_BIT + 6)
#define __NR_poll (__X32_SYSCALL_BIT + 7)
#define __NR_lseek (__X32_SYSCALL_BIT + 8)
#define __NR_mmap (__X32_SYSCALL_BIT + 9)
#define __NR_mprotect (__X32_SYSCALL_BIT + 10)
#define __NR_munmap (__X32_SYSCALL_BIT + 11)
#define __NR_brk (__X32_SYSCALL_BIT + 12)
#define __NR_rt_sigprocmask (__X32_SYSCALL_BIT + 14)
#define __NR_pread64 (__X32_SYSCALL_BIT + 17)
#define __NR_pwrite64 (__X32_SYSCALL_BIT + 18)
#define __NR_access (__X32_SYSCALL_BIT + 21)
#define __NR_pipe (__X32_SYSCALL_BIT + 22)
#define __NR_select (__X32_SYSCALL_BIT + 23)
#define __NR_sched_yield (__X32_SYSCALL_BIT + 24)
#define __NR_mremap (__X32_SYSCALL_BIT + 25)
#define __NR_msync (__X32_SYSCALL_BIT + 26)
#define __NR_mincore (__X32_SYSCALL_BIT + 27)
#define __NR_madvise (__X32_SYSCALL_BIT + 28)
#define __NR_shmget (__X32_SYSCALL_BIT + 29)
#define __NR_shmat (__X32_SYSCALL_BIT + 30)
#define __NR_shmctl (__X32_SYSCALL_BIT + 31)
#define __NR_dup (__X32_SYSCALL_BIT + 32)
#define __NR_dup2 (__X32_SYSCALL_BIT + 33)
#define __NR_pause (__X32_SYSCALL_BIT + 34)
#define __NR_nanosleep (__X32_SYSCALL_BIT + 35)
#define __NR_getitimer (__X32_SYSCALL_BIT + 36)
#define __NR_alarm (__X32_SYSCALL_BIT + 37)
#define __NR_setitimer (__X32_SYSCALL_BIT + 38)
#define __NR_getpid (__X32_SYSCALL_BIT + 39)
#define __NR_sendfile (__X32_SYSCALL_BIT + 40)
#define __NR_socket (__X32_SYSCALL_BIT + 41)
#define __NR_connect (__X32_SYSCALL_BIT + 42)
#define __NR_accept (__X32_SYSCALL_BIT + 43)
#define __NR_sendto (__X32_SYSCALL_BIT + 44)
#define __NR_shutdown (__X32_SYSCALL_BIT + 48)
#define __NR_bind (__X32_SYSCALL_BIT + 49)
#define __NR_listen (__X32_SYSCALL_BIT + 50)
#define __NR_getsockname (__X32_SYSCALL_BIT + 51)
#define __NR_getpeername (__X32_SYSCALL_BIT + 52)
#define __NR_socketpair (__X32_SYSCALL_BIT + 53)
#define __NR_clone (__X32_SYSCALL_BIT + 56)
#define __NR_fork (__X32_SYSCALL_BIT + 57)
#define __NR_vfork (__X32_SYSCALL_BIT + 58)
#define __NR_exit (__X32_SYSCALL_BIT + 60)
#define __NR_wait4 (__X32_SYSCALL_BIT + 61)
#define __NR_kill (__X32_SYSCALL_BIT + 62)
#define __NR_uname (__X32_SYSCALL_BIT + 63)
#define __NR_semget (__X32_SYSCALL_BIT + 64)
#define __NR_semop (__X32_SYSCALL_BIT + 65)
#define __NR_semctl (__X32_SYSCALL_BIT + 66)
#define __NR_shmdt (__X32_SYSCALL_BIT + 67)
#define __NR_msgget (__X32_SYSCALL_BIT + 68)
#define __NR_msgsnd (__X32_SYSCALL_BIT + 69)
#define __NR_msgrcv (__X32_SYSCALL_BIT + 70)
#define __NR_msgctl (__X32_SYSCALL_BIT + 71)
#define __NR_fcntl (__X32_SYSCALL_BIT + 72)
#define __NR_flock (__X32_SYSCALL_BIT + 73)
#define __NR_fsync (__X32_SYSCALL_BIT + 74)
#define __NR_fdatasync (__X32_SYSCALL_BIT + 75)
#define __NR_truncate (__X32_SYSCALL_BIT + 76)
#define __NR_ftruncate (__X32_SYSCALL_BIT + 77)
#define __NR_getdents (__X32_SYSCALL_BIT + 78)
#define __NR_getcwd (__X32_SYSCALL_BIT + 79)
#define __NR_chdir (__X32_SYSCALL_BIT + 80)
#define __NR_fchdir (__X32_SYSCALL_BIT + 81)
#define __NR_rename (__X32_SYSCALL_BIT + 82)
#define __NR_mkdir (__X32_SYSCALL_BIT + 83)
#define __NR_rmdir (__X32_SYSCALL_BIT + 84)
#define __NR_creat (__X32_SYSCALL_BIT + 85)
#define __NR_link (__X32_SYSCALL_BIT + 86)
#define __NR_unlink (__X32_SYSCALL_BIT + 87)
#define __NR_symlink (__X32_SYSCALL_BIT + 88)
#define __NR_readlink (__X32_SYSCALL_BIT + 89)
#define __NR_chmod (__X32_SYSCALL_BIT + 90)
#define __NR_fchmod (__X32_SYSCALL_BIT + 91)
#define __NR_chown (__X32_SYSCALL_BIT + 92)
#define __NR_fchown (__X32_SYSCALL_BIT + 93)
#define __NR_lchown (__X32_SYSCALL_BIT + 94)
#define __NR_umask (__X32_SYSCALL_BIT + 95)
#define __NR_gettimeofday (__X32_SYSCALL_BIT + 96)
#define __NR_getrlimit (__X32_SYSCALL_BIT + 97)
#define __NR_getrusage (__X32_SYSCALL_BIT + 98)
#define __NR_sysinfo (__X32_SYSCALL_BIT + 99)
#define __NR_times (__X32_SYSCALL_BIT + 100)
#define __NR_getuid (__X32_SYSCALL_BIT + 102)
#define __NR_syslog (__X32_SYSCALL_BIT + 103)
#define __NR_getgid (__X32_SYSCALL_BIT + 104)
#define __NR_setuid (__X32_SYSCALL_BIT + 105)
#define __NR_setgid (__X32_SYSCALL_BIT + 106)
#define __NR_geteuid (__X32_SYSCALL_BIT + 107)
#define __NR_getegid (__X32_SYSCALL_BIT + 108)
#define __NR_setpgid (__X32_SYSCALL_BIT + 109)
#define __NR_getppid (__X32_SYSCALL_BIT + 110)
#define __NR_getpgrp (__X32_SYSCALL_BIT + 111)
#define __NR_setsid (__X32_SYSCALL_BIT + 112)
#define __NR_setreuid (__X32_SYSCALL_BIT + 113)
#define __NR_setregid (__X32_SYSCALL_BIT + 114)
#define __NR_getgroups (__X32_SYSCALL_BIT + 115)
#define __NR_setgroups (__X32_SYSCALL_BIT + 116)
#define __NR_setresuid (__X32_SYSCALL_BIT + 117)
#define __NR_getresuid (__X32_SYSCALL_BIT + 118)
#define __NR_setresgid (__X32_SYSCALL_BIT + 119)
#define __NR_getresgid (__X32_SYSCALL_BIT + 120)
#define __NR_getpgid (__X32_SYSCALL_BIT + 121)
#define __NR_setfsuid (__X32_SYSCALL_BIT + 122)
#define __NR_setfsgid (__X32_SYSCALL_BIT + 123)
#define __NR_getsid (__X32_SYSCALL_BIT + 124)
#define __NR_capget (__X32_SYSCALL_BIT + 125)
#define __NR_capset (__X32_SYSCALL_BIT + 126)
#define __NR_rt_sigsuspend (__X32_SYSCALL_BIT + 130)
#define __NR_utime (__X32_SYSCALL_BIT + 132)
#define __NR_mknod (__X32_SYSCALL_BIT + 133)
#define __NR_personality (__X32_SYSCALL_BIT + 135)
#define __NR_ustat (__X32_SYSCALL_BIT + 136)
#define __NR_statfs (__X32_SYSCALL_BIT + 137)
#define __NR_fstatfs (__X32_SYSCALL_BIT + 138)
#define __NR_sysfs (__X32_SYSCALL_BIT + 139)
#define __NR_getpriority (__X32_SYSCALL_BIT + 140)
#define __NR_setpriority (__X32_SYSCALL_BIT + 141)
#define __NR_sched_setparam (__X32_SYSCALL_BIT + 142)
#define __NR_sched_getparam (__X32_SYSCALL_BIT + 143)
#define __NR_sched_setscheduler (__X32_SYSCALL_BIT + 144)
#define __NR_sched_getscheduler (__X32_SYSCALL_BIT + 145)
#define __NR_sched_get_priority_max (__X32_SYSCALL_BIT + 146)
#define __NR_sched_get_priority_min (__X32_SYSCALL_BIT + 147)
#define __NR_sched_rr_get_interval (__X32_SYSCALL_BIT + 148)
#define __NR_mlock (__X32_SYSCALL_BIT + 149)
#define __NR_munlock (__X32_SYSCALL_BIT + 150)
#define __NR_mlockall (__X32_SYSCALL_BIT + 151)
#define __NR_munlockall (__X32_SYSCALL_BIT + 152)
#define __NR_vhangup (__X32_SYSCALL_BIT + 153)
#define __NR_modify_ldt (__X32_SYSCALL_BIT + 154)
#define __NR_pivot_root (__X32_SYSCALL_BIT + 155)
#define __NR_prctl (__X32_SYSCALL_BIT + 157)
#define __NR_arch_prctl (__X32_SYSCALL_BIT + 158)
#define __NR_adjtimex (__X32_SYSCALL_BIT + 159)
#define __NR_setrlimit (__X32_SYSCALL_BIT + 160)
#define __NR_chroot (__X32_SYSCALL_BIT + 161)
#define __NR_sync (__X32_SYSCALL_BIT + 162)
#define __NR_acct (__X32_SYSCALL_BIT + 163)
#define __NR_settimeofday (__X32_SYSCALL_BIT + 164)
#define __NR_mount (__X32_SYSCALL_BIT + 165)
#define __NR_umount2 (__X32_SYSCALL_BIT + 166)
#define __NR_swapon (__X32_SYSCALL_BIT + 167)
#define __NR_swapoff (__X32_SYSCALL_BIT + 168)
#define __NR_reboot (__X32_SYSCALL_BIT + 169)
#define __NR_sethostname (__X32_SYSCALL_BIT + 170)
#define __NR_setdomainname (__X32_SYSCALL_BIT + 171)
#define __NR_iopl (__X32_SYSCALL_BIT + 172)
#define __NR_ioperm (__X32_SYSCALL_BIT + 173)
#define __NR_init_module (__X32_SYSCALL_BIT + 175)
#define __NR_delete_module (__X32_SYSCALL_BIT + 176)
#define __NR_quotactl (__X32_SYSCALL_BIT + 179)
#define __NR_getpmsg (__X32_SYSCALL_BIT + 181)
#define __NR_putpmsg (__X32_SYSCALL_BIT + 182)
#define __NR_afs_syscall (__X32_SYSCALL_BIT + 183)
#define __NR_tuxcall (__X32_SYSCALL_BIT + 184)
#define __NR_security (__X32_SYSCALL_BIT + 185)
#define __NR_gettid (__X32_SYSCALL_BIT + 186)
#define __NR_readahead (__X32_SYSCALL_BIT + 187)
#define __NR_setxattr (__X32_SYSCALL_BIT + 188)
#define __NR_lsetxattr (__X32_SYSCALL_BIT + 189)
#define __NR_fsetxattr (__X32_SYSCALL_BIT + 190)
#define __NR_getxattr (__X32_SYSCALL_BIT + 191)
#define __NR_lgetxattr (__X32_SYSCALL_BIT + 192)
#define __NR_fgetxattr (__X32_SYSCALL_BIT + 193)
#define __NR_listxattr (__X32_SYSCALL_BIT + 194)
#define __NR_llistxattr (__X32_SYSCALL_BIT + 195)
#define __NR_flistxattr (__X32_SYSCALL_BIT + 196)
#define __NR_removexattr (__X32_SYSCALL_BIT + 197)
#define __NR_lremovexattr (__X32_SYSCALL_BIT + 198)
#define __NR_fremovexattr (__X32_SYSCALL_BIT + 199)
#define __NR_tkill (__X32_SYSCALL_BIT + 200)
#define __NR_time (__X32_SYSCALL_BIT + 201)
#define __NR_futex (__X32_SYSCALL_BIT + 202)
#define __NR_sched_setaffinity (__X32_SYSCALL_BIT + 203)
#define __NR_sched_getaffinity (__X32_SYSCALL_BIT + 204)
#define __NR_io_destroy (__X32_SYSCALL_BIT + 207)
#define __NR_io_getevents (__X32_SYSCALL_BIT + 208)
#define __NR_io_cancel (__X32_SYSCALL_BIT + 210)
#define __NR_lookup_dcookie (__X32_SYSCALL_BIT + 212)
#define __NR_epoll_create (__X32_SYSCALL_BIT + 213)
#define __NR_remap_file_pages (__X32_SYSCALL_BIT + 216)
#define __NR_getdents64 (__X32_SYSCALL_BIT + 217)
#define __NR_set_tid_address (__X32_SYSCALL_BIT + 218)
#define __NR_restart_syscall (__X32_SYSCALL_BIT + 219)
#define __NR_semtimedop (__X32_SYSCALL_BIT + 220)
#define __NR_fadvise64 (__X32_SYSCALL_BIT + 221)
#define __NR_timer_settime (__X32_SYSCALL_BIT + 223)
#define __NR_timer_gettime (__X32_SYSCALL_BIT + 224)
#define __NR_timer_getoverrun (__X32_SYSCALL_BIT + 225)
#define __NR_timer_delete (__X32_SYSCALL_BIT + 226)
#define __NR_clock_settime (__X32_SYSCALL_BIT + 227)
#define __NR_clock_gettime (__X32_SYSCALL_BIT + 228)
#define __NR_clock_getres (__X32_SYSCALL_BIT + 229)
#define __NR_clock_nanosleep (__X32_SYSCALL_BIT + 230)
#define __NR_exit_group (__X32_SYSCALL_BIT + 231)
#define __NR_epoll_wait (__X32_SYSCALL_BIT + 232)
#define __NR_epoll_ctl (__X32_SYSCALL_BIT + 233)
#define __NR_tgkill (__X32_SYSCALL_BIT + 234)
#define __NR_utimes (__X32_SYSCALL_BIT + 235)
#define __NR_mbind (__X32_SYSCALL_BIT + 237)
#define __NR_set_mempolicy (__X32_SYSCALL_BIT + 238)
#define __NR_get_mempolicy (__X32_SYSCALL_BIT + 239)
#define __NR_mq_open (__X32_SYSCALL_BIT + 240)
#define __NR_mq_unlink (__X32_SYSCALL_BIT + 241)
#define __NR_mq_timedsend (__X32_SYSCALL_BIT + 242)
#define __NR_mq_timedreceive (__X32_SYSCALL_BIT + 243)
#define __NR_mq_getsetattr (__X32_SYSCALL_BIT + 245)
#define __NR_add_key (__X32_SYSCALL_BIT + 248)
#define __NR_request_key (__X32_SYSCALL_BIT + 249)
#define __NR_keyctl (__X32_SYSCALL_BIT + 250)
#define __NR_ioprio_set (__X32_SYSCALL_BIT + 251)
#define __NR_ioprio_get (__X32_SYSCALL_BIT + 252)
#define __NR_inotify_init (__X32_SYSCALL_BIT + 253)
#define __NR_inotify_add_watch (__X32_SYSCALL_BIT + 254)
#define __NR_inotify_rm_watch (__X32_SYSCALL_BIT + 255)
#define __NR_migrate_pages (__X32_SYSCALL_BIT + 256)
#define __NR_openat (__X32_SYSCALL_BIT + 257)
#define __NR_mkdirat (__X32_SYSCALL_BIT + 258)
#define __NR_mknodat (__X32_SYSCALL_BIT + 259)
#define __NR_fchownat (__X32_SYSCALL_BIT + 260)
#define __NR_futimesat (__X32_SYSCALL_BIT + 261)
#define __NR_newfstatat (__X32_SYSCALL_BIT + 262)
#define __NR_unlinkat (__X32_SYSCALL_BIT + 263)
#define __NR_renameat (__X32_SYSCALL_BIT + 264)
#define __NR_linkat (__X32_SYSCALL_BIT + 265)
#define __NR_symlinkat (__X32_SYSCALL_BIT + 266)
#define __NR_readlinkat (__X32_SYSCALL_BIT + 267)
#define __NR_fchmodat (__X32_SYSCALL_BIT + 268)
#define __NR_faccessat (__X32_SYSCALL_BIT + 269)
#define __NR_pselect6 (__X32_SYSCALL_BIT + 270)
#define __NR_ppoll (__X32_SYSCALL_BIT + 271)
#define __NR_unshare (__X32_SYSCALL_BIT + 272)
#define __NR_splice (__X32_SYSCALL_BIT + 275)
#define __NR_tee (__X32_SYSCALL_BIT + 276)
#define __NR_sync_file_range (__X32_SYSCALL_BIT + 277)
#define __NR_utimensat (__X32_SYSCALL_BIT + 280)
#define __NR_epoll_pwait (__X32_SYSCALL_BIT + 281)
#define __NR_signalfd (__X32_SYSCALL_BIT + 282)
#define __NR_timerfd_create (__X32_SYSCALL_BIT + 283)
#define __NR_eventfd (__X32_SYSCALL_BIT + 284)
#define __NR_fallocate (__X32_SYSCALL_BIT + 285)
#define __NR_timerfd_settime (__X32_SYSCALL_BIT + 286)
#define __NR_timerfd_gettime (__X32_SYSCALL_BIT + 287)
#define __NR_accept4 (__X32_SYSCALL_BIT + 288)
#define __NR_signalfd4 (__X32_SYSCALL_BIT + 289)
#define __NR_eventfd2 (__X32_SYSCALL_BIT + 290)
#define __NR_epoll_create1 (__X32_SYSCALL_BIT + 291)
#define __NR_dup3 (__X32_SYSCALL_BIT + 292)
#define __NR_pipe2 (__X32_SYSCALL_BIT + 293)
#define __NR_inotify_init1 (__X32_SYSCALL_BIT + 294)
#define __NR_perf_event_open (__X32_SYSCALL_BIT + 298)
#define __NR_fanotify_init (__X32_SYSCALL_BIT + 300)
#define __NR_fanotify_mark (__X32_SYSCALL_BIT + 301)
#define __NR_prlimit64 (__X32_SYSCALL_BIT + 302)
#define __NR_name_to_handle_at (__X32_SYSCALL_BIT + 303)
#define __NR_open_by_handle_at (__X32_SYSCALL_BIT + 304)
#define __NR_clock_adjtime (__X32_SYSCALL_BIT + 305)
#define __NR_syncfs (__X32_SYSCALL_BIT + 306)
#define __NR_setns (__X32_SYSCALL_BIT + 308)
#define __NR_getcpu (__X32_SYSCALL_BIT + 309)
#define __NR_kcmp (__X32_SYSCALL_BIT + 312)
#define __NR_finit_module (__X32_SYSCALL_BIT + 313)
#define __NR_sched_setattr (__X32_SYSCALL_BIT + 314)
#define __NR_sched_getattr (__X32_SYSCALL_BIT + 315)
#define __NR_renameat2 (__X32_SYSCALL_BIT + 316)
#define __NR_seccomp (__X32_SYSCALL_BIT + 317)
#define __NR_getrandom (__X32_SYSCALL_BIT + 318)
#define __NR_memfd_create (__X32_SYSCALL_BIT + 319)
#define __NR_kexec_file_load (__X32_SYSCALL_BIT + 320)
#define __NR_bpf (__X32_SYSCALL_BIT + 321)
#define __NR_userfaultfd (__X32_SYSCALL_BIT + 323)
#define __NR_membarrier (__X32_SYSCALL_BIT + 324)
#define __NR_mlock2 (__X32_SYSCALL_BIT + 325)
#define __NR_copy_file_range (__X32_SYSCALL_BIT + 326)
#define __NR_pkey_mprotect (__X32_SYSCALL_BIT + 329)
#define __NR_pkey_alloc (__X32_SYSCALL_BIT + 330)
#define __NR_pkey_free (__X32_SYSCALL_BIT + 331)
#define __NR_statx (__X32_SYSCALL_BIT + 332)
#define __NR_io_pgetevents (__X32_SYSCALL_BIT + 333)
#define __NR_rseq (__X32_SYSCALL_BIT + 334)
#define __NR_pidfd_send_signal (__X32_SYSCALL_BIT + 424)
#define __NR_io_uring_setup (__X32_SYSCALL_BIT + 425)
#define __NR_io_uring_enter (__X32_SYSCALL_BIT + 426)
#define __NR_io_uring_register (__X32_SYSCALL_BIT + 427)
#define __NR_open_tree (__X32_SYSCALL_BIT + 428)
#define __NR_move_mount (__X32_SYSCALL_BIT + 429)
#define __NR_fsopen (__X32_SYSCALL_BIT + 430)
#define __NR_fsconfig (__X32_SYSCALL_BIT + 431)
#define __NR_fsmount (__X32_SYSCALL_BIT + 432)
#define __NR_fspick (__X32_SYSCALL_BIT + 433)
#define __NR_pidfd_open (__X32_SYSCALL_BIT + 434)
#define __NR_clone3 (__X32_SYSCALL_BIT + 435)
#define __NR_close_range (__X32_SYSCALL_BIT + 436)
#define __NR_openat2 (__X32_SYSCALL_BIT + 437)
#define __NR_pidfd_getfd (__X32_SYSCALL_BIT + 438)
#define __NR_faccessat2 (__X32_SYSCALL_BIT + 439)
#define __NR_process_madvise (__X32_SYSCALL_BIT + 440)
#define __NR_epoll_pwait2 (__X32_SYSCALL_BIT + 441)
#define __NR_mount_setattr (__X32_SYSCALL_BIT + 442)
#define __NR_quotactl_fd (__X32_SYSCALL_BIT + 443)
#define __NR_landlock_create_ruleset (__X32_SYSCALL_BIT + 444)
#define __NR_landlock_add_rule (__X32_SYSCALL_BIT + 445)
#define __NR_landlock_restrict_self (__X32_SYSCALL_BIT + 446)
#define __NR_rt_sigaction (__X32_SYSCALL_BIT + 512)
#define __NR_rt_sigreturn (__X32_SYSCALL_BIT + 513)
#define __NR_ioctl (__X32_SYSCALL_BIT + 514)
#define __NR_readv (__X32_SYSCALL_BIT + 515)
#define __NR_writev (__X32_SYSCALL_BIT + 516)
#define __NR_recvfrom (__X32_SYSCALL_BIT + 517)
#define __NR_sendmsg (__X32_SYSCALL_BIT + 518)
#define __NR_recvmsg (__X32_SYSCALL_BIT + 519)
#define __NR_execve (__X32_SYSCALL_BIT + 520)
#define __NR_ptrace (__X32_SYSCALL_BIT + 521)
#define __NR_rt_sigpending (__X32_SYSCALL_BIT + 522)
#define __NR_rt_sigtimedwait (__X32_SYSCALL_BIT + 523)
#define __NR_rt_sigqueueinfo (__X32_SYSCALL_BIT + 524)
#define __NR_sigaltstack (__X32_SYSCALL_BIT + 525)
#define __NR_timer_create (__X32_SYSCALL_BIT + 526)
#define __NR_mq_notify (__X32_SYSCALL_BIT + 527)
#define __NR_kexec_load (__X32_SYSCALL_BIT + 528)
#define __NR_waitid (__X32_SYSCALL_BIT + 529)
#define __NR_set_robust_list (__X32_SYSCALL_BIT + 530)
#define __NR_get_robust_list (__X32_SYSCALL_BIT + 531)
#define __NR_vmsplice (__X32_SYSCALL_BIT + 532)
#define __NR_move_pages (__X32_SYSCALL_BIT + 533)
#define __NR_preadv (__X32_SYSCALL_BIT + 534)
#define __NR_pwritev (__X32_SYSCALL_BIT + 535)
#define __NR_rt_tgsigqueueinfo (__X32_SYSCALL_BIT + 536)
#define __NR_recvmmsg (__X32_SYSCALL_BIT + 537)
#define __NR_sendmmsg (__X32_SYSCALL_BIT + 538)
#define __NR_process_vm_readv (__X32_SYSCALL_BIT + 539)
#define __NR_process_vm_writev (__X32_SYSCALL_BIT + 540)
#define __NR_setsockopt (__X32_SYSCALL_BIT + 541)
#define __NR_getsockopt (__X32_SYSCALL_BIT + 542)
#define __NR_io_setup (__X32_SYSCALL_BIT + 543)
#define __NR_io_submit (__X32_SYSCALL_BIT + 544)
#define __NR_execveat (__X32_SYSCALL_BIT + 545)
#define __NR_preadv2 (__X32_SYSCALL_BIT + 546)
#define __NR_pwritev2 (__X32_SYSCALL_BIT + 547)


#endif /* _ASM_UNISTD_X32_H */
