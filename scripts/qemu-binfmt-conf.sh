#!/bin/sh
# enable automatic i386/ARM/M68K/MIPS/SPARC/PPC/s390 program execution by the kernel

# load the binfmt_misc module
if [ ! -d /proc/sys/fs/binfmt_misc ]; then
  /sbin/modprobe binfmt_misc
fi
if [ ! -f /proc/sys/fs/binfmt_misc/register ]; then
  mount binfmt_misc -t binfmt_misc /proc/sys/fs/binfmt_misc
fi

# probe cpu type
cpu=`uname -m`
case "$cpu" in
  i386|i486|i586|i686|i86pc|BePC|x86_64)
    cpu="i386"
  ;;
  m68k)
    cpu="m68k"
  ;;
  mips*)
    cpu="mips"
  ;;
  "Power Macintosh"|ppc|ppc64)
    cpu="ppc"
  ;;
  armv[4-9]*)
    cpu="arm"
  ;;
esac

# register the interpreter for each cpu except for the native one
if [ $cpu != "i386" ] ; then
    echo ':i386:M::\x7fELF\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x03\x00:\xff\xff\xff\xff\xff\xfe\xfe\xff\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff\xff:/usr/local/bin/qemu-i386:' > /proc/sys/fs/binfmt_misc/register
    echo ':i486:M::\x7fELF\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x06\x00:\xff\xff\xff\xff\xff\xfe\xfe\xff\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff\xff:/usr/local/bin/qemu-i386:' > /proc/sys/fs/binfmt_misc/register
fi
if [ $cpu != "alpha" ] ; then
    echo ':alpha:M::\x7fELF\x02\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x26\x90:\xff\xff\xff\xff\xff\xfe\xfe\xff\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff\xff:/usr/local/bin/qemu-alpha:' > /proc/sys/fs/binfmt_misc/register
fi
if [ $cpu != "arm" ] ; then
    echo   ':arm:M::\x7fELF\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x28\x00:\xff\xff\xff\xff\xff\xff\xff\x00\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff\xff:/usr/local/bin/qemu-arm:' > /proc/sys/fs/binfmt_misc/register
    echo   ':armeb:M::\x7fELF\x01\x02\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x28:\xff\xff\xff\xff\xff\xff\xff\x00\xff\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff:/usr/local/bin/qemu-armeb:' > /proc/sys/fs/binfmt_misc/register
fi
if [ $cpu != "sparc" ] ; then
    echo   ':sparc:M::\x7fELF\x01\x02\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x02:\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff:/usr/local/bin/qemu-sparc:' > /proc/sys/fs/binfmt_misc/register
fi
if [ $cpu != "ppc" ] ; then
    echo   ':ppc:M::\x7fELF\x01\x02\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x14:\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff:/usr/local/bin/qemu-ppc:' > /proc/sys/fs/binfmt_misc/register
fi
if [ $cpu != "m68k" ] ; then
    echo   'Please check cpu value and header information for m68k!'
    echo   ':m68k:M::\x7fELF\x01\x02\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x04:\xff\xff\xff\xff\xff\xff\xff\x00\xff\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff:/usr/local/bin/qemu-m68k:' > /proc/sys/fs/binfmt_misc/register
fi
if [ $cpu != "mips" ] ; then
    # FIXME: We could use the other endianness on a MIPS host.
    echo   ':mips:M::\x7fELF\x01\x02\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x08:\xff\xff\xff\xff\xff\xff\xff\x00\xff\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff:/usr/local/bin/qemu-mips:' > /proc/sys/fs/binfmt_misc/register
    echo   ':mipsel:M::\x7fELF\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x08\x00:\xff\xff\xff\xff\xff\xff\xff\x00\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff\xff:/usr/local/bin/qemu-mipsel:' > /proc/sys/fs/binfmt_misc/register
    echo   ':mipsn32:M::\x7fELF\x01\x02\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x08:\xff\xff\xff\xff\xff\xff\xff\x00\xff\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff:/usr/local/bin/qemu-mipsn32:' > /proc/sys/fs/binfmt_misc/register
    echo   ':mipsn32el:M::\x7fELF\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x08\x00:\xff\xff\xff\xff\xff\xff\xff\x00\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff\xff:/usr/local/bin/qemu-mipsn32el:' > /proc/sys/fs/binfmt_misc/register
    echo   ':mips64:M::\x7fELF\x02\x02\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x08:\xff\xff\xff\xff\xff\xff\xff\x00\xff\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff:/usr/local/bin/qemu-mips64:' > /proc/sys/fs/binfmt_misc/register
    echo   ':mips64el:M::\x7fELF\x02\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x08\x00:\xff\xff\xff\xff\xff\xff\xff\x00\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff\xff:/usr/local/bin/qemu-mips64el:' > /proc/sys/fs/binfmt_misc/register
fi
if [ $cpu != "sh" ] ; then
    echo    ':sh4:M::\x7fELF\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x2a\x00:\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff\xff:/usr/local/bin/qemu-sh4:' > /proc/sys/fs/binfmt_misc/register
    echo    ':sh4eb:M::\x7fELF\x01\x02\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x2a:\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff:/usr/local/bin/qemu-sh4eb:' > /proc/sys/fs/binfmt_misc/register
if [ $cpu != "s390x" ] ; then
    echo   ':s390x:M::\x7fELF\x02\x02\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\x16:\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff:/usr/local/bin/qemu-s390x:' > /proc/sys/fs/binfmt_misc/register
fi
