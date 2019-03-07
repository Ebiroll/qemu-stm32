# Linux initrd acceptance test.
#
# Copyright (c) 2018 Red Hat, Inc.
#
# Author:
#  Wainer dos Santos Moschetta <wainersm@redhat.com>
#
# This work is licensed under the terms of the GNU GPL, version 2 or
# later.  See the COPYING file in the top-level directory.

import logging
import tempfile
from avocado.utils.process import run

from avocado_qemu import Test


class LinuxInitrd(Test):
    """
    Checks QEMU evaluates correctly the initrd file passed as -initrd option.

    :avocado: tags=x86_64
    """

    timeout = 300

    def test_with_2gib_file_should_exit_error_msg_with_linux_v3_6(self):
        """
        Pretends to boot QEMU with an initrd file with size of 2GiB
        and expect it exits with error message.
        Fedora-18 shipped with linux-3.6 which have not supported xloadflags
        cannot support more than 2GiB initrd.
        """
        kernel_url = ('https://archives.fedoraproject.org/pub/archive/fedora/li'
                      'nux/releases/18/Fedora/x86_64/os/images/pxeboot/vmlinuz')
        kernel_hash = '41464f68efe42b9991250bed86c7081d2ccdbb21'
        kernel_path = self.fetch_asset(kernel_url, asset_hash=kernel_hash)
        max_size = 2 * (1024 ** 3) - 1

        with tempfile.NamedTemporaryFile() as initrd:
            initrd.seek(max_size)
            initrd.write(b'\0')
            initrd.flush()
            cmd = "%s -kernel %s -initrd %s -m 4096" % (
                  self.qemu_bin, kernel_path, initrd.name)
            res = run(cmd, ignore_status=True)
            self.assertEqual(res.exit_status, 1)
            expected_msg = r'.*initrd is too large.*max: \d+, need %s.*' % (
                max_size + 1)
            self.assertRegex(res.stderr_text, expected_msg)

    def test_with_2gib_file_should_work_with_linux_v4_16(self):
        """
        QEMU has supported up to 4 GiB initrd for recent kernel
        Expect guest can reach 'Unpacking initramfs...'
        """
        kernel_url = ('https://mirrors.kernel.org/fedora/releases/28/'
                      'Everything/x86_64/os/images/pxeboot/vmlinuz')
        kernel_hash = '238e083e114c48200f80d889f7e32eeb2793e02a'
        kernel_path = self.fetch_asset(kernel_url, asset_hash=kernel_hash)
        max_size = 2 * (1024 ** 3) + 1

        with tempfile.NamedTemporaryFile() as initrd:
            initrd.seek(max_size)
            initrd.write(b'\0')
            initrd.flush()

            self.vm.set_machine('pc')
            self.vm.set_console()
            kernel_command_line = 'console=ttyS0'
            self.vm.add_args('-kernel', kernel_path,
                             '-append', kernel_command_line,
                             '-initrd', initrd.name,
                             '-m', '5120')
            self.vm.launch()
            console = self.vm.console_socket.makefile()
            console_logger = logging.getLogger('console')
            while True:
                msg = console.readline()
                console_logger.debug(msg.strip())
                if 'Unpacking initramfs...' in msg:
                    break
                if 'Kernel panic - not syncing' in msg:
                    self.fail("Kernel panic reached")
