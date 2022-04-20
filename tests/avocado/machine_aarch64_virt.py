# Functional test that boots a Linux kernel and checks the console
#
# Copyright (c) 2022 Linaro Ltd.
#
# Author:
#  Alex Bennée <alex.bennee@linaro.org>
#
# SPDX-License-Identifier: GPL-2.0-or-later

import time

from avocado_qemu import QemuSystemTest
from avocado_qemu import wait_for_console_pattern
from avocado_qemu import exec_command

class Aarch64VirtMachine(QemuSystemTest):
    KERNEL_COMMON_COMMAND_LINE = 'printk.time=0 '

    def wait_for_console_pattern(self, success_message, vm=None):
        wait_for_console_pattern(self, success_message,
                                 failure_message='Kernel panic - not syncing',
                                 vm=vm)

    def test_aarch64_virt(self):
        """
        :avocado: tags=arch:aarch64
        :avocado: tags=machine:virt
        :avocado: tags=accel:tcg
        :avocado: tags=cpu:max
        """
        kernel_url = ('https://fileserver.linaro.org/s/'
                      'z6B2ARM7DQT3HWN/download')

        kernel_hash = 'ed11daab50c151dde0e1e9c9cb8b2d9bd3215347'
        kernel_path = self.fetch_asset(kernel_url, asset_hash=kernel_hash)

        self.vm.set_console()
        kernel_command_line = (self.KERNEL_COMMON_COMMAND_LINE +
                               'console=ttyAMA0')
        self.require_accelerator("tcg")
        self.vm.add_args('-cpu', 'max,pauth-impdef=on',
                         '-accel', 'tcg',
                         '-kernel', kernel_path,
                         '-append', kernel_command_line)
        self.vm.launch()
        self.wait_for_console_pattern('Welcome to Buildroot')
        time.sleep(0.1)
        exec_command(self, 'root')
        time.sleep(0.1)
        exec_command(self, 'cat /proc/self/maps')
        time.sleep(0.1)
