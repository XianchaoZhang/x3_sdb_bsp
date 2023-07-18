# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) 2016 Google, Inc
# Written by Simon Glass <sjg@chromium.org>
#
# Entry-type module for U-Boot device tree
#

from entry import Entry
from blob import Entry_blob

class Entry_u_boot_dtb(Entry_blob):
    """U-Boot device tree

    Properties / Entry arguments:
        - filename: Filename of u-boot.dtb (default 'u-boot.dtb')

    This is the U-Boot device tree, containing configuration information for
    U-Boot. U-Boot needs this to know what devices are present and which drivers
    to activate.
    """
    def __init__(self, section, etype, node):
        Entry_blob.__init__(self, section, etype, node)

    def GetDefaultFilename(self):
        return 'u-boot.dtb'
