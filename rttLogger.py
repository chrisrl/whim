# The following code was adapted from an example to log data from rtt and plot it.
# Modified by Aaron Farquharson


# -*- coding: utf-8 -*-
# Copyright 2017 Square, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#
# Example RTT terminal.
#
# This module creates an interactive terminal with the target using RTT.
#
# Usage: rtt target_device
# Author: Charles Nicholson
# Date: October 11, 2017
# Copyright: 2017 Square, Inc.

import pylink
import sys
import time
from builtins import input
import re
import matplotlib.pyplot as plt
import numpy
import signal, os

try:
    import thread
except ImportError:
    import _thread as thread


def read_rtt(jlink,f):
    """Reads the JLink RTT buffer #0 at 10Hz and prints to stdout.

    This method is a polling loop against the connected JLink unit. If
    the JLink is disconnected, it will exit. Additionally, if any exceptions
    are raised, they will be caught and re-raised after interrupting the
    main thread.

    sys.stdout.write and sys.stdout.flush are used since target terminals
    are expected to transmit newlines, which may or may not line up with the
    arbitrarily-chosen 1024-byte buffer that this loop uses to read.

    Args:
      jlink (pylink.JLink): The JLink to read.

    Raises:
      Exception on error.
    """
    try:
        while jlink.connected():
            if (jlink.opened()) and (jlink.connected()):
                terminal_bytes = jlink.rtt_read(0, 2048)
            if terminal_bytes and (not f.closed):
                # sys.stdout.write("".join(map(chr, terminal_bytes)))
                # sys.stdout.flush()
                f.write("".join(map(chr, terminal_bytes)))
                f.flush()
            # time.sleep(0.01)
    except Exception:
        print("IO read thread exception, exiting...")
        if terminal_bytes and (not f.closed):
            f.write("".join(map(chr, terminal_bytes)))
            f.flush()
            f.close()
            jlink.close()
        thread.interrupt_main()
        raise



def main(target_device, block_address=None):
    """Creates an interactive terminal to the target via RTT.

    The main loop opens a connection to the JLink, and then connects
    to the target device. RTT is started, the number of buffers is presented,
    and then two worker threads are spawned: one for read, and one for write.

    The main loops sleeps until the JLink is either disconnected or the
    user hits ctrl-c.

    Args:
      target_device (string): The target CPU to connect to.
      block_address (int): optional address pointing to start of RTT block.

    Returns:
      Always returns ``0`` or a JLinkException.

    Raises:
      JLinkException on error.
    """
    filename = "temp.txt"   #Temporary log file
    jlink = pylink.JLink()
    print("connecting to JLink...")
    jlink.open()
    print("connecting to %s..." % target_device)
    jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jlink.connect(target_device)
    print("connected, starting RTT...")
    jlink.rtt_start(block_address)
    

    f = open(filename, 'w+')
    while True:
        try:
            num_up = jlink.rtt_get_num_up_buffers()
            num_down = jlink.rtt_get_num_down_buffers()
            print("RTT started, %d up bufs, %d down bufs." % (num_up, num_down))
            break
        except pylink.errors.JLinkRTTException:
            time.sleep(0.1)

    print("up channels:")
    for buf_index in range(jlink.rtt_get_num_up_buffers()):
        buf = jlink.rtt_get_buf_descriptor(buf_index, True)
        print("    %d: name = %r, size = %d bytes, flags = %d" % (buf.BufferIndex, buf.name,
                                                                buf.SizeOfBuffer, buf.Flags))

    print("down channels:")
    for buf_index in range(jlink.rtt_get_num_down_buffers()):
        buf = jlink.rtt_get_buf_descriptor(buf_index, False)
        print("    %d: name = %r, size = %d bytes, flags = %d" % (buf.BufferIndex, buf.name,
                                                                buf.SizeOfBuffer, buf.Flags))

    sys.stdout.flush()

    try:
        print("Beginning Logging")
        sys.stdout.flush()
        thread.start_new_thread(read_rtt, (jlink,f,))
        while jlink.connected():
            time.sleep(1)
        print("JLink disconnected, exiting...")
    except KeyboardInterrupt:
        f.close()
        jlink.close()
        print("Exiting Program...")
        pass
    f.close()
    f = open(filename, 'r+')
    content = f.readlines()
    data = []
    for line in content:
        line = re.sub(r"[\n\t\s]*", "", line)
        if 'G-ForceValue:' in line:
            data.append(float(line.split('G-ForceValue:',1)[1]))
    plt.plot(data)
    plt.autoscale()
    plt.xlabel('Sample Number')
    plt.ylabel('G-Force Value')
    plt.grid(b=True, which='major', color='#666666', linestyle='-')
    # Show the minor grid lines with very faint and almost transparent grey lines
    plt.minorticks_on()
    plt.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
    plt.show()


if __name__ == "__main__":
  main('nRF52832_xxAA')
