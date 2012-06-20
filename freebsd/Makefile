#
# Quantis PCI/PCIe driver
#
# Copyright (c) 2004-2010 id Quantique SA, Carouge/Geneva, Switzerland
# Copyright (c) 2010 Alex Dupre
# All rights reserved.
#
# ----------------------------------------------------------------------------
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions, and the following disclaimer,
#    without modification.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. The name of the author may not be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY.
#
# ----------------------------------------------------------------------------
#
# Alternatively, this software may be distributed under the terms of the
# terms of the GNU General Public License version 2 as published by the
# Free Software Foundation.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
#
# ----------------------------------------------------------------------------
#
# For history of changes, ChangeLog.txt

### default target ###
all: modules

### Build modules ###
modules: 
	$(MAKE) -f Makefile.kld

### Install modules ###
install: modules
	@$(MAKE) -f Makefile.kld install

modules_install: install

### to have compatibility with the common makefile ###
uninstall:
	rm -f /boot/kernel/quantis.ko
	kldxref /boot/kernel

load: install
	@$(MAKE) -f Makefile.kld load

unload:
	@$(MAKE) -f Makefile.kld unload


### Remove generates files ###
clean:
	@$(MAKE) -f Makefile.kld clean
	rm -f @ machine


### Display help ###
help:
	@echo "  Builds quantis_pci module."
	@echo "  Syntax: make target"
	@echo ""
	@echo "  Available targets:"
	@echo "    modules         - build the module (default target)"
	@echo "    install         - install the module"
	@echo "    modules_install - same as 'install'"
	@echo "    uninstall       - uninstall the module"
	@echo "    load            - load the module in the kernel"
	@echo "    unload          - unload the module in the kernel"
	@echo "    clean           - remove generated files"


