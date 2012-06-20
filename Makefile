#
# Quantis PCI/PCIe driver
#
# Copyright (c) 2004-2010 id Quantique SA, Carouge/Geneva, Switzerland
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

#Shell command to get the operating system name
KERNEL_NAME_CMD = uname -s | tr [A-Z] [a-z] | sed -e 's/sunos/solaris/'
#Shell command to get the operating system version
KERNEL_VERSION_CMD = uname -r

### default target ###
all: modules

#":" is a shell command doing nothing. Seems to be mandatory on Linux to
#make the first assignation to OSName.
check_supported_os:
	@:;OSName=`$(KERNEL_NAME_CMD)`;OSVer=`$(KERNEL_VERSION_CMD)` ; \
	case "$$OSName$$OSVer" in  \
		freebsd*)  \
			echo "Found FreeBSD compatible platform!" ;  \
			;;  \
		linux2.4*)  \
			echo "GNU/Linux is no more supported!"  \
			false  \
			;;  \
		linux2.6*)  \
			echo "Found GNU/Linux compatible platform"  \
			;;  \
		"solaris5.10")  \
			echo "Found Solaris compatible platform" ;  \
			;;  \
		"solaris5.11")  \
			echo "Found Solaris compatible platform" ;  \
			;;  \
		*)  \
			echo "$$OSName (Version $$OSVer) is not a supported platform!" ;  \
			false  \
			;;  \
	esac  \

### Build modules ###
modules: check_supported_os
	@cd `$(KERNEL_NAME_CMD)` ;  \
	$(MAKE) modules

### Install modules in system directories ###
install: modules
	@cd `$(KERNEL_NAME_CMD)` ;  \
	$(MAKE) install

modules_install: install

### Remove modules from system directories ###
uninstall: 
	@cd `$(KERNEL_NAME_CMD)` ;  \
	$(MAKE) uninstall

### Load modules in the kernel ###
load: 
	@cd `$(KERNEL_NAME_CMD)` ;  \
	$(MAKE) load

### Unload modules in the kernel ###
unload: 
	@cd `$(KERNEL_NAME_CMD)` ;  \
	$(MAKE) unload

### Remove generates files ###
clean:
	@cd `$(KERNEL_NAME_CMD)` ;  \
	$(MAKE) clean
	@rm -f *~


### Display help ###
help:
	@echo ""
	@echo "  Builds quantis_pci module on a Unix systems."
	@echo "  Syntax: make target"
	@echo ""
	@echo "  List of supported systems: "
	@echo "    Linux 2.6"
	@echo "    FreeBSD 7 and 8"
	@echo "    Solaris/OpenSolaris 5.10 and 5.11"
	@echo ""
	@echo "  Available targets:"
	@echo "    modules         - build the module (default target)"
	@echo "    install         - install the module"
	@echo "    modules_install - same as 'install'"
	@echo "    uninstall       - uninstall the module"
	@echo "    load            - load the module in the kernel"
	@echo "    unload          - unload the module in the kernel"
	@echo "    clean           - remove generated files"

