#
# Quantis PCI/PCIe driver for Solaris OS
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

.KEEP_STATE:

ARCH=$(shell uname -m)
ARCH:sh	= uname -m

CFLAGS = -I. -I../common -D_KERNEL
#CFLAGS += -DDEBUG

# uncomment this if you use suncc: the compilation
#is done with c99 semantics, all warnings and all warnings produce
#errors
CC=cc
#To debug: a lot of debbugging traces are available and no optimisation are used
#C64_SPARC_FLAGS = -m64 -DDEBUG -xO0 -xc99 -erroff=%none -errwarn
#C64_X86_FLAGS = -m64 -xarch=ss2a -xmodel=kernel -DDEBUG -XO0 -xc99 -erroff=%none -errwarn
#C32FLAGS = -xO0 -DDEBUG -xc99 -erroff=%none -errwarn
#To production code: all the debugging traces are removed from the code, standard optimisations are used
C64_SPARC_FLAGS = -m64 -xO3 -xc99 -erroff=%none 
C64_X86_FLAGS = -m64 -xarch=ss2a -xmodel=kernel -XO3 -xc99 -erroff=%none -errwarn
C32FLAGS = -xO3 -xc99 -erroff=%none

# uncomment this if you use gcc
#CC=g++
#C64_SPARC_FLAGS = -O2 -std=c99 -m64 -mcpu=v9 -mcmodel=medlow -fno-pic -mno-fpu -ffreestanding -nodefaultlibs -Wall -pedantic
#C64_X86_FLAGS = -O2 -std=c99 -m64 -mcmodel=kernel -mno-red-zone -ffreestanding -nodefaultlibs -Wall -pedantic
#C32FLAGS = -O2 -ffreestanding -nodefaultlibs



all: all_$(ARCH)

# For compatibility with the common Makefile
modules: all_$(ARCH)


all_sun4u: sparcv9/quantis sparc/quantis 
all_i86pc: i386/quantis

sparcv9/quantis.o: quantis_pci_solaris.c
	-mkdir sparcv9 > /dev/null 2>&1
	$(CC) $(CFLAGS) $(C64_SPARC_FLAGS) -c -o $@ quantis_pci_solaris.c

sparcv9/quantis: sparcv9/quantis.o
	ld -r -o $@ sparcv9/quantis.o

sparc/quantis.o: quantis_pci_solaris.c
	-mkdir sparc > /dev/null 2>&1
	$(CC) $(CFLAGS) $(C32FLAGS) -c -o $@ quantis_pci_solaris.c

sparc/quantis: sparc/quantis.o
	ld -r -o $@ sparc/quantis.o

amd64/quantis.o: quantis_pci_solaris.c
	-mkdir amd64 > /dev/null 2>&1
	$(CC) $(CFLAGS) $(C64_X86_FLAGS) -c -o $@ quantis_pci_solaris.c

amd64/quantis: amd64/quantis.o
	ld -r -o $@ amd64/quantis.o

i386/quantis.o: quantis_pci_solaris.c
	-mkdir i386 > /dev/null 2>&1
	$(CC) $(CFLAGS) $(C32FLAGS) -c -o $@ quantis_pci_solaris.c

i386/quantis: i386/quantis.o
	ld -r -o $@ i386/quantis.o

clean:
	\rm -f sparc/quantis sparcv9/quantis i386/quantis sparc/*.o sparcv9/*.o i386/*.o *~

install: install_$(ARCH)
uninstall: uninstall_$(ARCH)

install_sun4u:
	/usr/ucb/install sparc/quantis /kernel/drv 
	/usr/ucb/install sparcv9/quantis /kernel/drv/sparcv9 

uninstall_sun4u:
	\rm /kernel/drv/quantis
	\rm /kernel/drv/sparcv9/quantis

install_i86pc:
	/usr/ucb/install i386/quantis /kernel/drv 

uninstall_i86pc:
	\rm /kernel/drv/quantis

load:
	./make-devlink.sh
	add_drv -v -i '"pci179a,1"' -m '* 0444 root sys' quantis

unload:
	rem_drv quantis

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
