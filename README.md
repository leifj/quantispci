
quantispci-dkms
===============

This is the debian packaging of drivers for the http://www.idquantique.com/ Quantis PCI board 
plus some patches to make it build for Linux 3 kernels. Tested on Ubuntu 12.04

building
========

```bash
# cd quantispci
# git checkout debian
# git-buildpackage -uc -us

```

I've used the docs at http://tjworld.net/wiki/Linux/Ubuntu/Kernel/BuildDebianDKMSPackage as
a basis for the debian packaging.
