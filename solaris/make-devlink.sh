#!/bin/sh

grep qrandom /etc/devlink.tab > /dev/null
if [ $? -eq 0 ]; then
  echo qrandom entry already in /etc/devlink.tab
  exit 0
fi
echo "type=ddi_pseudo;name=pci179a,1	qrandom\N0" >> /etc/devlink.tab
echo "qrandom entry added to /etc/devlink.tab. reload with 'devfsadm'"

