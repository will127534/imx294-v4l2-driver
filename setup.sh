#!/usr/bin/bash

DRV_VERSION=0.0.1

DRV_IMX=imx294

echo "Uninstalling any previous ${DRV_IMX} module"
dkms status ${DRV_IMX} | awk -F', ' '{print $2}' | xargs -n1 sudo dkms remove -m ${DRV_IMX} -v 

sudo mkdir -p /usr/src/${DRV_IMX}-${DRV_VERSION}

sudo cp -r $(pwd)/* /usr/src/${DRV_IMX}-${DRV_VERSION}

sudo dkms add -m ${DRV_IMX} -v ${DRV_VERSION}
sudo dkms build -m ${DRV_IMX} -v ${DRV_VERSION}
sudo dkms install -m ${DRV_IMX} -v ${DRV_VERSION}
