#!/bin/bash
BAREMETAL_IDE_PATH=~/Downloads/Baremetal-IDE-BWRC

cd ${BAREMETAL_IDE_PATH}
openocd -f ${BAREMETAL_IDE_PATH}/bsp/bearlyml/debug/bearlyml.cfg 