# real2sim

The open library and toolchain for failure case reconstruction and continuous learning

## tools folder

The folder includes toolchain for extract data for recording files such as rosbag e.t.c. Currently it supports rosbag only.

## disengagement cases folder

The collected disengagement cases will be continually shared in this folder. The individual scenario is described by a folder numbered from 1. In each scenario folder, there are 3 type of file:

- ego_vehicle: this contains ego-vehicle recordings in this scenario.
- surrounding: this contains a list which describe the number and the classification of surroundsing
- number named files: these contain recording of surroundings distinguished by number

A README file is in each folder for describing the scenario.