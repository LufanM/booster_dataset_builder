#!/bin/bash
##Booster 2.3.4 ntpdate upgrade 
sudo apt install ntpdate
sudo cp ntpdate.service /etc/systemd/system/
sudo cp ntpdate.timer /etc/systemd/system/
sudo systemctl enable ntpdate.service
sudo systemctl enable ntpdate.timer
sudo systemctl stop systemd-timesyncd.service
sudo systemctl disable systemd-timesyncd.service
