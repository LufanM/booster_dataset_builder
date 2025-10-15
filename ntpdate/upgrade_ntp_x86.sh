#!/bin/bash
##Booster 2.3.4 ntpdate upgrade 
sudo apt install ntp

if grep -q "Booster 2.3.4 ntpdate upgrade" "/etc/ntp.conf"; then  
    sudo echo "#Booster 2.3.4 ntpdate upgrade start" >> /etc/ntp.conf
    sudo echo "restrict 192.168.10.0 mask 255.255.255.0 nomodify notrap" >> /etc/ntp.conf
    sudo echo "#Booster 2.3.4 ntpdate upgrade end" >> /etc/ntp.conf
fi
