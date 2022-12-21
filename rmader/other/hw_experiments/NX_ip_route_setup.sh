#!/bin/bash

sudo ip route add 192.168.15.0/24 via 192.168.0.201  #NUC1
sudo ip route add 192.168.16.0/24 via 192.168.0.202  #NUC2
sudo ip route add 192.168.17.0/24 via 192.168.0.203  #NUC3
sudo ip route add 192.168.18.0/24 via 192.168.0.204  #NUC4
sudo ip route add 192.168.19.0/24 via 192.168.0.205  #NUC5
sudo ip route add 192.168.20.0/24 via 192.168.0.206  #NUC6
sudo ip route add 192.168.21.0/24 via 192.168.0.207  #NUC7
sudo ip route add 192.168.22.0/24 via 192.168.0.208  #NUC8
sudo ip route add 192.168.23.0/24 via 192.168.0.209  #NUC9
sudo ip route add 192.168.24.0/24 via 192.168.0.210  #NUC10

sudo ip route add 192.168.100.1 via 192.168.0.201 #NUC1 on mesh
sudo ip route add 192.168.100.2 via 192.168.0.202 #NUC2 on mesh
sudo ip route add 192.168.100.3 via 192.168.0.203 #NUC3 on mesh
sudo ip route add 192.168.100.4 via 192.168.0.204 #NUC4 on mesh
sudo ip route add 192.168.100.5 via 192.168.0.205 #NUC5 on mesh
sudo ip route add 192.168.100.6 via 192.168.0.206 #NUC6 on mesh
sudo ip route add 192.168.100.7 via 192.168.0.207 #NUC7 on mesh
sudo ip route add 192.168.100.8 via 192.168.0.208 #NUC8 on mesh
sudo ip route add 192.168.100.9 via 192.168.0.209 #NUC9 on mesh
sudo ip route add 192.168.100.10 via 192.168.0.210 #NUC10 on mesh

