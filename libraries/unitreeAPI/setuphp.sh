sudo ifconfig eno1 down # eth0 is your PC Ethernet port
sudo ifconfig eno1 192.168.123.162/24
sudo ifconfig eno1 up
ping 192.168.123.161
