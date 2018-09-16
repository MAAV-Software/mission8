# Administration

This folder holds various navigation computer administration stuff.

* `mac_to_ip.map` - A MAC address to IP address map for setting up a router

## Router
Username: admin
Password: get pwnd

## Bootstrapping a New Atom

### Installing Ubuntu
Steps for installing Ubuntu 16.04 below. Quotes are used to indicate values to
put in to the installer, do not include them in typed values.

**NOTE: Ensure a heatsink is on the Atom at all times**

#### Installer
1. Use `dd` to write a Ubuntu 16.04 Server ISO to a flash drive
2. Hook up a new Atom to a power supply
3. Attach VGA, USB, Ethernet stuff to Atom
4. Boot from USB
5. Select "English" for a language
6. Select "Install Ubuntu Server"
7. Select "English - English"
8. Select "United States"
9. For "Detect keyboard layout?", select "No"
10. Select "English (US)"
11. Select "English (US)"
12. If prompted to select a network interface, choose the ethernet interface
13. For hostname, set it to "brain"
14. Full name for user - "MAAV"
15. Username for your account: "maav"
16. Choose a password for the new user: "get pwnd"
17. Enter "get pwnd" again to confirm
18. Encrypt your home directory? - "No"
19. Select your time zone: Accept default choice (fixed later)
20. Partitioning method: "Guided - use entire disk"
21. Select the non-USB drive option in the disks listed
22. Write the changes to disks? - "Yes"
23. HTTP Proxy? Leave blank and hit enter
24. Select "No automatic updates"
25. Select solely "OpenSSH Server" by moving to its box, hitting space, and
	hitting enter
26. Install the GRUB boot loader ... - "Yes"
27. Installation complete - "Continue"

#### Intel NUC7i7BHN "NUC" First Boot
1.  Log in as the "maav" user
2.  Run `sudo apt install wireless-tools` to install iwconfig
3.  Run `sudo apt install lshw` so we can list hardware
4.  Run `sudo apt install linux-generic-hwe-16.04-edge` to install wireless
    driver
5.  Edit "/etc/network/interfaces" and comment out the three lines:
    `source /etc/network/interfaces.d/*`, 
    `auto eno1`, and `iface eno1 inet dhcp`
6.  Run `sudo /etc/init.d/networking restart`
7.  Run `sudo reboot`
8.  Run `iwconfig`, and write down your interface name (should be "wlp58s0")
9.  Run `sudo ifdown <INTERFACE> && sudo ifup -v <INTERFACE>`, but actually
    use the interface name you acquired from step 8.
10. Follow steps 2 - 7 in the "Atom" First Boot instruction.

#### Axiomtek PICO831 "Atom" First Boot
1. Log in as the "maav" user
2. Run `ip a` to get the MAC address of its wireless interface. Ensure this
   MAC is in `mac_to_ip.map`, and add it to the router's allocated IP addresses
   if necessary.
3. Run "sudo apt-get install -y network-manager python"
4. Run "sudo systemctl start NetworkManager"
5. Run "sudo systemctl enable NetworkManager"
6. Run "sudo nmtui":
  1. Activate a connection
  2. Find the 'maav' network, enter the password ("impossible")
  3. Quit
  4. Run "sudo nmtui"
  4. Edit a connection
  5. Ensure that the "maav" connection is set to automatically connect and
     is available to all users
  6. Quit
7. Edit "/etc/init.d/networking restart" and comment out the two lines
   "auto eno1" and "iface eno1 inet dhcp"
8. Run "sudo poweroff"

### Configure Ubuntu
If you have not already, make sure to run `bash/vehicle-copy-keys.sh` to copy
your public SSH key to the vehicle so you can do password-less login.

From the root of the repository, run `bash/vehicle-config.sh`. This will
launch ansible and configure the vehicle. You must have internet access in
order to run this, as it creates a python virtualenv and installs ansible.

## Deploy the Code
Run `bash/vehicle-deploy.sh`.
