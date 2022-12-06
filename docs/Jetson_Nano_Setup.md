# Instruction for setting up Jetson Nano SSH

## Firt Time Setup
A green LED next to the Micro-USB connector will light as soon as the developer kit powers on. When you boot the first time, the developer kit will take you through some initial setup, including:

* Review and accept NVIDIA Jetson software EULA
* Select system language, keyboard layout, and time zone
* Create username, password, and computer name
* Select APP partition size—it is recommended to use the max size suggested

## Setting up a WiFi in Jetson Nano

You can download the appropriate drivers by opening a terminal and entering the following command:

```bash
git clone https://github.com/lwfinger/rtl8723bu.git
```
Once the download is complete you can navigate into the drivers directory with the following command:
```bash
cd rtl8723bu
```

You are now in the the directory (folder) to start the install process for the drivers!

Assuming you are still in the driver directory named “rtl8723bu” type the following command:
```bash
source dkms.conf
```
Once you get the command prompt back (which should almost be instantaneous) type the following command to create a working project directory:

```bash
sudo mkdir /usr/src/$PACKAGE_NAME-$PACKAGE_VERSION
```

With the directory created, type the following to move a number of files to your working project directory:

```bash
sudo cp -r core hal include os_dep platform dkms.conf Makefile rtl8723b_fw.bin /usr/src/$PACKAGE_NAME-$PACKAGE_VERSION
```
We finally add those files to DKMS with by executing the following command:
```bash
sudo dkms add $PACKAGE_NAME/$PACKAGE_VERSION
```

Now that everything is ready and in its place we can finally install the drivers by typing the following command:

```bash
sudo dkms autoinstall $PACKAGE_NAME/$PACKAGE_VERSION
```

DKMS will take a number of actions to install the drivers including cleaning up after itself and deleting unnecessary files and directories. Once the DKMS completes the installation you should get a positive confirmation of the installation!

With the installation complete it is a good idea to reboot your Nvidia Jetson Nano with this command:

```bash
sudo reboot now
```

### Connecting to Your Network
Upon reboot of your system, you should now have WiFi connection available to you! Open a command prompt to verify a succefful driver installation by checking if you have a wireless network device installed. Open a terminal and type the following command:

```bash
ifconfig wlan0
```
To set up your connection from the command prompt you can use the NetworkManager tool from Ubuntu as outlined here. First, we will list all of our possible network connections by typing the following command:
```bash
nmcli d
```
You should get a connection listing.
Next we will make sure that the WiFi module is turned on by typing the following command:

```bash
nmcli r wifi on
```
Now we can scan and list off all visible WiFi networks available to us by typing the following command:

```bash
nmcli d wifi list
```

### Testing your connection
```bash
ping google.com
```
Should give the ping rate in a constant frequency

### Troubleshooting

If you experience intermittent WiFi connection through this adapter open a terminal window and enter the following command to turn Power Saving Mode off:

```bash
sudo iw dev wlan0 set power_save off
```

## Setting up SSH in a Jetson Nano

Once the Wifi is setup, download PuTTy and use the localhost for connecting directly over the same internet. Generally of the form `jetson.local` But will depend on what you set suring flahing the image on board.

Then Note down the IP Adress of the Nano and SSH using IP of the form `192.168.100.105`

Can also be done using a COM Port.