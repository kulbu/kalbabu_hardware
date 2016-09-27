# kulbabu_hardware

```
sudo modprobe aml_i2c
echo "aml_i2c" | sudo tee -a /etc/modules
sudo apt-get install -y i2c-tools
#sudo modprobe i2c-dev
#echo "i2c-dev" | sudo tee -a /etc/modules
sudo adduser $USER i2c
echo -e "KERNEL==\"i2c-[0-9]*\", GROUP=\"i2c\"" | sudo tee -a /etc/udev/rules.d/20-usbi2c.rules
```
