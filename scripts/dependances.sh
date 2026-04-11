sudo apt-get update
sudo apt-get upgrade -y
sudo apt autoremove -y
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
tailscale ip
tailscale status
sudo apt install git -y
# Download and run the official Docker installation script
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
# Clean up the install script
rm get-docker.sh
# Add your user to the docker group
sudo usermod -aG docker $USER
newgrp docker
sudo apt install python3-pip -y
python3 -m venv venv
source venv/bin/activate
pip3 install Adafruit-SSD1306 Adafruit-GPIO Pillow
# python3 yahboom_oled.py
deactivate
git clone https://github.com/dirennoukpo/MUTO_RL.git
cd MUTO_RL/
