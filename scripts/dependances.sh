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
# Vérifier le nom de l’interface réseau
nmcli device status

# Ajouter une nouvelle connexion Ethernet avec IP statique
sudo nmcli con add type ethernet ifname eth0 con-name static-eth0 ipv4.addresses 10.0.0.1/24 ipv4.method manual

# (Optionnel) Ajouter une passerelle si nécessaire
sudo nmcli con mod static-eth0 ipv4.gateway 10.0.0.254

# (Optionnel) Ajouter des serveurs DNS
sudo nmcli con mod static-eth0 ipv4.dns "8.8.8.8 8.8.4.4"

# Activer la connexion
sudo nmcli con up static-eth0

# Vérifier que l’adresse est bien appliquée
ip addr show eth0
git clone https://github.com/dirennoukpo/MUTO_RL.git
cd MUTO_RL/
