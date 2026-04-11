sudo apt-get update
sudo apt-get ugrade -y
sudo apt autoremove -y
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
tailscale ip
tailscale status
sudo apt install git -y
git clone https://github.com/dirennoukpo/MUTO_RL.git
cd MUTO_RL/
# Download and run the official Docker installation script
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
# Clean up the install script
rm get-docker.sh
# Add your user to the docker group
sudo usermod -aG docker $USER
