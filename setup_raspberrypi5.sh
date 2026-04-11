sudo apt-get update
sudo apt-get ugrade -y
sudo apt autoremove -y
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
tailscale ip
tailscale status
