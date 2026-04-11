sudo apt-get update
sudo apt-get upgrade -y
sudo apt autoremove -y
sudo dpkg --configure -a
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
tailscale ip
tailscale status
# Créer le dossier .ssh si nécessaire
mkdir -p ~/.ssh

# Ajouter ta clé publique dans authorized_keys
echo "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIEPDri4+yuTIIvRk06gAnUslqn/0Wg6Rw+Lrd95hBrVP diren.noukpo@epitech.eu" >> ~/.ssh/authorized_keys

# Sécuriser les permissions
chmod 700 ~/.ssh
chmod 600 ~/.ssh/authorized_keys

# Vérifier que la clé est bien ajoutée
cat ~/.ssh/authorized_keys
# Vérifier le nom de l’interface réseau (souvent eth0)
nmcli device status

# Ajouter une nouvelle connexion Ethernet avec IP statique
sudo nmcli con add type ethernet ifname eth0 con-name static-eth0 ipv4.addresses 10.0.0.2/24 ipv4.method manual

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
