[![Nightly](https://github.com/osrf/rmf-web/workflows/Nightly/badge.svg)](https://github.com/osrf/rmf-web/actions?query=workflow%3ANightly)
[![codecov](https://codecov.io/gh/osrf/rmf-web/branch/main/graph/badge.svg)](https://codecov.io/gh/osrf/rmf-web)

# Installation

On Ubuntu 20.04, we need to install Docker, NVM, and node:

### Install Docker

```
sudo apt update
sudo apt install apt-transport-https gnupg-agent
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu focal stable"
sudo apt-get install docker-ce docker-ce-cli containerd.io
sudo curl -L "https://github.com/docker/compose/releases/download/1.27.4/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose
sudo usermod -aG docker $USER
```
Unfortunately, it seems that on Ubuntu 20.04 you have to reboot the machine for the new group membership to be fully reloaded. Maybe some smaller steps are possible, but a reboot definitely works (logout/login didn't work for me, but maybe logout/login followed by restarting the docker service?). Just reboot :)

### Install NVM and Node

```
sudo apt install git
git clone https://github.com/nvm-sh/nvm.git $HOME/.nvm -b v0.37.2
. ~/.nvm/nvm.sh
nvm install 12.20.0
# build rmf_core and traffic_editor into a colcon workspace in ~/rmf
# TODO: copy those here for reference
. ~/rmf/install/setup.bash
cd ~/rmf/src/rmf-web/packages/dashboard
npm install
```

# Running the Dashboard

```
source ~/.nvm/nvm.sh
source ~/rmf/install/setup.bash
cd ~/rmf/src/rmf-web/packages/dashboard
npm start
```

# Further documentation

Please see the [README](packages/dashboard/README.md) in the dashboard package.
