# Installation

On Ubuntu 20.04:

```
sudo apt install git
git clone https://github.com/nvm-sh/nvm.git $HOME/.nvm -b v0.37.2
. ~/.nvm/nvm.sh
nvm install 12.20.0
# follow along the rmf_core and traffic_editor repo build instructions
# TODO: copy those here for reference using colcon, etc.
. ~/rmf/install/setup.bash
cd ~/rmf/src/rmf-web/packages/dashboard
npm install
```

# Welcome

Hello

Please see the [README](packages/dashboard/README.md) in the dashboard package.
