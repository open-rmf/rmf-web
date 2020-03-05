## ROMI-dashboard

Hello! `romi-dashboard` is a web application that provides overall visualization and control over the RoMi-H system.

## Setup

Install `npm` and `nodejs`,

```bash
sudo apt update && sudo apt install curl
curl -sL https://deb.nodesource.com/setup_13.x | sudo -E bash -
sudo apt install nodejs
```

Clone the repository and install the required packages,

```bash
cd ~/.
git clone ssh://git@github.com/osrf/romi-dashboard
cd romi-dashboard
npm install
```

## Launch

Launch the web application using the commands below,

```bash
cd ~/romi-dashboard
npm start
```

At this point, the dashboard should have been launched on the default browser of the machine, under `localhost:3000`. 

For development, the page will reload if edits are made, while any lint errors will also appear on the console.
