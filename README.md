## RoMi-dashboard

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

## Launching

Launch the web application using the commands below,

```bash
cd ~/romi-dashboard
npm start
```

or alternatively, if using a different soss or trajectory server, the follow env variables can be defined

  * REACT_APP_SOSS_SERVER
  * REACT_APP_SOSS_TOKEN
  * REACT_APP_TRAJECTORY_SERVER

and launched with

```bash
npm run start:custom
```

At this point, the dashboard should have been launched on the default browser of the machine, under `localhost:3000`.

For development, the page will reload if edits are made, while any lint errors will also appear on the console.

## Building for production

**Note: Not fully supported yet as it is missing a way to obtain the soss token.**

The following environment variables need to be defined

  * REACT_APP_SOSS_SERVER
  * REACT_APP_TRAJECTORY_SERVER

build using the command

```bash
npm run build
```

You can then serve the `build` directory using the webserver of your choice.