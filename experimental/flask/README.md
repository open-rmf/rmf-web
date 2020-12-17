# setup

Let's create a virtual environment named `venv` for all this Python magic:
```
sudo apt install python3-venv
python3 -m venv venv
```

Now let's activate the Python virtual environment so we can install all sorts of things into it without breaking our system:
```
. venv/bin/activate
pip install flask python-dotenv
pip install wheel
pip install python-keycloak
```

Now run the flask+rclpy demo program in the venv, assuming you still have a shell with the venv activated in it:
```
. /opt/ros/foxy/setup.bash
./run.sh
```
