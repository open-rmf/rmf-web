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

### Keycloak setup

* Create a new role called `read-only`
* Create a new user named `viewer`
* Assign the `read-only` role to the `viewer` user.
* Tell Keycloak to send the roles with the userinfo tokens:
  * Keycloak admin panel -> Client Scopes -> `roles` -> Mappers -> `client roles`
  * set `Client ID` to `romi-dashboard`
  * set `Token Claim Name` to `roles`
  * set `Add to userinfo` to `ON`

### keycloak + sqlite3 experiments

##### Terminal 1: Keycloak container
From `rmf-web` repo checkout directory:
```
cd packages/dashboard
npm run start:auth
```

##### Terminal 2: Flash (backend)
```
. venv/bin/activate
```

##### Terminal 3: Mock frontend (python script)
```
. venv/bin/activate
./mock_frontend.py
```

### rclpy experiments

THIS IS OUTDATED since the `.flaskenv` now points to a different app...

Now run the flask+rclpy demo program in the venv, assuming you still have a shell with the venv activated in it:
```
. /opt/ros/foxy/setup.bash
./run.sh
```
