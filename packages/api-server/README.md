# Setup

Let's create a virtual environment named `venv` for all this Python magic:
```
sudo apt install python3-venv
python3 -m venv --system-site-packages .venv
```
It is recommended to create the virtual environment at the root of this repo so other packages can share it.

Now let's activate the Python virtual environment and install all sorts of things into it without breaking our system:
```
. .venv/bin/activate
pip3 install -e .
```

# Run development server

```bash
./run.sh
```

# Developers

The files in `api_server/models/mixins` are generated, they are committed to the repo to make distribution easier. Follow these steps to regenerate the files if needed:

```bash
pip3 install jinja2
./generate_mixins.sh
```
