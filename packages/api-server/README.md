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

# Run the server

```bash
rmf_api_server
```

## Configuration

Config files are python modules that export a variable named `config`. See [default_config.py](api_server/default_config.py) for an example and list of the options available. All options are REQUIRED unless specified otherwise.

Configuration is read from the file specified in the env `RMF_API_SERVER_CONFIG`, if not provided, the default config is used.

e.g.
```bash
RMF_API_SERVER_CONFIG='my_config.py' rmf_api_server
```

# Developers

## Running tests

Install the following dev dependencies

```bash
pip3 install aiohttp
```

Run the tests

```bash
python3 -m unittest
```

### Collecting code coverage

Install the following dependencies

```bash
pip3 install coverage
```

Run tests with coverage

```bash
python3 -m coverage run -m unittest
```

Generate coverage report

```bash
python3 -m coverage combine
python3 -m coverage html
xdg-open htmlcov/index.html
```

## Generated files

The files in `api_server/models/mixins` are generated, they are committed to the repo to make distribution easier. Follow these steps to regenerate the files if needed:

```bash
pip3 install jinja2
./generate_mixins.sh
```

## Live reload

```bash
uvicorn --reload api_server.app:app
```
