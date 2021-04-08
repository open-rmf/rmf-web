# Setup

Install pipenv

```bash
pip3 install pipenv
```

If not already done so, [bootstrap](../../README.md#bootstrap) the project, you can use
```bash
npm run bootstrap -- packages/api-server
```
to bootstrap only this package.

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

## Running behind a proxy

When running behind a reverse proxy like nginx, you need to set the `public_url` option to the url where rmf-server is served on. The reverse proxy also MUST strip the prefix.

For example, if rmf-server is served on https://example.com/rmf/api/v1, `public_url` must be set to `https://example.com/rmf/api/v1` and your reverse proxy must be configured to strip the prefix such that it forwards requests from `/rmf/api/v1/something` to `/something`.

# Developers

## Running tests

### Running unit tests

```bash
npm test
```

### Collecting code coverage

```bash
npm run test:cov
```

Generate coverage report
```bash
npm run test:report
```

## Live reload

```bash
uvicorn --reload api_server.app:app
```
