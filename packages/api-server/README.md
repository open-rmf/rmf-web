# Setup

Install pipenv and lerna

```bash
pip3 install pipenv
npm install -g lerna@4
```

If not already done so, [bootstrap](../../README.md#bootstrap) the project, you can use
```bash
lerna bootstrap --scope=api-server
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

## Supported databases

rmf-server uses [tortoise-orm](https://github.com/tortoise/tortoise-orm/) to perform database operations. Currently, the supported databases are

* PostgreSQL
* SQLite
* MySQL
* MariaDB

by default it uses a in-memory sqlite instance, to use other databases, install rmf-server with the relevalent extras

* PostgreSQL - postgres
* MySQL - mysql
* MariaDB - maria

.e.g.

```bash
pip3 install rmf-server[postgres]
```

Then in your config, set the `db_url` accordingly, the url should be in the form

```
DB_TYPE://USERNAME:PASSWORD@HOST:PORT/DB_NAME?PARAM1=value&PARAM2=value
```

for example, to connect to postgres

```
postgres://<user>:<password>@<host>/<database>
```

for more information, see https://tortoise-orm.readthedocs.io/en/latest/databases.html.

## Running behind a proxy

When running behind a reverse proxy like nginx, you need to set the `public_url` option to the url where rmf-server is served on. The reverse proxy also MUST strip the prefix.

For example, if rmf-server is served on https://example.com/rmf/api/v1, `public_url` must be set to `https://example.com/rmf/api/v1` and your reverse proxy must be configured to strip the prefix such that it forwards requests from `/rmf/api/v1/something` to `/something`.

## Running with RMF simulations

When running with rmf simulations, you need to set the env `RMF_SERVER_USE_SIM_TIME=true`. This is needed to ensure that times from the client are correctly converted to RMF's simulation time.

# Developers

## About FastIO

FastIO is a wrapper between socket.io and fastapi to automatically unify the endpoints. It adds a new asgi app class `FastIO` which contains both a FastAPI app and socket.io app inside. A new `FastIORouter` is also added that can be used with the new `FastIO` app similar to the default FastAPI router. The main functionality `FastIORouter` adds is a new `watch` method, registering an endpoint with the `watch` method automatically adds both a socket.io endpoint and a rest GET endpoint. The GET endpoint will automatically returns the latest response from the socket.io endpoint.

Here is an example of how the FastIO works

```python
        import rx.subject
        import uvicorn

        from api_server.fast_io import FastIORouter

        class ReturnVideo(pydantic.BaseModel):
            film_title: str

        app = FastIO()
        router = FastIORouter()
        target = rx.subject.Subject()


        @router.watch("/video_rental/{film_title}/available", target)
        def watch_availability(rental: dict):
            return {"film_title": rental["film"]}, rental


        @router.post("/video_rental/return_video")
        def post_return_video(return_video: ReturnVideo):
            target.on_next({"film": return_video.film_title, "available": True})


        app.include_router(router)

        uvicorn.run(app)
```

This registers a socket.io endpoint, a GET endpoint at `/video_rental/{film_title}/available` and a POST endpoint at `/video_rental/return_video`. When a request is received in the POST endpoint, it triggers an event in the target observable, when this happens, FastIO will call the function registered in the watch decorator (`watch_availability` in this case), the function should return a tuple containing a dict mapping the path parameters and the response to send. FastIO uses this to map the event into a socket.io room and automatically sends a message to it, it also remembers the last message sent, requests to the GET endpoint will automatically return the last event sent.

An example flow:
1. Received a POST request at `/video_rental/return_video`, containing body `{"film_title": "inception"}`.
2. Triggers an event in the target observable, containing the returned film title.
3. FastIO calls `watch_availability` to get the path mapping, in this case, the path param `film_title` is mapped to `inception`.
4. FastIO uses the path param mappings to send an event to the room `/video_rental/inception/available`. It also remembers the last event sent to each room.
5. When a GET request is received at `/video_rental/inception/available`, FastIO looks at the last sent event and automatically response with that.

In order to keep the bandwidth down, FastIO will not automatically add a client to any room, the client have to send an event to the `subscribe` room first.

e.g.
```js
sio.emit('subscribe', { path: '/video_rental/inception/available' });
sio.on('/video_rental/inception/available', console.log);
```

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
