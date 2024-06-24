# Open-RMF API Server

This API server sets up the necessary endpoints with an Open-RMF deployment and enables the use of the web dashboard. The server comes with the capability of logging to databases, as well as handling authentication and permissions.

# Setup

If not already done so, [install dependencies](../../README.md#Install-dependencies), you can use
```bash
pnpm install -w --filter api-server...
```
to install dependencies for only this package.

# API

Check out the latest API definitions [here](https://open-rmf.github.io/rmf-web/docs/api-server), or visit `/docs` relative to your running server's url, e.g. `http://localhost:8000/docs`.

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
To run the api-server with PostgreSQL, assuming it has been set up to listen on 127.0.0.1:5432 with user `postgres` and password `postgres`:
```
RMF_API_SERVER_CONFIG=api_server/psql_local_config.py rmf_api_server
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

### PostgreSQL
If you would like to use PostgreSQL, you will also need to install and set it up. The defaults are for PostgreSQL to be listening on 127.0.0.1:5432.

#### Docker
We can use Docker to quickly bring up a PostgreSQL instance.

Install docker: `https://docs.docker.com/engine/install/ubuntu/`
Start a a database instance: `docker run -it --rm --name rmf-postgres --network=host -e POSTGRES_PASSWORD=postgres -d postgres`

To stop the instance: `docker kill rmf-postgres`

#### Bare Metal
Alternatively, we can install PostgreSQL 'bare metal'.
```
apt install postgresql postgresql-contrib -y
# Set a default password
sudo -u postgres psql -c "ALTER USER postgres PASSWORD 'postgres';"

sudo systemctl restart postgresql
# interactive prompt
sudo -i -u postgres
```
To manually reset the database:
```
sudo -u postgres bash -c "dropdb postgres; createdb postgres"
```

## Running behind a proxy

When running behind a reverse proxy like nginx, you need to set the `public_url` option to the url where rmf-server is served on. The reverse proxy also MUST strip the prefix.

For example, if rmf-server is served on https://example.com/rmf/api/v1, `public_url` must be set to `https://example.com/rmf/api/v1` and your reverse proxy must be configured to strip the prefix such that it forwards requests from `/rmf/api/v1/something` to `/something`.

## Running with RMF simulations

When running with rmf simulations, you need to set the env `RMF_SERVER_USE_SIM_TIME=true`. This is needed to ensure that times from the client are correctly converted to RMF's simulation time.

# Authentication and Authorization

## OpenID Connect

rmf-server does not manage user identities and access levels by itself, it uses an OpenID Connect compatible identity provider to perform authentication. Authorization however, is performed in-app by rmf-server.

### Access Token Format

OpenID Connect does not define the format of the access token, the canonical way for a resource server to validate an access token is to use the token introspection endpoint of the authentication server. In order to not have to connect to the identity provider for every request, rmf-server assumes the convention that the access token is a JWT that can be verified independently. Most modern identity providers like keycloak and auth0 follows this convention.

### Access Token Claims

The access token must include the `preferred_username` claim. It will be used to determine an user's authorization levels.

## Roles, Actions and Authorization Group

An user's permission to perform certain actions on a protected resource is determined by 3 values, role, action, and authorization group of the resource. A resource belongs to one authorization group and an user can belong to multiple roles. An user has access to perform an action on a resource if any of their roles has permission to perform the action on the authorization group which the resource belongs to. An admin always have permission to perform any action on any group.

For example, given the following permissions and users

Permissions:
| Role | Authz Group | Action |
| --- | --- | --- |
| role1 | group1 | task_submit |
| role2 | group2 | task_submit |
| role3 | group3 | task_submit |
| role4 | group1 | task_submit |
| role4 | group3 | task_submit |

Users:
| User | Roles | Is Admin |
| --- | --- | --- |
| user1 | role1 | false |
| user2 | role2, role3 | false |
| user3 | role4 | false |
| user4 | | true


`user1` will be able to submit a task that belongs to authorization group `group1`. `user2` can submit tasks belonging to `group2` and `group3`. `user3` can submit tasks in group `group1` and `group3`. `user4`, being an admin can submit tasks in any group.

The authorization group of a resource is determined automatically based on different factors according to the resource type. For example, a task's authorization group may be determined by the region where it takes place.

## Synchronization (or lack of) of User Data

rmf-server maintains its own database of users, roles and permissions. In order to keep compatibility with as many identitiy provider as possible and amount of code small, this database is never synchronized with the identity provider's database. Instead, rmf-server takes the following approach to keep things working even without a synchronized database.

* When an user first access any of the protected api, a new rmf-server user is automatically created, the  user will have no roles and no privileges.
  * rmf-server checks if the token is valid before creating the user. If it is valid, the user must exist in the identity provider.
* An admin can use the admin endpoints to perform various user management like
  * Create users
  * Create roles
  * Add/remove permissions to roles
  * Add/remove roles to users
* The admin endpoints only work on rmf-server's database and does not require delegation of any functions to the identity provider, as a result there are some cavaets
  * There is no endpoints to manage an user's authentication like reset user password, enable/disable users etc.
  * Endpoints that list/search users only includes users that is already added on to rmf-server.
* If an admin wishes to manage authorization for an user that exists in the identity provider, but not in rmf-server, they need to use the create user endpoint to create a new user with the same username.
* Deleting an user from rmf-server does not prevent them from accessing "semi-protected" apis (apis that require login but does not require any permissions). It also does not prevent them from logging into a frontend that connects to rmf-server (e.g. rmf dashboard).
  * This is because it does not delete the user from the identity provider and the next time they access any protected api, a new user will be automatically created.
  * Fully deleting an user so that they can no longer login should be done on the identity provider. There is no endpoint in rmf-server to forward deletion of an user to the identity provider because such behaviour may be unintuitive, undesirable and require specialized code paths for every provider.
* If the user is deleted from the identity provider, it will still exist in rmf-server.
  * This is harmless as the user will not be able to authenticate with the identity provider and get a valid token, so a "zombie" user will not be able to access any protected api.
* Since a new user is created with minimal privileges, and an admin is required to give users privileges (including the admin privilege), there is a problem.
  * This is worked around by adding a config to automatically make an user an admin on startup. Note that since there is no synchronization between the identity provider, there is no guarantee that such an user actually exists.

## TODO

A resource's authorizaion group is determined by it's contents. Exactly how they are determined for each type of resource is still either undecided or lacking information from RMF to be implemented, so currently every resource is put into a default empty group of `` (empty string).


## Database Migration

[`aerich`](https://github.com/tortoise/aerich) is a database migration tool for TortoiseORM. `aerich` requires a configuration file for initialization before doing any mutations, which can be found in `api_server.__main__.TORTOISE_ORM`.

Install `aerich`

```bash
pip3 install aerich
```

This migration example will be for PostgreSQL. First, setup `rmf-web` following [instructions](../../README.md). Then, run the `api-server` with `psql`,

```bash
# source RMF
cd ~/rmf-web/packages/api-server
pnpm run start:psql
```

In another terminal, activate the virtual environment manually and initialize `aerich`,

```bash
cd ~/rmf-web
source .venv/bin/activate
cd ~/rmf-web/packages/api-server

# First export the RMF_API_SERVER_CONFIG variable
export RMF_API_SERVER_CONFIG=psql_local_config.py

# Init aerich and save migration workspace to /tmp
aerich init -t api_server.__main__.TORTOISE_ORM --location /tmp/migrations
aerich init-db
```

You can check the current schema of a table. For example, using the `taskstate` table,

```bash
sudo -u postgres bash -c "psql -c '\d+ taskstate;'"
```

Now, modify the `TaskState` class in the `api_server/models/tortoise_models/tasks.py` to add a new field:

```python
new_field = fields.CharField(255)
```

Don't forget to make necessary changes to `api_server/repositories/tasks.py` too, to allow the dashboard to use the newly added fields.

Now attempt a migration, and allow `aerich` to find the changes required,

```bash
aerich migrate
# aerich will generate the migration file with the version
# check file at /tmp/migration/models/
```

Perform database upgrade,

```bash
aerich upgrade
```

Now, inspecting the database schema, you will find "new_field" available in the schema

```bash
sudo -u postgres bash -c "psql -c '\d+ taskstate;'"
```

Restart the `api-server` and the changes to the databse should be reflected.


## Running tests

### Running unit tests

```bash
pnpm test
```

By default in-memory sqlite database is used for testing, to test on another database, set the `RMF_API_SERVER_TEST_DB_URL` environment variable.

```bash
RMF_API_SERVER_TEST_DB_URL=<db_url> pnpm test
```

### Collecting code coverage

```bash
pnpm run test:cov
```

Generate coverage report
```bash
pnpm run test:report
```

## Live reload

```bash
uvicorn --reload api_server.app:app
```
