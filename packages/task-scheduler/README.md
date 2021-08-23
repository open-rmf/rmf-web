# Description

This package is responsible for generating pre-scheduled tasks. This module exposes REST endpoints for creating and managing pre-scheduled tasks. The idea is that the API-server (or rmf-server) serves as a bridge between this module and the UI. This module should also be connected to the RMF task manager to send the task in a specific time.

Among the supported schedule are:

* At a specific time
* At a specific time on a daily schedule
* At a specific time on a weekly schedule
* At a specific time on a monthly schedule
* At a specific time on a weekday monthly schedule

It supports:
Create task rules.
Allow enable / disable specific tasks
Delete rules and specific tasks
Add additional Information to a rule task.

In the following picture we can see how this package will interact with the whole system
![scheduler](https://user-images.githubusercontent.com/11761240/125002827-ca73dc80-e023-11eb-834f-3ff439a576ea.png)


# Setup

Install pipenv

```bash
pip3 install pipenv
```

If not already done so, [bootstrap](../../README.md#bootstrap) the project, you can use

```bash
lerna bootstrap --scope=task-scheduler
```

to bootstrap only this package.

# Run the server

```bash
task_scheduler
```

For development we recommend running this command:

```bash
uvicorn --reload task_scheduler.app:get_app
```

## Configuration

Config files are python modules that export a variable named `config`. See [default_config.py](rest_server/default_config.py) for an example and list of the options available. All options are REQUIRED unless specified otherwise.

Configuration is read from the file specified in the env `RMF_TASK_SCHEDULER_CONFIG`, if not provided, the default config is used.

e.g.
```bash
RMF_TASK_SCHEDULER_CONFIG='my_config.py' task_scheduler
```


## Supported databases

`task-scheduler` uses [tortoise-orm](https://github.com/tortoise/tortoise-orm/) to perform database operations. Currently, the supported databases are

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
pip3 install task-scheduler[postgres]
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


# Developers

## Running tests

### Running unit tests

```bash
npm run test
```

### Collecting code coverage

```bash
npm run test:cov
```

Generate coverage report

``` bash
npm run test:report
```

## Live reload

```bash
uvicorn --reload task_scheduler.app:get_app
```
