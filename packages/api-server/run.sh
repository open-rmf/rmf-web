#!/bin/bash

export FLASK_APP=api_server.app
export FLASK_ENV=development

flask run --eager-loading --port 5001
