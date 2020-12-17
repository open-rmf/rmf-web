from flask import Flask
from flask import request
import config


app = Flask(__name__)
app.config.from_object('config.Config')
app.config.from_envvar('RMF_WEB_CONFIG', silent=True)

print(f'config: {app.config}')
print(f'site name: [{app.config["SITE_NAME"]}]')


@app.route('/')
def get_root():
    print(request.headers)
    return {'message': 'hello'}
