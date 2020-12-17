#!/usr/bin/env python3
from keycloak import KeycloakOpenID

keycloak_openid = KeycloakOpenID(
    server_url='http://localhost:8080/auth/',
    client_id='romi-dashboard',
    realm_name='master'
)

token = keycloak_openid.token('admin', 'admin')
user_id = keycloak_openid.userinfo(token['access_token'])['sub']
user_name = keycloak_openid.userinfo(token['access_token'])['preferred_username']
print(f'user ID: {user_id}')
print(f'user name: {user_name}')

#import requests
# todo: stuff the token into the request headers
#headers = {'foo': 'bar'}
#r = requests.get('http://localhost:5000', headers=headers)
#print(r.text)
