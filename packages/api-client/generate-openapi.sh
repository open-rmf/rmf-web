#!/bin/bash
set -e

function usage() {
  echo "Usage: generate-models.sh"
}

cd $(dirname $0)

source ../../scripts/version.sh

expected_sha='b2d46d4990af3d442e4e228e1e627b93ca371ad972f54a7e82272b0ce7968c8b'

if [[ ! -f '.bin/openapi-generator-cli-5.2.1.jar' ]]; then
  mkdir -p .bin
  wget https://repo1.maven.org/maven2/org/openapitools/openapi-generator-cli/5.2.1/openapi-generator-cli-5.2.1.jar -O .bin/openapi-generator-cli-5.2.1.jar
fi

sha=$(sha256sum .bin/openapi-generator-cli-5.2.1.jar | awk '{print $1}')

if [[ $sha != $expected_sha ]]; then
  echo "ERR: .bin/openapi-generator-cli-5.2.1.jar sha doesn't match"
  exit 1
fi

openapi_generator_ver=$(java -jar .bin/openapi-generator-cli-5.2.1.jar version)
pipenv run python generate-openapi.py
rm -rf 'lib/openapi'
java -jar .bin/openapi-generator-cli-5.2.1.jar generate -i'build/openapi.json' -gtypescript-axios -olib/openapi -copenapi-generator.json

rmf_server_ver=$(getVersion .)

cat << EOF > lib/version.ts
// THIS FILE IS GENERATED
import { version as rmfModelVer } from 'rmf-models';

export const version = {
  rmfModels: rmfModelVer,
  rmfServer: '$rmf_server_ver',
  openapiGenerator: '$openapi_generator_ver',
};

EOF

../../node_modules/.bin/prettier -w lib
