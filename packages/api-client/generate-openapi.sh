#!/bin/bash
set -e

function usage() {
  echo "Usage: generate-models.sh"
}

cd $(dirname $0)

source ../../scripts/version.sh
openapi_generator_ver=5.4.0

expected_sha='f3ed312310e390324b33ba2ffff290ce812935207a1493ec5c098d0a441be51c'

if [[ ! -f ".bin/openapi-generator-cli-${openapi_generator_ver}.jar" ]]; then
  mkdir -p .bin
  wget https://repo1.maven.org/maven2/org/openapitools/openapi-generator-cli/${openapi_generator_ver}/openapi-generator-cli-${openapi_generator_ver}.jar -O .bin/openapi-generator-cli-${openapi_generator_ver}.jar
fi

sha=$(sha256sum .bin/openapi-generator-cli-${openapi_generator_ver}.jar | awk '{print $1}')

if [[ $sha != $expected_sha ]]; then
  echo "ERR: .bin/openapi-generator-cli-${openapi_generator_ver}.jar sha doesn't match"
  exit 1
fi

pipenv run python generate-openapi.py
rm -rf 'lib/openapi'
java -jar .bin/openapi-generator-cli-${openapi_generator_ver}.jar generate -i'build/openapi.json' -gtypescript-axios -olib/openapi -copenapi-generator.json

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

npx prettier -w lib

# generate schema
cat << EOF > schema/index.ts
export default $(cat build/openapi.json)
EOF
npx prettier -w openapi/schema
