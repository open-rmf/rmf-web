#!/bin/bash
set -e

function usage() {
  echo "Usage: generate-models.sh"
}

cd $(dirname $0)

source ../../scripts/version.sh

pipenv run python generate-openapi.py
# openapi-generator support for openapi 3.1.0 is WIP (as of 7.4.0), the code it generates are invalid.
# hacky workaround by tricking it to think it is an openapi 3.0.3 spec.
sed '0,/"openapi": "3.1.0"/s//"openapi": "3.0.3"/' -i build/openapi.json
rm -rf 'lib/openapi'
pnpm exec openapi-generator-cli generate

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

pnpm exec prettier -w lib

# generate schema
cat << EOF > schema/index.ts
export default $(cat build/openapi.json)
EOF
npx prettier -w schema
