#!/bin/bash
set -e

function usage() {
  echo "Usage: generate-models.sh"
}

cd $(dirname $0)

source ../../scripts/version.sh

pipenv run python generate-openapi.py
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

npx prettier -w lib

# generate schema
cat << EOF > schema/index.ts
export default $(cat build/openapi.json)
EOF
npx prettier -w schema
