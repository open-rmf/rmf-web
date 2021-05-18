#!/bin/bash
set -e

function usage() {
  echo "Usage: generate-models.sh"
}

cd $(dirname $0)

source ../../scripts/version.sh

expected_sha='cde6255246cf76b9e4c20e85968d36505930700f8adad68e76c0bc5a6721bfe5'

if [[ ! -f '.bin/swagger-codegen-cli.jar' ]]; then
  mkdir -p .bin
  wget https://repo1.maven.org/maven2/io/swagger/codegen/v3/swagger-codegen-cli/3.0.25/swagger-codegen-cli-3.0.25.jar -O .bin/swagger-codegen-cli.jar
fi

sha=$(sha256sum .bin/swagger-codegen-cli.jar | awk '{print $1}')

if [[ $sha != $expected_sha ]]; then
  echo "ERR: .bin/swagger-codegen-cli.jar sha doesn't match"
  exit 1
fi

swagger_ver=$(java -jar .bin/swagger-codegen-cli.jar version)
pipenv run python generate-openapi.py
rm -r 'lib/openapi'
java -jar .bin/swagger-codegen-cli.jar generate -i'build/openapi.json' -ltypescript-axios -olib/openapi -cswagger-codegen.json
# There is a bug with `ModelObject` type being missing, workaround it by adding a type to the generated models.
echo 'export type ModelObject = Record<string, any>;' >> lib/openapi/models/index.ts

rmf_server_ver=$(getVersion .)

cat << EOF > lib/version.ts
// THIS FILE IS GENERATED
import { version as rmfModelVer } from 'rmf-models';

export const version = {
  rmfModels: rmfModelVer,
  rmfServer: '$rmf_server_ver',
  swaggerCodegen: '$swagger_ver',
};

EOF

../../node_modules/.bin/prettier -w lib
