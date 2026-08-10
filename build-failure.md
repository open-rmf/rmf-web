```
#13 [stage-0 6/7] RUN cd /ws   && (pnpm install --filter rmf-dashboard-framework... || (pnpm approve-builds --all && pnpm install --filter rmf-dashboard-framework...))
#13 0.695 Scope: 5 of 8 workspace projects
#13 0.847 ? Verifying lockfile against supply-chain policies (1420 entries)...
#13 0.865 Lockfile is up to date, resolution step is skipped
#13 1.044 Progress: resolved 1, reused 0, downloaded 0, added 0
#13 1.185 .                                        | +919 ++++++++++++++++++++++++++++++++
#13 2.047 Progress: resolved 919, reused 0, downloaded 0, added 0
#13 3.156 Progress: resolved 919, reused 0, downloaded 1, added 0
#13 4.172 Progress: resolved 919, reused 0, downloaded 48, added 0
#13 5.181 Progress: resolved 919, reused 0, downloaded 60, added 0
#13 5.481 Packages are hard linked from the content-addressable store to the virtual store.
#13 5.481   Content-addressable store is at: /root/.local/share/pnpm/store/v11
#13 5.481   Virtual store is at:             node_modules/.pnpm
#13 6.222 Progress: resolved 919, reused 0, downloaded 88, added 4
#13 7.077 ✓ Lockfile passes supply-chain policies (1420 entries in 6.2s)
#13 7.223 Progress: resolved 919, reused 0, downloaded 135, added 8
#13 8.223 Progress: resolved 919, reused 0, downloaded 308, added 20
#13 9.228 Progress: resolved 919, reused 0, downloaded 585, added 38
#13 10.23 Progress: resolved 919, reused 0, downloaded 903, added 115
#13 11.23 Progress: resolved 919, reused 0, downloaded 903, added 918
#13 11.95 Progress: resolved 919, reused 0, downloaded 903, added 919, done
#13 12.60 .../node_modules/@swc/core postinstall$ node postinstall.js
#13 12.62 .../node_modules/@nestjs/core postinstall$ opencollective || exit 0
#13 12.64 .../esbuild@0.27.7/node_modules/esbuild postinstall$ node install.js
#13 12.67 .../node_modules/@swc/core postinstall: Done
#13 12.72 .../esbuild@0.27.7/node_modules/esbuild postinstall: Done
#13 12.82 .../node_modules/@nestjs/core postinstall:                            Thanks for installing nest
#13 12.82 .../node_modules/@nestjs/core postinstall:                  Please consider donating to our open collective
#13 12.82 .../node_modules/@nestjs/core postinstall:                         to help us maintain this package.
#13 12.82 .../node_modules/@nestjs/core postinstall:
#13 12.82 .../node_modules/@nestjs/core postinstall:                         Number of contributors: undefined
#13 12.82 .../node_modules/@nestjs/core postinstall:                              Number of backers: 1213
#13 12.83 .../node_modules/@nestjs/core postinstall:                              Annual budget: $130,427
#13 12.83 .../node_modules/@nestjs/core postinstall:                              Current balance: $8,113
#13 12.83 .../node_modules/@nestjs/core postinstall:
#13 12.83 .../node_modules/@nestjs/core postinstall:              Become a partner: https://opencollective.com/nest/donate
#13 12.83 .../node_modules/@nestjs/core postinstall:
#13 12.87 .../node_modules/@nestjs/core postinstall: Done
#13 12.91 .../@openapitools/openapi-generator-cli postinstall$ opencollective || exit 0
#13 13.13 .../@openapitools/openapi-generator-cli postinstall:                      Thanks for installing openapi_generator
#13 13.13 .../@openapitools/openapi-generator-cli postinstall:                  Please consider donating to our open collective
#13 13.13 .../@openapitools/openapi-generator-cli postinstall:                         to help us maintain this package.
#13 13.13 .../@openapitools/openapi-generator-cli postinstall:
#13 13.13 .../@openapitools/openapi-generator-cli postinstall:                         Number of contributors: undefined
#13 13.13 .../@openapitools/openapi-generator-cli postinstall:                               Number of backers: 242
#13 13.15 .../@openapitools/openapi-generator-cli postinstall:                               Annual budget: $17,916
#13 13.15 .../@openapitools/openapi-generator-cli postinstall:                              Current balance: $58,365
#13 13.16 .../@openapitools/openapi-generator-cli postinstall:
#13 13.16 .../@openapitools/openapi-generator-cli postinstall:   Please sponsor OpenAPI Generator. https://opencollective.com/openapi_generator/donate
#13 13.16 .../@openapitools/openapi-generator-cli postinstall:
#13 13.17 .../@openapitools/openapi-generator-cli postinstall: Done
#13 13.30 pipenv-install install$ ./bootstrap-pipenv.sh && ../.venv/bin/pipenv install -d --site-packages
#13 13.33 pipenv-install install: creating virtualenv at .venv
#13 13.35 . prepare$ husky
#13 13.40 . prepare: .git can't be found
#13 13.40 . prepare: Done
#13 16.11 pipenv-install install: Collecting pipenv
#13 16.24 pipenv-install install:   Downloading pipenv-2026.6.2-py3-none-any.whl.metadata (18 kB)
#13 16.28 pipenv-install install: Collecting certifi (from pipenv)
#13 16.33 pipenv-install install:   Downloading certifi-2026.7.22-py3-none-any.whl.metadata (2.5 kB)
#13 16.36 pipenv-install install: Collecting packaging>=22 (from pipenv)
#13 16.40 pipenv-install install:   Downloading packaging-26.2-py3-none-any.whl.metadata (3.5 kB)
#13 16.54 pipenv-install install: Collecting setuptools>=67 (from pipenv)
#13 16.58 pipenv-install install:   Downloading setuptools-83.0.0-py3-none-any.whl.metadata (6.6 kB)
#13 16.65 pipenv-install install: Collecting virtualenv>=20.24.2 (from pipenv)
#13 16.69 pipenv-install install:   Downloading virtualenv-21.7.1-py3-none-any.whl.metadata (3.5 kB)
#13 16.73 pipenv-install install: Collecting distlib<1,>=0.3.7 (from virtualenv>=20.24.2->pipenv)
#13 16.77 pipenv-install install:   Downloading distlib-0.4.3-py2.py3-none-any.whl.metadata (5.3 kB)
#13 16.81 pipenv-install install: Collecting filelock<4,>=3.24.2 (from virtualenv>=20.24.2->pipenv)
#13 16.85 pipenv-install install:   Downloading filelock-3.32.2-py3-none-any.whl.metadata (2.0 kB)
#13 16.89 pipenv-install install: Collecting platformdirs<5,>=3.9.1 (from virtualenv>=20.24.2->pipenv)
#13 16.93 pipenv-install install:   Downloading platformdirs-4.11.0-py3-none-any.whl.metadata (5.5 kB)
#13 16.97 pipenv-install install: Collecting python-discovery>=1.4.2 (from virtualenv>=20.24.2->pipenv)
#13 17.01 pipenv-install install:   Downloading python_discovery-1.5.1-py3-none-any.whl.metadata (5.0 kB)
#13 17.07 pipenv-install install: Downloading pipenv-2026.6.2-py3-none-any.whl (2.3 MB)
#13 17.39 pipenv-install install:    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 2.3/2.3 MB 10.3 MB/s eta 0:00:00
#13 17.44 pipenv-install install: Downloading packaging-26.2-py3-none-any.whl (100 kB)
#13 17.48 pipenv-install install: Downloading setuptools-83.0.0-py3-none-any.whl (1.0 MB)
#13 17.50 pipenv-install install:    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 1.0/1.0 MB 66.6 MB/s eta 0:00:00
#13 17.54 pipenv-install install: Downloading virtualenv-21.7.1-py3-none-any.whl (5.5 MB)
#13 17.64 pipenv-install install:    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 5.5/5.5 MB 55.0 MB/s eta 0:00:00
#13 17.68 pipenv-install install: Downloading distlib-0.4.3-py2.py3-none-any.whl (470 kB)
#13 17.72 pipenv-install install: Downloading filelock-3.32.2-py3-none-any.whl (98 kB)
#13 17.77 pipenv-install install: Downloading platformdirs-4.11.0-py3-none-any.whl (23 kB)
#13 17.81 pipenv-install install: Downloading python_discovery-1.5.1-py3-none-any.whl (35 kB)
#13 17.85 pipenv-install install: Downloading certifi-2026.7.22-py3-none-any.whl (136 kB)
#13 17.94 pipenv-install install: Installing collected packages: distlib, setuptools, platformdirs, packaging, filelock, certifi, python-discovery, virtualenv, pipenv
#13 19.95 pipenv-install install: Successfully installed certifi-2026.7.22 distlib-0.4.3 filelock-3.32.2 packaging-26.2 pipenv-2026.6.2 platformdirs-4.11.0 python-discovery-1.5.1 setuptools-83.0.0 virtualenv-21.7.1
#13 20.31 pipenv-install install: To activate this project's virtualenv, run pipenv shell.
#13 20.31 pipenv-install install: Alternatively, run a command inside the virtualenv with pipenv run.
#13 20.32 pipenv-install install: Installing dependencies from Pipfile.lock (d78544)...
#13 20.33 pipenv-install install: Installing dependencies from Pipfile.lock (d78544)...
#13 45.31 pipenv-install install: Collecting aiofiles==23.2.1 (from -r
#13 45.31 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.31 pipenv-install install: 8-hashed-reqs.txt (line 1))
#13 45.31 pipenv-install install:   Downloading aiofiles-23.2.1-py3-none-any.whl (15 kB)
#13 45.31 pipenv-install install: Collecting aiosqlite==0.20.0 (from -r
#13 45.31 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.31 pipenv-install install: 8-hashed-reqs.txt (line 2))
#13 45.31 pipenv-install install:   Downloading aiosqlite-0.20.0-py3-none-any.whl (15 kB)
#13 45.31 pipenv-install install: Collecting annotated-types==0.7.0 (from -r
#13 45.31 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.31 pipenv-install install: 8-hashed-reqs.txt (line 3))
#13 45.31 pipenv-install install:   Downloading annotated_types-0.7.0-py3-none-any.whl (13 kB)
#13 45.31 pipenv-install install: Collecting anyio==4.13.0 (from -r
#13 45.31 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.31 pipenv-install install: 8-hashed-reqs.txt (line 4))
#13 45.31 pipenv-install install:   Downloading anyio-4.13.0-py3-none-any.whl (114 kB)
#13 45.31 pipenv-install install: Requirement already satisfied: argcomplete==3.6.3 in
#13 45.31 pipenv-install install: /usr/lib/python3/dist-packages (from -r
#13 45.31 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.31 pipenv-install install: 8-hashed-reqs.txt (line 5)) (3.6.3)
#13 45.31 pipenv-install install: Collecting astroid==3.1.0 (from -r
#13 45.31 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.31 pipenv-install install: 8-hashed-reqs.txt (line 6))
#13 45.31 pipenv-install install:   Downloading astroid-3.1.0-py3-none-any.whl (275 kB)
#13 45.31 pipenv-install install: Collecting asyncpg==0.29.0 (from -r
#13 45.31 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.31 pipenv-install install: 8-hashed-reqs.txt (line 7))
#13 45.31 pipenv-install install:   Downloading asyncpg-0.29.0.tar.gz (820 kB)
#13 45.32 pipenv-install install:      ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 820.5/820.5 kB 22.0 MB/s  0:00:00
#13 45.32 pipenv-install install:   Installing build dependencies: started
#13 45.32 pipenv-install install:   Installing build dependencies: finished with status 'done'
#13 45.32 pipenv-install install:   Getting requirements to build wheel: started
#13 45.32 pipenv-install install:   Getting requirements to build wheel: finished with status 'done'
#13 45.32 pipenv-install install:   Preparing metadata (pyproject.toml): started
#13 45.32 pipenv-install install:   Preparing metadata (pyproject.toml): finished with status 'done'
#13 45.32 pipenv-install install: Collecting bidict==0.23.1 (from -r
#13 45.32 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.32 pipenv-install install: 8-hashed-reqs.txt (line 8))
#13 45.32 pipenv-install install:   Downloading bidict-0.23.1-py3-none-any.whl (32 kB)
#13 45.32 pipenv-install install: Collecting black==26.3.1 (from -r
#13 45.32 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.32 pipenv-install install: 8-hashed-reqs.txt (line 9))
#13 45.32 pipenv-install install:   Downloading
#13 45.32 pipenv-install install: black-26.3.1-cp314-cp314-manylinux2014_x86_64.manylinux_2_17_x86_64.manylinux_2_
#13 45.32 pipenv-install install: 28_x86_64.whl (1.8 MB)
#13 45.32 pipenv-install install:      ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 1.8/1.8 MB 34.3 MB/s  0:00:00
#13 45.32 pipenv-install install: Collecting certifi==2026.4.22 (from -r
#13 45.32 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.32 pipenv-install install: 8-hashed-reqs.txt (line 10))
#13 45.32 pipenv-install install:   Downloading certifi-2026.4.22-py3-none-any.whl (135 kB)
#13 45.32 pipenv-install install: Collecting cffi==2.0.0 (from -r
#13 45.32 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.32 pipenv-install install: 8-hashed-reqs.txt (line 11))
#13 45.32 pipenv-install install:   Downloading
#13 45.32 pipenv-install install: cffi-2.0.0-cp314-cp314-manylinux2014_x86_64.manylinux_2_17_x86_64.whl (219 kB)
#13 45.32 pipenv-install install: Collecting charset-normalizer==3.4.7 (from -r
#13 45.32 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.32 pipenv-install install: 8-hashed-reqs.txt (line 12))
#13 45.32 pipenv-install install:   Downloading
#13 45.32 pipenv-install install: charset_normalizer-3.4.7-cp314-cp314-manylinux2014_x86_64.manylinux_2_17_x86_64.
#13 45.32 pipenv-install install: manylinux_2_28_x86_64.whl (215 kB)
#13 45.32 pipenv-install install: Collecting click==8.3.3 (from -r
#13 45.32 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.32 pipenv-install install: 8-hashed-reqs.txt (line 13))
#13 45.32 pipenv-install install:   Downloading click-8.3.3-py3-none-any.whl (110 kB)
#13 45.32 pipenv-install install: Collecting coverage==7.14.0 (from -r
#13 45.32 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.32 pipenv-install install: 8-hashed-reqs.txt (line 14))
#13 45.33 pipenv-install install:   Downloading
#13 45.33 pipenv-install install: coverage-7.14.0-cp314-cp314-manylinux1_x86_64.manylinux_2_28_x86_64.manylinux_2_
#13 45.33 pipenv-install install: 5_x86_64.whl (253 kB)
#13 45.33 pipenv-install install: Collecting cryptography==48.0.0 (from -r
#13 45.33 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.33 pipenv-install install: 8-hashed-reqs.txt (line 15))
#13 45.33 pipenv-install install:   Downloading cryptography-48.0.0-cp311-abi3-manylinux_2_34_x86_64.whl (4.7 MB)
#13 45.33 pipenv-install install:      ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 4.7/4.7 MB 71.1 MB/s  0:00:00
#13 45.33 pipenv-install install: Collecting datamodel-code-generator==0.25.7 (from -r
#13 45.33 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.33 pipenv-install install: 8-hashed-reqs.txt (line 16))
#13 45.33 pipenv-install install:   Downloading datamodel_code_generator-0.25.7-py3-none-any.whl (108 kB)
#13 45.33 pipenv-install install: Collecting dill==0.4.1 (from -r
#13 45.33 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.33 pipenv-install install: 8-hashed-reqs.txt (line 17))
#13 45.33 pipenv-install install:   Downloading dill-0.4.1-py3-none-any.whl (120 kB)
#13 45.33 pipenv-install install: Collecting dnspython==2.8.0 (from -r
#13 45.33 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.33 pipenv-install install: 8-hashed-reqs.txt (line 18))
#13 45.33 pipenv-install install:   Downloading dnspython-2.8.0-py3-none-any.whl (331 kB)
#13 45.33 pipenv-install install: Collecting email-validator==2.3.0 (from -r
#13 45.33 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.33 pipenv-install install: 8-hashed-reqs.txt (line 19))
#13 45.33 pipenv-install install:   Downloading email_validator-2.3.0-py3-none-any.whl (35 kB)
#13 45.33 pipenv-install install: Collecting fastapi==0.109.2 (from -r
#13 45.33 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.33 pipenv-install install: 8-hashed-reqs.txt (line 20))
#13 45.33 pipenv-install install:   Downloading fastapi-0.109.2-py3-none-any.whl (92 kB)
#13 45.33 pipenv-install install: Collecting freezegun==1.5.5 (from -r
#13 45.33 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.33 pipenv-install install: 8-hashed-reqs.txt (line 21))
#13 45.33 pipenv-install install:   Downloading freezegun-1.5.5-py3-none-any.whl (19 kB)
#13 45.33 pipenv-install install: Collecting genson==1.3.0 (from -r
#13 45.33 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.33 pipenv-install install: 8-hashed-reqs.txt (line 22))
#13 45.34 pipenv-install install:   Downloading genson-1.3.0-py3-none-any.whl (21 kB)
#13 45.34 pipenv-install install: Collecting h11==0.16.0 (from -r
#13 45.34 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.34 pipenv-install install: 8-hashed-reqs.txt (line 23))
#13 45.34 pipenv-install install:   Downloading h11-0.16.0-py3-none-any.whl (37 kB)
#13 45.34 pipenv-install install: Collecting httpcore==1.0.9 (from -r
#13 45.34 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.34 pipenv-install install: 8-hashed-reqs.txt (line 24))
#13 45.34 pipenv-install install:   Downloading httpcore-1.0.9-py3-none-any.whl (78 kB)
#13 45.34 pipenv-install install: Collecting httptools==0.7.1 (from -r
#13 45.34 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.34 pipenv-install install: 8-hashed-reqs.txt (line 25))
#13 45.34 pipenv-install install:   Downloading
#13 45.34 pipenv-install install: httptools-0.7.1-cp314-cp314-manylinux1_x86_64.manylinux_2_28_x86_64.manylinux_2_
#13 45.34 pipenv-install install: 5_x86_64.whl (472 kB)
#13 45.34 pipenv-install install: Collecting httpx==0.26.0 (from -r
#13 45.34 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.34 pipenv-install install: 8-hashed-reqs.txt (line 26))
#13 45.34 pipenv-install install:   Downloading httpx-0.26.0-py3-none-any.whl (75 kB)
#13 45.34 pipenv-install install: Collecting idna==3.15 (from -r
#13 45.34 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.34 pipenv-install install: 8-hashed-reqs.txt (line 27))
#13 45.34 pipenv-install install:   Downloading idna-3.15-py3-none-any.whl (72 kB)
#13 45.34 pipenv-install install: Collecting inflect==5.6.2 (from -r
#13 45.34 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.34 pipenv-install install: 8-hashed-reqs.txt (line 28))
#13 45.34 pipenv-install install:   Downloading inflect-5.6.2-py3-none-any.whl (33 kB)
#13 45.34 pipenv-install install: Collecting iso8601==2.1.0 (from -r
#13 45.34 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.34 pipenv-install install: 8-hashed-reqs.txt (line 29))
#13 45.34 pipenv-install install:   Downloading iso8601-2.1.0-py3-none-any.whl (7.5 kB)
#13 45.34 pipenv-install install: Collecting isort==5.13.2 (from -r
#13 45.34 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.34 pipenv-install install: 8-hashed-reqs.txt (line 30))
#13 45.34 pipenv-install install:   Downloading isort-5.13.2-py3-none-any.whl (92 kB)
#13 45.34 pipenv-install install: Requirement already satisfied: jinja2==3.1.6 in /usr/lib/python3/dist-packages
#13 45.34 pipenv-install install: (from -r
#13 45.34 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.34 pipenv-install install: 8-hashed-reqs.txt (line 31)) (3.1.6)
#13 45.35 pipenv-install install: Requirement already satisfied: markupsafe==3.0.3 in
#13 45.35 pipenv-install install: /usr/lib/python3/dist-packages (from -r
#13 45.35 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.35 pipenv-install install: 8-hashed-reqs.txt (line 32)) (3.0.3)
#13 45.35 pipenv-install install: Requirement already satisfied: mccabe==0.7.0 in /usr/lib/python3/dist-packages
#13 45.35 pipenv-install install: (from -r
#13 45.35 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.35 pipenv-install install: 8-hashed-reqs.txt (line 33)) (0.7.0)
#13 45.35 pipenv-install install: Requirement already satisfied: mypy-extensions==1.1.0 in
#13 45.35 pipenv-install install: /usr/lib/python3/dist-packages (from -r
#13 45.35 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.35 pipenv-install install: 8-hashed-reqs.txt (line 34)) (1.1.0)
#13 45.35 pipenv-install install: Collecting pathspec==1.1.1 (from -r
#13 45.35 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.35 pipenv-install install: 8-hashed-reqs.txt (line 35))
#13 45.35 pipenv-install install:   Downloading pathspec-1.1.1-py3-none-any.whl (57 kB)
#13 45.35 pipenv-install install: Collecting platformdirs==4.9.6 (from -r
#13 45.35 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.35 pipenv-install install: 8-hashed-reqs.txt (line 36))
#13 45.35 pipenv-install install:   Downloading platformdirs-4.9.6-py3-none-any.whl (21 kB)
#13 45.35 pipenv-install install: Collecting pycparser==3.0 (from -r
#13 45.35 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.35 pipenv-install install: 8-hashed-reqs.txt (line 37))
#13 45.35 pipenv-install install:   Downloading pycparser-3.0-py3-none-any.whl (48 kB)
#13 45.35 pipenv-install install: Collecting pydantic==2.8.2 (from pydantic==2.8.2->-r
#13 45.35 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.35 pipenv-install install: 8-hashed-reqs.txt (line 38))
#13 45.35 pipenv-install install:   Downloading pydantic-2.8.2-py3-none-any.whl (423 kB)
#13 45.35 pipenv-install install: Collecting pydantic-core==2.20.1 (from -r
#13 45.35 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.35 pipenv-install install: 8-hashed-reqs.txt (line 39))
#13 45.35 pipenv-install install:   Downloading pydantic_core-2.20.1.tar.gz (388 kB)
#13 45.35 pipenv-install install:   Installing build dependencies: started
#13 45.35 pipenv-install install:   Installing build dependencies: finished with status 'done'
#13 45.35 pipenv-install install:   Getting requirements to build wheel: started
#13 45.36 pipenv-install install:   Getting requirements to build wheel: finished with status 'done'
#13 45.36 pipenv-install install:   Installing backend dependencies: started
#13 45.36 pipenv-install install:   Installing backend dependencies: finished with status 'done'
#13 45.36 pipenv-install install:   Preparing metadata (pyproject.toml): started
#13 45.36 pipenv-install install:   Preparing metadata (pyproject.toml): finished with status 'done'
#13 45.36 pipenv-install install: Collecting pyjwt==2.8.0 (from pyjwt==2.8.0->-r
#13 45.36 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.36 pipenv-install install: 8-hashed-reqs.txt (line 40))
#13 45.36 pipenv-install install:   Downloading PyJWT-2.8.0-py3-none-any.whl (22 kB)
#13 45.36 pipenv-install install: Collecting pylint==3.1.0 (from -r
#13 45.36 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.36 pipenv-install install: 8-hashed-reqs.txt (line 41))
#13 45.36 pipenv-install install:   Downloading pylint-3.1.0-py3-none-any.whl (515 kB)
#13 45.36 pipenv-install install: Collecting pypika-tortoise==0.2.2 (from -r
#13 45.36 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.36 pipenv-install install: 8-hashed-reqs.txt (line 42))
#13 45.36 pipenv-install install:   Downloading pypika_tortoise-0.2.2-py3-none-any.whl (50 kB)
#13 45.36 pipenv-install install: Collecting python-dateutil==2.9.0.post0 (from -r
#13 45.36 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.36 pipenv-install install: 8-hashed-reqs.txt (line 43))
#13 45.36 pipenv-install install:   Downloading python_dateutil-2.9.0.post0-py2.py3-none-any.whl (229 kB)
#13 45.36 pipenv-install install: Collecting python-dotenv==1.2.2 (from -r
#13 45.36 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.36 pipenv-install install: 8-hashed-reqs.txt (line 44))
#13 45.36 pipenv-install install:   Downloading python_dotenv-1.2.2-py3-none-any.whl (22 kB)
#13 45.36 pipenv-install install: Collecting python-engineio==4.13.1 (from -r
#13 45.36 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.36 pipenv-install install: 8-hashed-reqs.txt (line 45))
#13 45.36 pipenv-install install:   Downloading python_engineio-4.13.1-py3-none-any.whl (59 kB)
#13 45.36 pipenv-install install: Collecting python-socketio==5.11.4 (from -r
#13 45.36 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.36 pipenv-install install: 8-hashed-reqs.txt (line 46))
#13 45.36 pipenv-install install:   Downloading python_socketio-5.11.4-py3-none-any.whl (76 kB)
#13 45.37 pipenv-install install: Collecting pytokens==0.4.1 (from -r
#13 45.37 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.37 pipenv-install install: 8-hashed-reqs.txt (line 47))
#13 45.37 pipenv-install install:   Downloading
#13 45.37 pipenv-install install: pytokens-0.4.1-cp314-cp314-manylinux2014_x86_64.manylinux_2_17_x86_64.manylinux_
#13 45.37 pipenv-install install: 2_28_x86_64.whl (268 kB)
#13 45.37 pipenv-install install: Collecting pytz==2026.2 (from -r
#13 45.37 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.37 pipenv-install install: 8-hashed-reqs.txt (line 48))
#13 45.37 pipenv-install install:   Downloading pytz-2026.2-py2.py3-none-any.whl (510 kB)
#13 45.37 pipenv-install install: Requirement already satisfied: pyyaml==6.0.3 in /usr/lib/python3/dist-packages
#13 45.37 pipenv-install install: (from -r
#13 45.37 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.37 pipenv-install install: 8-hashed-reqs.txt (line 49)) (6.0.3)
#13 45.37 pipenv-install install: Collecting reactivex==4.0.4 (from -r
#13 45.37 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.37 pipenv-install install: 8-hashed-reqs.txt (line 50))
#13 45.37 pipenv-install install:   Downloading reactivex-4.0.4-py3-none-any.whl (217 kB)
#13 45.37 pipenv-install install: Collecting requests==2.34.2 (from -r
#13 45.37 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.37 pipenv-install install: 8-hashed-reqs.txt (line 51))
#13 45.37 pipenv-install install:   Downloading requests-2.34.2-py3-none-any.whl (73 kB)
#13 45.37 pipenv-install install: Collecting schedule==1.2.2 (from -r
#13 45.37 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.37 pipenv-install install: 8-hashed-reqs.txt (line 52))
#13 45.37 pipenv-install install:   Downloading schedule-1.2.2-py3-none-any.whl (12 kB)
#13 45.37 pipenv-install install: Collecting simple-websocket==1.1.0 (from -r
#13 45.37 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.37 pipenv-install install: 8-hashed-reqs.txt (line 53))
#13 45.37 pipenv-install install:   Downloading simple_websocket-1.1.0-py3-none-any.whl (13 kB)
#13 45.37 pipenv-install install: Collecting six==1.17.0 (from -r
#13 45.37 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.37 pipenv-install install: 8-hashed-reqs.txt (line 54))
#13 45.37 pipenv-install install:   Downloading six-1.17.0-py2.py3-none-any.whl (11 kB)
#13 45.38 pipenv-install install: Collecting sniffio==1.3.1 (from -r
#13 45.38 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.38 pipenv-install install: 8-hashed-reqs.txt (line 55))
#13 45.38 pipenv-install install:   Downloading sniffio-1.3.1-py3-none-any.whl (10 kB)
#13 45.38 pipenv-install install: Collecting starlette==0.36.3 (from -r
#13 45.38 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.38 pipenv-install install: 8-hashed-reqs.txt (line 56))
#13 45.38 pipenv-install install:   Downloading starlette-0.36.3-py3-none-any.whl (71 kB)
#13 45.38 pipenv-install install: Collecting termcolor==2.4.0 (from -r
#13 45.38 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.38 pipenv-install install: 8-hashed-reqs.txt (line 57))
#13 45.38 pipenv-install install:   Downloading termcolor-2.4.0-py3-none-any.whl (7.7 kB)
#13 45.38 pipenv-install install: Collecting tomlkit==0.15.0 (from -r
#13 45.38 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.38 pipenv-install install: 8-hashed-reqs.txt (line 58))
#13 45.38 pipenv-install install:   Downloading tomlkit-0.15.0-py3-none-any.whl (41 kB)
#13 45.38 pipenv-install install: Collecting tortoise-orm==0.21.7 (from -r
#13 45.38 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.38 pipenv-install install: 8-hashed-reqs.txt (line 59))
#13 45.38 pipenv-install install:   Downloading tortoise_orm-0.21.7-py3-none-any.whl (175 kB)
#13 45.38 pipenv-install install: Requirement already satisfied: typing-extensions==4.15.0 in
#13 45.38 pipenv-install install: /usr/lib/python3/dist-packages (from -r
#13 45.38 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.38 pipenv-install install: 8-hashed-reqs.txt (line 60)) (4.15.0)
#13 45.38 pipenv-install install: Collecting urllib3==2.7.0 (from -r
#13 45.38 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.38 pipenv-install install: 8-hashed-reqs.txt (line 61))
#13 45.38 pipenv-install install:   Downloading urllib3-2.7.0-py3-none-any.whl (131 kB)
#13 45.38 pipenv-install install: Collecting uvicorn==0.28.1 (from uvicorn==0.28.1->-r
#13 45.38 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.38 pipenv-install install: 8-hashed-reqs.txt (line 62))
#13 45.38 pipenv-install install:   Downloading uvicorn-0.28.1-py3-none-any.whl (60 kB)
#13 45.38 pipenv-install install: Collecting uvloop==0.22.1 (from -r
#13 45.38 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.38 pipenv-install install: 8-hashed-reqs.txt (line 63))
#13 45.38 pipenv-install install:   Downloading
#13 45.38 pipenv-install install: uvloop-0.22.1-cp314-cp314-manylinux2014_x86_64.manylinux_2_17_x86_64.manylinux_2
#13 45.38 pipenv-install install: _28_x86_64.whl (4.3 MB)
#13 45.38 pipenv-install install:      ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 4.3/4.3 MB 144.9 MB/s  0:00:00
#13 45.38 pipenv-install install: Collecting watchfiles==1.1.1 (from -r
#13 45.38 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.38 pipenv-install install: 8-hashed-reqs.txt (line 64))
#13 45.38 pipenv-install install:   Downloading
#13 45.39 pipenv-install install: watchfiles-1.1.1-cp314-cp314-manylinux_2_17_x86_64.manylinux2014_x86_64.whl (455
#13 45.39 pipenv-install install: kB)
#13 45.39 pipenv-install install: Collecting websocket-client==1.7.0 (from -r
#13 45.39 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.39 pipenv-install install: 8-hashed-reqs.txt (line 65))
#13 45.39 pipenv-install install:   Downloading websocket_client-1.7.0-py3-none-any.whl (58 kB)
#13 45.39 pipenv-install install: Collecting websockets==16.0 (from -r
#13 45.39 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.39 pipenv-install install: 8-hashed-reqs.txt (line 66))
#13 45.39 pipenv-install install:   Downloading
#13 45.39 pipenv-install install: websockets-16.0-cp314-cp314-manylinux1_x86_64.manylinux_2_28_x86_64.manylinux_2_
#13 45.39 pipenv-install install: 5_x86_64.whl (185 kB)
#13 45.39 pipenv-install install: Collecting werkzeug==3.1.8 (from -r
#13 45.39 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.39 pipenv-install install: 8-hashed-reqs.txt (line 67))
#13 45.39 pipenv-install install:   Downloading werkzeug-3.1.8-py3-none-any.whl (226 kB)
#13 45.39 pipenv-install install: Collecting wsproto==1.3.2 (from -r
#13 45.39 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-wup6gfia-requirements/pipenv-a4vp1zx
#13 45.39 pipenv-install install: 8-hashed-reqs.txt (line 68))
#13 45.39 pipenv-install install:   Downloading wsproto-1.3.2-py3-none-any.whl (24 kB)
#13 45.39 pipenv-install install: Building wheels for collected packages: asyncpg, pydantic-core
#13 45.39 pipenv-install install:   Building wheel for asyncpg (pyproject.toml): started
#13 45.39 pipenv-install install:   Building wheel for asyncpg (pyproject.toml): finished with status 'error'
#13 45.39 pipenv-install install:   Building wheel for pydantic-core (pyproject.toml): started
#13 45.39 pipenv-install install:   Building wheel for pydantic-core (pyproject.toml): finished with status
#13 45.39 pipenv-install install: 'error'
#13 45.39 pipenv-install install: Failed to build asyncpg pydantic-core
#13 45.39 pipenv-install install: error: subprocess-exited-with-error
#13 45.39 pipenv-install install:
#13 45.39 pipenv-install install:   × Building wheel for asyncpg (pyproject.toml) did not run successfully.
#13 45.39 pipenv-install install:   │ exit code: 1
#13 45.39 pipenv-install install:   ╰─> [146 lines of output]
#13 45.40 pipenv-install install:       /ws/pipenv-install/node_modules/.tmp/pip-build-env-7v0x4dvw/overlay/lib/py
#13 45.40 pipenv-install install: thon3.14/site-packages/setuptools/config/_apply_pyprojecttoml.py:82:
#13 45.40 pipenv-install install: SetuptoolsDeprecationWarning: `project.license` as a TOML table is deprecated
#13 45.40 pipenv-install install:       !!
#13 45.40 pipenv-install install:
#13 45.40 pipenv-install install:               ******************************************************************
#13 45.40 pipenv-install install: **************
#13 45.40 pipenv-install install:               Please use a simple string containing a SPDX expression for
#13 45.40 pipenv-install install: `project.license`. You can also use `project.license-files`. (Both options
#13 45.40 pipenv-install install: available on setuptools>=77.0.0).
#13 45.40 pipenv-install install:
#13 45.40 pipenv-install install:               By 2027-Feb-18, you need to update your project and remove
#13 45.40 pipenv-install install: deprecated calls
#13 45.40 pipenv-install install:               or your builds will no longer be supported.
#13 45.40 pipenv-install install:
#13 45.40 pipenv-install install:               See
#13 45.40 pipenv-install install: https://packaging.python.org/en/latest/guides/writing-pyproject-toml/#license
#13 45.40 pipenv-install install: for details.
#13 45.40 pipenv-install install:               ******************************************************************
#13 45.40 pipenv-install install: **************
#13 45.40 pipenv-install install:
#13 45.40 pipenv-install install:       !!
#13 45.40 pipenv-install install:         corresp(dist, value, root_dir)
#13 45.40 pipenv-install install:       /ws/pipenv-install/node_modules/.tmp/pip-build-env-7v0x4dvw/overlay/lib/py
#13 45.40 pipenv-install install: thon3.14/site-packages/setuptools/config/_apply_pyprojecttoml.py:61:
#13 45.40 pipenv-install install: SetuptoolsDeprecationWarning: License classifiers are deprecated.
#13 45.40 pipenv-install install:       !!
#13 45.40 pipenv-install install:
#13 45.40 pipenv-install install:               ******************************************************************
#13 45.40 pipenv-install install: **************
#13 45.40 pipenv-install install:               Please consider removing the following classifiers in favor of a
#13 45.40 pipenv-install install: SPDX license expression:
#13 45.40 pipenv-install install:
#13 45.40 pipenv-install install:               License :: OSI Approved :: Apache Software License
#13 45.40 pipenv-install install:
#13 45.40 pipenv-install install:               See
#13 45.40 pipenv-install install: https://packaging.python.org/en/latest/guides/writing-pyproject-toml/#license
#13 45.40 pipenv-install install: for details.
#13 45.40 pipenv-install install:               ******************************************************************
#13 45.40 pipenv-install install: **************
#13 45.40 pipenv-install install:
#13 45.40 pipenv-install install:       !!
#13 45.40 pipenv-install install:         dist._finalize_license_expression()
#13 45.41 pipenv-install install:       /ws/pipenv-install/node_modules/.tmp/pip-build-env-7v0x4dvw/overlay/lib/py
#13 45.41 pipenv-install install: thon3.14/site-packages/setuptools/dist.py:765: SetuptoolsDeprecationWarning:
#13 45.41 pipenv-install install: License classifiers are deprecated.
#13 45.41 pipenv-install install:       !!
#13 45.41 pipenv-install install:
#13 45.41 pipenv-install install:               ******************************************************************
#13 45.41 pipenv-install install: **************
#13 45.41 pipenv-install install:               Please consider removing the following classifiers in favor of a
#13 45.41 pipenv-install install: SPDX license expression:
#13 45.41 pipenv-install install:
#13 45.41 pipenv-install install:               License :: OSI Approved :: Apache Software License
#13 45.41 pipenv-install install:
#13 45.41 pipenv-install install:               See
#13 45.41 pipenv-install install: https://packaging.python.org/en/latest/guides/writing-pyproject-toml/#license
#13 45.41 pipenv-install install: for details.
#13 45.41 pipenv-install install:               ******************************************************************
#13 45.41 pipenv-install install: **************
#13 45.41 pipenv-install install:
#13 45.41 pipenv-install install:       !!
#13 45.41 pipenv-install install:         self._finalize_license_expression()
#13 45.41 pipenv-install install:       running bdist_wheel
#13 45.41 pipenv-install install:       running build
#13 45.41 pipenv-install install:       running build_py
#13 45.41 pipenv-install install:       creating build/lib.linux-x86_64-cpython-314/asyncpg
#13 45.41 pipenv-install install:       copying asyncpg/connection.py ->
#13 45.41 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg
#13 45.41 pipenv-install install:       copying asyncpg/connect_utils.py ->
#13 45.41 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg
#13 45.41 pipenv-install install:       copying asyncpg/types.py -> build/lib.linux-x86_64-cpython-314/asyncpg
#13 45.41 pipenv-install install:       copying asyncpg/pool.py -> build/lib.linux-x86_64-cpython-314/asyncpg
#13 45.41 pipenv-install install:       copying asyncpg/cluster.py -> build/lib.linux-x86_64-cpython-314/asyncpg
#13 45.41 pipenv-install install:       copying asyncpg/_version.py -> build/lib.linux-x86_64-cpython-314/asyncpg
#13 45.41 pipenv-install install:       copying asyncpg/__init__.py -> build/lib.linux-x86_64-cpython-314/asyncpg
#13 45.41 pipenv-install install:       copying asyncpg/introspection.py ->
#13 45.41 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg
#13 45.41 pipenv-install install:       copying asyncpg/compat.py -> build/lib.linux-x86_64-cpython-314/asyncpg
#13 45.41 pipenv-install install:       copying asyncpg/transaction.py ->
#13 45.41 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg
#13 45.42 pipenv-install install:       copying asyncpg/serverversion.py ->
#13 45.42 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg
#13 45.42 pipenv-install install:       copying asyncpg/_asyncio_compat.py ->
#13 45.42 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg
#13 45.42 pipenv-install install:       copying asyncpg/cursor.py -> build/lib.linux-x86_64-cpython-314/asyncpg
#13 45.42 pipenv-install install:       copying asyncpg/connresource.py ->
#13 45.42 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg
#13 45.42 pipenv-install install:       copying asyncpg/utils.py -> build/lib.linux-x86_64-cpython-314/asyncpg
#13 45.42 pipenv-install install:       copying asyncpg/prepared_stmt.py ->
#13 45.42 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg
#13 45.42 pipenv-install install:       creating build/lib.linux-x86_64-cpython-314/asyncpg/exceptions
#13 45.42 pipenv-install install:       copying asyncpg/exceptions/_base.py ->
#13 45.42 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/exceptions
#13 45.42 pipenv-install install:       copying asyncpg/exceptions/__init__.py ->
#13 45.42 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/exceptions
#13 45.42 pipenv-install install:       creating build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 45.42 pipenv-install install:       copying asyncpg/pgproto/types.py ->
#13 45.42 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 45.42 pipenv-install install:       copying asyncpg/pgproto/__init__.py ->
#13 45.42 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 45.42 pipenv-install install:       creating build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 45.42 pipenv-install install:       copying asyncpg/protocol/__init__.py ->
#13 45.42 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 45.42 pipenv-install install:       creating build/lib.linux-x86_64-cpython-314/asyncpg/_testbase
#13 45.42 pipenv-install install:       copying asyncpg/_testbase/fuzzer.py ->
#13 45.42 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/_testbase
#13 45.42 pipenv-install install:       copying asyncpg/_testbase/__init__.py ->
#13 45.42 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/_testbase
#13 45.42 pipenv-install install:       creating build/lib.linux-x86_64-cpython-314/asyncpg/protocol/codecs
#13 45.42 pipenv-install install:       copying asyncpg/protocol/codecs/__init__.py ->
#13 45.42 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol/codecs
#13 45.42 pipenv-install install:       running egg_info
#13 45.42 pipenv-install install:       writing asyncpg.egg-info/PKG-INFO
#13 45.42 pipenv-install install:       writing dependency_links to asyncpg.egg-info/dependency_links.txt
#13 45.43 pipenv-install install:       writing requirements to asyncpg.egg-info/requires.txt
#13 45.43 pipenv-install install:       writing top-level names to asyncpg.egg-info/top_level.txt
#13 45.43 pipenv-install install:       reading manifest file 'asyncpg.egg-info/SOURCES.txt'
#13 45.43 pipenv-install install:       reading manifest template 'MANIFEST.in'
#13 45.43 pipenv-install install:       warning: no files found matching '*.py' under directory 'examples'
#13 45.43 pipenv-install install:       adding license file 'LICENSE'
#13 45.43 pipenv-install install:       adding license file 'AUTHORS'
#13 45.43 pipenv-install install:       writing manifest file 'asyncpg.egg-info/SOURCES.txt'
#13 45.43 pipenv-install install:       copying asyncpg/pgproto/__init__.pxd ->
#13 45.43 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 45.43 pipenv-install install:       copying asyncpg/pgproto/buffer.pxd ->
#13 45.43 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 45.43 pipenv-install install:       copying asyncpg/pgproto/buffer.pyx ->
#13 45.43 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 45.43 pipenv-install install:       copying asyncpg/pgproto/consts.pxi ->
#13 45.43 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 45.43 pipenv-install install:       copying asyncpg/pgproto/cpythonx.pxd ->
#13 45.43 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 45.43 pipenv-install install:       copying asyncpg/pgproto/debug.pxd ->
#13 45.43 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 45.43 pipenv-install install:       copying asyncpg/pgproto/frb.pxd ->
#13 45.43 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 45.43 pipenv-install install:       copying asyncpg/pgproto/frb.pyx ->
#13 45.43 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 45.43 pipenv-install install:       copying asyncpg/pgproto/hton.pxd ->
#13 45.43 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 45.43 pipenv-install install:       copying asyncpg/pgproto/pgproto.pxd ->
#13 45.43 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 45.44 pipenv-install install:       copying asyncpg/pgproto/pgproto.pyx ->
#13 45.44 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 45.44 pipenv-install install:       copying asyncpg/pgproto/tohex.pxd ->
#13 45.44 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 45.44 pipenv-install install:       copying asyncpg/pgproto/uuid.pyx ->
#13 45.44 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 45.44 pipenv-install install:       copying asyncpg/protocol/consts.pxi ->
#13 45.44 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 45.44 pipenv-install install:       copying asyncpg/protocol/coreproto.pxd ->
#13 45.44 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 45.44 pipenv-install install:       copying asyncpg/protocol/coreproto.pyx ->
#13 45.44 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 45.44 pipenv-install install:       copying asyncpg/protocol/cpythonx.pxd ->
#13 45.44 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 45.44 pipenv-install install:       copying asyncpg/protocol/encodings.pyx ->
#13 45.44 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 45.44 pipenv-install install:       copying asyncpg/protocol/pgtypes.pxi ->
#13 45.44 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 45.44 pipenv-install install:       copying asyncpg/protocol/prepared_stmt.pxd ->
#13 45.44 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 45.44 pipenv-install install:       copying asyncpg/protocol/prepared_stmt.pyx ->
#13 45.44 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 45.44 pipenv-install install:       copying asyncpg/protocol/protocol.pxd ->
#13 45.44 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 45.44 pipenv-install install:       copying asyncpg/protocol/protocol.pyx ->
#13 45.44 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 45.44 pipenv-install install:       copying asyncpg/protocol/scram.pxd ->
#13 45.44 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 45.44 pipenv-install install:       copying asyncpg/protocol/scram.pyx ->
#13 45.44 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 45.44 pipenv-install install:       copying asyncpg/protocol/settings.pxd ->
#13 45.44 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 45.45 pipenv-install install:       copying asyncpg/protocol/settings.pyx ->
#13 45.45 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 45.45 pipenv-install install:       creating build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 45.45 pipenv-install install:       copying asyncpg/pgproto/codecs/__init__.pxd ->
#13 45.45 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 45.45 pipenv-install install:       copying asyncpg/pgproto/codecs/bits.pyx ->
#13 45.45 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 45.45 pipenv-install install:       copying asyncpg/pgproto/codecs/bytea.pyx ->
#13 45.45 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 45.45 pipenv-install install:       copying asyncpg/pgproto/codecs/context.pyx ->
#13 45.45 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 45.45 pipenv-install install:       copying asyncpg/pgproto/codecs/datetime.pyx ->
#13 45.45 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 45.45 pipenv-install install:       copying asyncpg/pgproto/codecs/float.pyx ->
#13 45.45 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 45.45 pipenv-install install:       copying asyncpg/pgproto/codecs/geometry.pyx ->
#13 45.45 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 45.45 pipenv-install install:       copying asyncpg/pgproto/codecs/hstore.pyx ->
#13 45.45 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 45.45 pipenv-install install:       copying asyncpg/pgproto/codecs/int.pyx ->
#13 45.45 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 45.45 pipenv-install install:       copying asyncpg/pgproto/codecs/json.pyx ->
#13 45.45 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 45.45 pipenv-install install:       copying asyncpg/pgproto/codecs/jsonpath.pyx ->
#13 45.45 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 45.46 pipenv-install install:       copying asyncpg/pgproto/codecs/misc.pyx ->
#13 45.46 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 45.46 pipenv-install install:       copying asyncpg/pgproto/codecs/network.pyx ->
#13 45.46 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 45.46 pipenv-install install:       copying asyncpg/pgproto/codecs/numeric.pyx ->
#13 45.46 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 45.46 pipenv-install install:       copying asyncpg/pgproto/codecs/pg_snapshot.pyx ->
#13 45.46 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 45.46 pipenv-install install:       copying asyncpg/pgproto/codecs/text.pyx ->
#13 45.46 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 45.46 pipenv-install install:       copying asyncpg/pgproto/codecs/tid.pyx ->
#13 45.46 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 45.46 pipenv-install install:       copying asyncpg/pgproto/codecs/uuid.pyx ->
#13 45.46 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 45.46 pipenv-install install:       creating build/lib.linux-x86_64-cpython-314/asyncpg/protocol/record
#13 45.46 pipenv-install install:       copying asyncpg/protocol/record/__init__.pxd ->
#13 45.46 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol/record
#13 45.46 pipenv-install install:       copying asyncpg/protocol/codecs/array.pyx ->
#13 45.46 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol/codecs
#13 45.46 pipenv-install install:       copying asyncpg/protocol/codecs/base.pxd ->
#13 45.46 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol/codecs
#13 45.46 pipenv-install install:       copying asyncpg/protocol/codecs/base.pyx ->
#13 45.46 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol/codecs
#13 45.46 pipenv-install install:       copying asyncpg/protocol/codecs/pgproto.pyx ->
#13 45.46 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol/codecs
#13 45.46 pipenv-install install:       copying asyncpg/protocol/codecs/range.pyx ->
#13 45.46 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol/codecs
#13 45.46 pipenv-install install:       copying asyncpg/protocol/codecs/record.pyx ->
#13 45.46 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol/codecs
#13 45.46 pipenv-install install:       copying asyncpg/protocol/codecs/textutils.pyx ->
#13 45.46 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol/codecs
#13 45.46 pipenv-install install:       warning: build_py: byte-compiling is disabled, skipping.
#13 45.46 pipenv-install install:
#13 45.46 pipenv-install install:       running build_ext
#13 45.47 pipenv-install install:       building 'asyncpg.pgproto.pgproto' extension
#13 45.47 pipenv-install install:       creating build/temp.linux-x86_64-cpython-314/asyncpg/pgproto
#13 45.47 pipenv-install install:       x86_64-linux-gnu-gcc -fno-strict-overflow -Wsign-compare -DNDEBUG -g -O2
#13 45.47 pipenv-install install: -Wall -fPIC -I/ws/.venv/include -I/usr/include/python3.14 -c
#13 45.47 pipenv-install install: asyncpg/pgproto/pgproto.c -o
#13 45.47 pipenv-install install: build/temp.linux-x86_64-cpython-314/asyncpg/pgproto/pgproto.o -O2 -fsigned-char
#13 45.47 pipenv-install install: -Wall -Wsign-compare -Wconversion
#13 45.47 pipenv-install install:       error: command 'x86_64-linux-gnu-gcc' failed: No such file or directory
#13 45.47 pipenv-install install:
#13 45.47 pipenv-install install:
#13 45.47 pipenv-install install:   note: This error originates from a subprocess, and is likely not a problem
#13 45.47 pipenv-install install: with pip.
#13 45.47 pipenv-install install:   ERROR: Failed building wheel for asyncpg
#13 45.47 pipenv-install install:   error: subprocess-exited-with-error
#13 45.47 pipenv-install install:
#13 45.47 pipenv-install install:   × Building wheel for pydantic-core (pyproject.toml) did not run successfully.
#13 45.47 pipenv-install install:   │ exit code: 1
#13 45.47 pipenv-install install:   ╰─> [40 lines of output]
#13 45.47 pipenv-install install:       Python reports SOABI: cpython-314-x86_64-linux-gnu
#13 45.47 pipenv-install install:       Computed rustc target triple: x86_64-unknown-linux-gnu
#13 45.47 pipenv-install install:       Installation directory: /root/.cache/puccinialin
#13 45.47 pipenv-install install:       Rustup already downloaded
#13 45.47 pipenv-install install:       Installing rust to /root/.cache/puccinialin/rustup
#13 45.47 pipenv-install install:       warn: It looks like you have an existing rustup settings file at:
#13 45.47 pipenv-install install:       warn: /root/.cache/puccinialin/rustup/settings.toml
#13 45.47 pipenv-install install:       warn: Rustup will install the default toolchain as specified in the
#13 45.47 pipenv-install install: settings file,
#13 45.47 pipenv-install install:       warn: instead of the one inferred from the default host triple.
#13 45.47 pipenv-install install:       info: profile set to minimal
#13 45.47 pipenv-install install:       info: setting default host triple to x86_64-unknown-linux-gnu
#13 45.47 pipenv-install install:       warn: Updating existing toolchain, profile choice will be ignored
#13 45.48 pipenv-install install:       info: syncing channel updates for stable-x86_64-unknown-linux-gnu
#13 45.48 pipenv-install install:       info: default toolchain set to stable-x86_64-unknown-linux-gnu
#13 45.48 pipenv-install install:       warn: no default linker (`cc`) was found in your PATH
#13 45.48 pipenv-install install:       warn: many Rust crates require a system C toolchain to build
#13 45.48 pipenv-install install:       Checking if cargo is installed
#13 45.48 pipenv-install install:       cargo 1.97.1 (c980f4866 2026-06-30)
#13 45.48 pipenv-install install:       Rust not found, installing into a temporary directory
#13 45.48 pipenv-install install:       Running `maturin pep517 build-wheel -i /ws/.venv/bin/python
#13 45.48 pipenv-install install: --compatibility off`
#13 45.48 pipenv-install install:       📦 Including license file `LICENSE`
#13 45.48 pipenv-install install:       🍹 Building a mixed python/rust project
#13 45.48 pipenv-install install:       🐍 Found CPython 3.14 at /ws/.venv/bin/python
#13 45.48 pipenv-install install:       🔗 Found pyo3 bindings
#13 45.48 pipenv-install install:       📡 Using build options features, bindings from pyproject.toml
#13 45.48 pipenv-install install:          Compiling target-lexicon v0.12.14
#13 45.48 pipenv-install install:          Compiling python3-dll-a v0.2.10
#13 45.48 pipenv-install install:          Compiling once_cell v1.19.0
#13 45.48 pipenv-install install:          Compiling proc-macro2 v1.0.86
#13 45.48 pipenv-install install:       error: linker `cc` not found
#13 45.48 pipenv-install install:         |
#13 45.48 pipenv-install install:         = note: No such file or directory (os error 2)
#13 45.48 pipenv-install install:
#13 45.49 pipenv-install install:       error: could not compile `proc-macro2` (build script) due to 1 previous
#13 45.49 pipenv-install install: error
#13 45.49 pipenv-install install:       warning: build failed, waiting for other jobs to finish...
#13 45.49 pipenv-install install:       error: could not compile `target-lexicon` (build script) due to 1 previous
#13 45.49 pipenv-install install: error
#13 45.49 pipenv-install install:       💥 maturin failed
#13 45.49 pipenv-install install:         Caused by: Failed to build a native library through cargo
#13 45.49 pipenv-install install:         Caused by: Cargo build finished with "exit status: 101": `env -u CARGO
#13 45.49 pipenv-install install: PYO3_BUILD_EXTENSION_MODULE="1" PYO3_ENVIRONMENT_SIGNATURE="cpython-3.14-64bit"
#13 45.49 pipenv-install install: PYO3_PYTHON="/ws/.venv/bin/python" PYTHON_SYS_EXECUTABLE="/ws/.venv/bin/python"
#13 45.49 pipenv-install install: "cargo" "rustc" "--profile" "release" "--features" "pyo3/extension-module"
#13 45.49 pipenv-install install: "--message-format" "json-render-diagnostics" "--manifest-path"
#13 45.49 pipenv-install install: "/ws/pipenv-install/node_modules/.tmp/pip-install-knpt3pfc/pydantic-core_34a3c22
#13 45.49 pipenv-install install: a88a642438ef43f2bb8265cb1/Cargo.toml" "--lib" "--crate-type" "cdylib"`
#13 45.49 pipenv-install install:       Error: command ['maturin', 'pep517', 'build-wheel', '-i',
#13 45.49 pipenv-install install: '/ws/.venv/bin/python', '--compatibility', 'off'] returned non-zero exit status
#13 45.49 pipenv-install install: 1
#13 45.49 pipenv-install install:
#13 45.49 pipenv-install install:
#13 45.49 pipenv-install install:   note: This error originates from a subprocess, and is likely not a problem
#13 45.49 pipenv-install install: with pip.
#13 45.49 pipenv-install install:   ERROR: Failed building wheel for pydantic-core
#13 45.49 pipenv-install install: error: failed-wheel-build-for-install
#13 45.49 pipenv-install install: × Failed to build installable wheels for some pyproject.toml based projects
#13 45.49 pipenv-install install: ╰─> asyncpg, pydantic-core
#13 45.51 pipenv-install install: ERROR: Couldn't install package(s): aiofiles==23.2.1; python_version >= '3.7'
#13 45.51 pipenv-install install: --hash=sha256:19297512c647d4b27a2cf7c34caa7e405c0d60b5560618a29a9fe027b18b0107
#13 45.51 pipenv-install install: --hash=sha256:84ec2218d8419404abcb9f0c02df3f34c6e0a68ed41072acfb1cef5cbc29051a,
#13 45.51 pipenv-install install: aiosqlite==0.20.0; python_version >= '3.8'
#13 45.51 pipenv-install install: --hash=sha256:36a1deaca0cac40ebe32aac9977a6e2bbc7f5189f23f4a54d5908986729e5bd6
#13 45.51 pipenv-install install: --hash=sha256:6d35c8c256637f4672f843c31021464090805bf925385ac39473fb16eaaca3d7,
#13 45.51 pipenv-install install: annotated-types==0.7.0; python_version >= '3.8'
#13 45.51 pipenv-install install: --hash=sha256:1f02e8b43a8fbbc3f3e0d4f0f4bfc8131bcb4eebe8849b8e5c773f3a1c582a53
#13 45.51 pipenv-install install: --hash=sha256:aff07c09a53a08bc8cfccb9c85b05f1aa9a2a6f23728d790723543408344ce89,
#13 45.51 pipenv-install install: anyio==4.13.0; python_version >= '3.10'
#13 45.51 pipenv-install install: --hash=sha256:08b310f9e24a9594186fd75b4f73f4a4152069e3853f1ed8bfbf58369f4ad708
#13 45.51 pipenv-install install: --hash=sha256:334b70e641fd2221c1505b3890c69882fe4a2df910cba14d97019b90b24439dc,
#13 45.51 pipenv-install install: argcomplete==3.6.3; python_version >= '3.8'
#13 45.51 pipenv-install install: --hash=sha256:62e8ed4fd6a45864acc8235409461b72c9a28ee785a2011cc5eb78318786c89c
#13 45.51 pipenv-install install: --hash=sha256:f5007b3a600ccac5d25bbce33089211dfd49eab4a7718da3f10e3082525a92ce,
#13 45.51 pipenv-install install: astroid==3.1.0; python_full_version >= '3.8.0'
#13 45.51 pipenv-install install: --hash=sha256:951798f922990137ac090c53af473db7ab4e70c770e6d7fae0cec59f74411819
#13 45.51 pipenv-install install: --hash=sha256:ac248253bfa4bd924a0de213707e7ebeeb3138abeb48d798784ead1e56d419d4,
#13 45.51 pipenv-install install: asyncpg==0.29.0; python_full_version >= '3.8.0'
#13 45.51 pipenv-install install: --hash=sha256:0009a300cae37b8c525e5b449233d59cd9868fd35431abc470a3e364d2b85cb9
#13 45.51 pipenv-install install: --hash=sha256:000c996c53c04770798053e1730d34e30cb645ad95a63265aec82da9093d88e7
#13 45.51 pipenv-install install: --hash=sha256:012d01df61e009015944ac7543d6ee30c2dc1eb2f6b10b62a3f598beb6531548
#13 45.51 pipenv-install install: --hash=sha256:039a261af4f38f949095e1e780bae84a25ffe3e370175193174eb08d3cecab23
#13 45.51 pipenv-install install: --hash=sha256:103aad2b92d1506700cbf51cd8bb5441e7e72e87a7b3a2ca4e32c840f051a6a3
#13 45.51 pipenv-install install: --hash=sha256:1e186427c88225ef730555f5fdda6c1812daa884064bfe6bc462fd3a71c4b675
#13 45.51 pipenv-install install: --hash=sha256:2245be8ec5047a605e0b454c894e54bf2ec787ac04b1cb7e0d3c67aa1e32f0fe
#13 45.51 pipenv-install install: --hash=sha256:37a2ec1b9ff88d8773d3eb6d3784dc7e3fee7756a5317b67f923172a4748a175
#13 45.51 pipenv-install install: --hash=sha256:48e7c58b516057126b363cec8ca02b804644fd012ef8e6c7e23386b7d5e6ce83
#13 45.51 pipenv-install install: --hash=sha256:52e8f8f9ff6e21f9b39ca9f8e3e33a5fcdceaf5667a8c5c32bee158e313be385
#13 45.51 pipenv-install install: --hash=sha256:5340dd515d7e52f4c11ada32171d87c05570479dc01dc66d03ee3e150fb695da
#13 45.51 pipenv-install install: --hash=sha256:54858bc25b49d1114178d65a88e48ad50cb2b6f3e475caa0f0c092d5f527c106
#13 45.51 pipenv-install install: --hash=sha256:5b52e46f165585fd6af4863f268566668407c76b2c72d366bb8b522fa66f1870
#13 45.51 pipenv-install install: --hash=sha256:5bbb7f2cafd8d1fa3e65431833de2642f4b2124be61a449fa064e1a08d27e449
#13 45.52 pipenv-install install: --hash=sha256:5cad1324dbb33f3ca0cd2074d5114354ed3be2b94d48ddfd88af75ebda7c43cc
#13 45.52 pipenv-install install: --hash=sha256:6011b0dc29886ab424dc042bf9eeb507670a3b40aece3439944006aafe023178
#13 45.52 pipenv-install install: --hash=sha256:642a36eb41b6313ffa328e8a5c5c2b5bea6ee138546c9c3cf1bffaad8ee36dd9
#13 45.52 pipenv-install install: --hash=sha256:6feaf2d8f9138d190e5ec4390c1715c3e87b37715cd69b2c3dfca616134efd2b
#13 45.52 pipenv-install install: --hash=sha256:72fd0ef9f00aeed37179c62282a3d14262dbbafb74ec0ba16e1b1864d8a12169
#13 45.52 pipenv-install install: --hash=sha256:746e80d83ad5d5464cfbf94315eb6744222ab00aa4e522b704322fb182b83610
#13 45.52 pipenv-install install: --hash=sha256:76c3ac6530904838a4b650b2880f8e7af938ee049e769ec2fba7cd66469d7772
#13 45.52 pipenv-install install: --hash=sha256:797ab8123ebaed304a1fad4d7576d5376c3a006a4100380fb9d517f0b59c1ab2
#13 45.52 pipenv-install install: --hash=sha256:8d36c7f14a22ec9e928f15f92a48207546ffe68bc412f3be718eedccdf10dc5c
#13 45.52 pipenv-install install: --hash=sha256:97eb024685b1d7e72b1972863de527c11ff87960837919dac6e34754768098eb
#13 45.52 pipenv-install install: --hash=sha256:a65c1dcd820d5aea7c7d82a3fdcb70e096f8f70d1a8bf93eb458e49bfad036ac
#13 45.52 pipenv-install install: --hash=sha256:a921372bbd0aa3a5822dd0409da61b4cd50df89ae85150149f8c119f23e8c408
#13 45.52 pipenv-install install: --hash=sha256:a9e6823a7012be8b68301342ba33b4740e5a166f6bbda0aee32bc01638491a22
#13 45.52 pipenv-install install: --hash=sha256:b544ffc66b039d5ec5a7454667f855f7fec08e0dfaf5a5490dfafbb7abbd2cfb
#13 45.52 pipenv-install install: --hash=sha256:bb1292d9fad43112a85e98ecdc2e051602bce97c199920586be83254d9dafc02
#13 45.52 pipenv-install install: --hash=sha256:bde17a1861cf10d5afce80a36fca736a86769ab3579532c03e45f83ba8a09c59
#13 45.52 pipenv-install install: --hash=sha256:cce08a178858b426ae1aa8409b5cc171def45d4293626e7aa6510696d46decd8
#13 45.52 pipenv-install install: --hash=sha256:cfe73ffae35f518cfd6e4e5f5abb2618ceb5ef02a2365ce64f132601000587d3
#13 45.52 pipenv-install install: --hash=sha256:d1c49e1f44fffafd9a55e1a9b101590859d881d639ea2922516f5d9c512d354e
#13 45.52 pipenv-install install: --hash=sha256:d4900ee08e85af01adb207519bb4e14b1cae8fd21e0ccf80fac6aa60b6da37b4
#13 45.52 pipenv-install install: --hash=sha256:d84156d5fb530b06c493f9e7635aa18f518fa1d1395ef240d211cb563c4e2364
#13 45.52 pipenv-install install: --hash=sha256:dc600ee8ef3dd38b8d67421359779f8ccec30b463e7aec7ed481c8346decf99f
#13 45.52 pipenv-install install: --hash=sha256:e0bfe9c4d3429706cf70d3249089de14d6a01192d617e9093a8e941fea8ee775
#13 45.52 pipenv-install install: --hash=sha256:e17b52c6cf83e170d3d865571ba574577ab8e533e7361a2b8ce6157d02c665d3
#13 45.52 pipenv-install install: --hash=sha256:f100d23f273555f4b19b74a96840aa27b85e99ba4b1f18d4ebff0734e78dc090
#13 45.52 pipenv-install install: --hash=sha256:f9ea3f24eb4c49a615573724d88a48bd1b7821c890c2effe04f05382ed9e8810
#13 45.52 pipenv-install install: --hash=sha256:ff8e8109cd6a46ff852a5e6bab8b0a047d7ea42fcb7ca5ae6eaae97d8eacf397,
#13 45.52 pipenv-install install: bidict==0.23.1; python_version >= '3.8'
#13 45.52 pipenv-install install: --hash=sha256:03069d763bc387bbd20e7d49914e75fc4132a41937fa3405417e1a5a2d006d71
#13 45.52 pipenv-install install: --hash=sha256:5dae8d4d79b552a71cbabc7deb25dfe8ce710b17ff41711e13010ead2abfc3e5,
#13 45.52 pipenv-install install: black==26.3.1; python_version >= '3.10'
#13 45.52 pipenv-install install: --hash=sha256:0126ae5b7c09957da2bdbd91a9ba1207453feada9e9fe51992848658c6c8e01c
#13 45.52 pipenv-install install: --hash=sha256:0f76ff19ec5297dd8e66eb64deda23631e642c9393ab592826fd4bdc97a4bce7
#13 45.52 pipenv-install install: --hash=sha256:28ef38aee69e4b12fda8dba75e21f9b4f979b490c8ac0baa7cb505369ac9e1ff
#13 45.52 pipenv-install install: --hash=sha256:2bd5aa94fc267d38bb21a70d7410a89f1a1d318841855f698746f8e7f51acd1b
#13 45.52 pipenv-install install: --hash=sha256:2c50f5063a9641c7eed7795014ba37b0f5fa227f3d408b968936e24bc0566b07
#13 45.52 pipenv-install install: --hash=sha256:2d6bfaf7fd0993b420bed691f20f9492d53ce9a2bcccea4b797d34e947318a78
#13 45.52 pipenv-install install: --hash=sha256:41cd2012d35b47d589cb8a16faf8a32ef7a336f56356babd9fcf70939ad1897f
#13 45.52 pipenv-install install: --hash=sha256:474c27574d6d7037c1bc875a81d9be0a9a4f9ee95e62800dab3cfaadbf75acd5
#13 45.52 pipenv-install install: --hash=sha256:5602bdb96d52d2d0672f24f6ffe5218795736dd34807fd0fd55ccd6bf206168b
#13 45.52 pipenv-install install: --hash=sha256:5e9d0d86df21f2e1677cc4bd090cd0e446278bcbbe49bf3659c308c3e402843e
#13 45.52 pipenv-install install: --hash=sha256:5ed0ca58586c8d9a487352a96b15272b7fa55d139fc8496b519e78023a8dab0a
#13 45.52 pipenv-install install: --hash=sha256:6c54a4a82e291a1fee5137371ab488866b7c86a3305af4026bdd4dc78642e1ac
#13 45.52 pipenv-install install: --hash=sha256:6e131579c243c98f35bce64a7e08e87fb2d610544754675d4a0e73a070a5aa3a
#13 45.52 pipenv-install install: --hash=sha256:855822d90f884905362f602880ed8b5df1b7e3ee7d0db2502d4388a954cc8c54
#13 45.52 pipenv-install install: --hash=sha256:86a8b5035fce64f5dcd1b794cf8ec4d31fe458cf6ce3986a30deb434df82a1d2
#13 45.52 pipenv-install install: --hash=sha256:8a33d657f3276328ce00e4d37fe70361e1ec7614da5d7b6e78de5426cb56332f
#13 45.52 pipenv-install install: --hash=sha256:92c0ec1f2cc149551a2b7b47efc32c866406b6891b0ee4625e95967c8f4acfb1
#13 45.52 pipenv-install install: --hash=sha256:9a5e9f45e5d5e1c5b5c29b3bd4265dcc90e8b92cf4534520896ed77f791f4da5
#13 45.52 pipenv-install install: --hash=sha256:afc622538b430aa4c8c853f7f63bc582b3b8030fd8c80b70fb5fa5b834e575c2
#13 45.52 pipenv-install install: --hash=sha256:b07fc0dab849d24a80a29cfab8d8a19187d1c4685d8a5e6385a5ce323c1f015f
#13 45.52 pipenv-install install: --hash=sha256:b5e6f89631eb88a7302d416594a32faeee9fb8fb848290da9d0a5f2903519fc1
#13 45.52 pipenv-install install: --hash=sha256:bf9bf162ed91a26f1adba8efda0b573bc6924ec1408a52cc6f82cb73ec2b142c
#13 45.52 pipenv-install install: --hash=sha256:c7e72339f841b5a237ff14f7d3880ddd0fc7f98a1199e8c4327f9a4f478c1839
#13 45.52 pipenv-install install: --hash=sha256:ddb113db38838eb9f043623ba274cfaf7d51d5b0c22ecb30afe58b1bb8322983
#13 45.52 pipenv-install install: --hash=sha256:dfdd51fc3e64ea4f35873d1b3fb25326773d55d2329ff8449139ebaad7357efb
#13 45.52 pipenv-install install: --hash=sha256:f1cd08e99d2f9317292a311dfe578fd2a24b15dbce97792f9c4d752275c1fa56
#13 45.52 pipenv-install install: --hash=sha256:f89f2ab047c76a9c03f78d0d66ca519e389519902fa27e7a91117ef7611c0568,
#13 45.52 pipenv-install install: certifi==2026.4.22; python_version >= '3.7'
#13 45.52 pipenv-install install: --hash=sha256:3cb2210c8f88ba2318d29b0388d1023c8492ff72ecdde4ebdaddbb13a31b1c4a
#13 45.52 pipenv-install install: --hash=sha256:8d455352a37b71bf76a79caa83a3d6c25afee4a385d632127b6afb3963f1c580,
#13 45.52 pipenv-install install: ...
#13 45.52 pipenv-install install: Package installation failed...
#13 45.57 pipenv-install install: Failed
#13 45.57 [ELIFECYCLE] Command failed with exit code 1.
#13 46.31 There are no packages awaiting approval
#13 46.93 Scope: 5 of 8 workspace projects
#13 47.05 ✓ Lockfile passes supply-chain policies (verified 39s ago)
#13 47.06 Lockfile is up to date, resolution step is skipped
#13 48.20 pipenv-install install$ ./bootstrap-pipenv.sh && ../.venv/bin/pipenv install -d --site-packages
#13 48.23 . prepare$ husky
#13 48.28 . prepare: .git can't be found
#13 48.28 . prepare: Done
#13 48.52 pipenv-install install: To activate this project's virtualenv, run pipenv shell.
#13 48.52 pipenv-install install: Alternatively, run a command inside the virtualenv with pipenv run.
#13 48.52 pipenv-install install: Installing dependencies from Pipfile.lock (d78544)...
#13 48.55 pipenv-install install: Installing dependencies from Pipfile.lock (d78544)...
#13 58.90 pipenv-install install: Collecting aiofiles==23.2.1 (from -r
#13 58.90 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.90 pipenv-install install: q-hashed-reqs.txt (line 1))
#13 58.90 pipenv-install install:   Using cached aiofiles-23.2.1-py3-none-any.whl (15 kB)
#13 58.90 pipenv-install install: Collecting aiosqlite==0.20.0 (from -r
#13 58.90 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.90 pipenv-install install: q-hashed-reqs.txt (line 2))
#13 58.90 pipenv-install install:   Using cached aiosqlite-0.20.0-py3-none-any.whl (15 kB)
#13 58.90 pipenv-install install: Collecting annotated-types==0.7.0 (from -r
#13 58.90 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.90 pipenv-install install: q-hashed-reqs.txt (line 3))
#13 58.90 pipenv-install install:   Using cached annotated_types-0.7.0-py3-none-any.whl (13 kB)
#13 58.90 pipenv-install install: Collecting anyio==4.13.0 (from -r
#13 58.91 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.91 pipenv-install install: q-hashed-reqs.txt (line 4))
#13 58.91 pipenv-install install:   Using cached anyio-4.13.0-py3-none-any.whl (114 kB)
#13 58.91 pipenv-install install: Requirement already satisfied: argcomplete==3.6.3 in
#13 58.91 pipenv-install install: /usr/lib/python3/dist-packages (from -r
#13 58.91 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.91 pipenv-install install: q-hashed-reqs.txt (line 5)) (3.6.3)
#13 58.91 pipenv-install install: Collecting astroid==3.1.0 (from -r
#13 58.91 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.91 pipenv-install install: q-hashed-reqs.txt (line 6))
#13 58.91 pipenv-install install:   Using cached astroid-3.1.0-py3-none-any.whl (275 kB)
#13 58.91 pipenv-install install: Collecting asyncpg==0.29.0 (from -r
#13 58.91 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.91 pipenv-install install: q-hashed-reqs.txt (line 7))
#13 58.91 pipenv-install install:   Using cached asyncpg-0.29.0.tar.gz (820 kB)
#13 58.91 pipenv-install install:   Installing build dependencies: started
#13 58.91 pipenv-install install:   Installing build dependencies: finished with status 'done'
#13 58.91 pipenv-install install:   Getting requirements to build wheel: started
#13 58.91 pipenv-install install:   Getting requirements to build wheel: finished with status 'done'
#13 58.91 pipenv-install install:   Preparing metadata (pyproject.toml): started
#13 58.91 pipenv-install install:   Preparing metadata (pyproject.toml): finished with status 'done'
#13 58.91 pipenv-install install: Collecting bidict==0.23.1 (from -r
#13 58.91 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.91 pipenv-install install: q-hashed-reqs.txt (line 8))
#13 58.91 pipenv-install install:   Using cached bidict-0.23.1-py3-none-any.whl (32 kB)
#13 58.91 pipenv-install install: Collecting black==26.3.1 (from -r
#13 58.91 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.91 pipenv-install install: q-hashed-reqs.txt (line 9))
#13 58.91 pipenv-install install:   Using cached
#13 58.91 pipenv-install install: black-26.3.1-cp314-cp314-manylinux2014_x86_64.manylinux_2_17_x86_64.manylinux_2_
#13 58.91 pipenv-install install: 28_x86_64.whl (1.8 MB)
#13 58.91 pipenv-install install: Collecting certifi==2026.4.22 (from -r
#13 58.91 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.91 pipenv-install install: q-hashed-reqs.txt (line 10))
#13 58.91 pipenv-install install:   Using cached certifi-2026.4.22-py3-none-any.whl (135 kB)
#13 58.92 pipenv-install install: Collecting cffi==2.0.0 (from -r
#13 58.92 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.92 pipenv-install install: q-hashed-reqs.txt (line 11))
#13 58.92 pipenv-install install:   Using cached
#13 58.92 pipenv-install install: cffi-2.0.0-cp314-cp314-manylinux2014_x86_64.manylinux_2_17_x86_64.whl (219 kB)
#13 58.92 pipenv-install install: Collecting charset-normalizer==3.4.7 (from -r
#13 58.92 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.92 pipenv-install install: q-hashed-reqs.txt (line 12))
#13 58.92 pipenv-install install:   Using cached
#13 58.92 pipenv-install install: charset_normalizer-3.4.7-cp314-cp314-manylinux2014_x86_64.manylinux_2_17_x86_64.
#13 58.92 pipenv-install install: manylinux_2_28_x86_64.whl (215 kB)
#13 58.92 pipenv-install install: Collecting click==8.3.3 (from -r
#13 58.92 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.92 pipenv-install install: q-hashed-reqs.txt (line 13))
#13 58.92 pipenv-install install:   Using cached click-8.3.3-py3-none-any.whl (110 kB)
#13 58.92 pipenv-install install: Collecting coverage==7.14.0 (from -r
#13 58.92 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.92 pipenv-install install: q-hashed-reqs.txt (line 14))
#13 58.92 pipenv-install install:   Using cached
#13 58.92 pipenv-install install: coverage-7.14.0-cp314-cp314-manylinux1_x86_64.manylinux_2_28_x86_64.manylinux_2_
#13 58.92 pipenv-install install: 5_x86_64.whl (253 kB)
#13 58.92 pipenv-install install: Collecting cryptography==48.0.0 (from -r
#13 58.92 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.92 pipenv-install install: q-hashed-reqs.txt (line 15))
#13 58.92 pipenv-install install:   Using cached cryptography-48.0.0-cp311-abi3-manylinux_2_34_x86_64.whl (4.7 MB)
#13 58.92 pipenv-install install: Collecting datamodel-code-generator==0.25.7 (from -r
#13 58.92 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.92 pipenv-install install: q-hashed-reqs.txt (line 16))
#13 58.92 pipenv-install install:   Using cached datamodel_code_generator-0.25.7-py3-none-any.whl (108 kB)
#13 58.92 pipenv-install install: Collecting dill==0.4.1 (from -r
#13 58.92 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.92 pipenv-install install: q-hashed-reqs.txt (line 17))
#13 58.92 pipenv-install install:   Using cached dill-0.4.1-py3-none-any.whl (120 kB)
#13 58.93 pipenv-install install: Collecting dnspython==2.8.0 (from -r
#13 58.93 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.93 pipenv-install install: q-hashed-reqs.txt (line 18))
#13 58.93 pipenv-install install:   Using cached dnspython-2.8.0-py3-none-any.whl (331 kB)
#13 58.93 pipenv-install install: Collecting email-validator==2.3.0 (from -r
#13 58.93 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.93 pipenv-install install: q-hashed-reqs.txt (line 19))
#13 58.93 pipenv-install install:   Using cached email_validator-2.3.0-py3-none-any.whl (35 kB)
#13 58.93 pipenv-install install: Collecting fastapi==0.109.2 (from -r
#13 58.93 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.93 pipenv-install install: q-hashed-reqs.txt (line 20))
#13 58.93 pipenv-install install:   Using cached fastapi-0.109.2-py3-none-any.whl (92 kB)
#13 58.93 pipenv-install install: Collecting freezegun==1.5.5 (from -r
#13 58.93 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.93 pipenv-install install: q-hashed-reqs.txt (line 21))
#13 58.93 pipenv-install install:   Using cached freezegun-1.5.5-py3-none-any.whl (19 kB)
#13 58.93 pipenv-install install: Collecting genson==1.3.0 (from -r
#13 58.93 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.93 pipenv-install install: q-hashed-reqs.txt (line 22))
#13 58.93 pipenv-install install:   Using cached genson-1.3.0-py3-none-any.whl (21 kB)
#13 58.93 pipenv-install install: Collecting h11==0.16.0 (from -r
#13 58.93 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.93 pipenv-install install: q-hashed-reqs.txt (line 23))
#13 58.93 pipenv-install install:   Using cached h11-0.16.0-py3-none-any.whl (37 kB)
#13 58.93 pipenv-install install: Collecting httpcore==1.0.9 (from -r
#13 58.93 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.93 pipenv-install install: q-hashed-reqs.txt (line 24))
#13 58.93 pipenv-install install:   Using cached httpcore-1.0.9-py3-none-any.whl (78 kB)
#13 58.93 pipenv-install install: Collecting httptools==0.7.1 (from -r
#13 58.93 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.93 pipenv-install install: q-hashed-reqs.txt (line 25))
#13 58.94 pipenv-install install:   Using cached
#13 58.94 pipenv-install install: httptools-0.7.1-cp314-cp314-manylinux1_x86_64.manylinux_2_28_x86_64.manylinux_2_
#13 58.94 pipenv-install install: 5_x86_64.whl (472 kB)
#13 58.94 pipenv-install install: Collecting httpx==0.26.0 (from -r
#13 58.94 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.94 pipenv-install install: q-hashed-reqs.txt (line 26))
#13 58.94 pipenv-install install:   Using cached httpx-0.26.0-py3-none-any.whl (75 kB)
#13 58.94 pipenv-install install: Collecting idna==3.15 (from -r
#13 58.94 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.94 pipenv-install install: q-hashed-reqs.txt (line 27))
#13 58.94 pipenv-install install:   Using cached idna-3.15-py3-none-any.whl (72 kB)
#13 58.94 pipenv-install install: Collecting inflect==5.6.2 (from -r
#13 58.94 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.94 pipenv-install install: q-hashed-reqs.txt (line 28))
#13 58.94 pipenv-install install:   Using cached inflect-5.6.2-py3-none-any.whl (33 kB)
#13 58.94 pipenv-install install: Collecting iso8601==2.1.0 (from -r
#13 58.94 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.94 pipenv-install install: q-hashed-reqs.txt (line 29))
#13 58.94 pipenv-install install:   Using cached iso8601-2.1.0-py3-none-any.whl (7.5 kB)
#13 58.94 pipenv-install install: Collecting isort==5.13.2 (from -r
#13 58.94 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.94 pipenv-install install: q-hashed-reqs.txt (line 30))
#13 58.94 pipenv-install install:   Using cached isort-5.13.2-py3-none-any.whl (92 kB)
#13 58.94 pipenv-install install: Requirement already satisfied: jinja2==3.1.6 in /usr/lib/python3/dist-packages
#13 58.94 pipenv-install install: (from -r
#13 58.94 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.94 pipenv-install install: q-hashed-reqs.txt (line 31)) (3.1.6)
#13 58.94 pipenv-install install: Requirement already satisfied: markupsafe==3.0.3 in
#13 58.94 pipenv-install install: /usr/lib/python3/dist-packages (from -r
#13 58.94 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.94 pipenv-install install: q-hashed-reqs.txt (line 32)) (3.0.3)
#13 58.94 pipenv-install install: Requirement already satisfied: mccabe==0.7.0 in /usr/lib/python3/dist-packages
#13 58.94 pipenv-install install: (from -r
#13 58.95 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.95 pipenv-install install: q-hashed-reqs.txt (line 33)) (0.7.0)
#13 58.95 pipenv-install install: Requirement already satisfied: mypy-extensions==1.1.0 in
#13 58.95 pipenv-install install: /usr/lib/python3/dist-packages (from -r
#13 58.95 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.95 pipenv-install install: q-hashed-reqs.txt (line 34)) (1.1.0)
#13 58.95 pipenv-install install: Collecting pathspec==1.1.1 (from -r
#13 58.95 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.95 pipenv-install install: q-hashed-reqs.txt (line 35))
#13 58.95 pipenv-install install:   Using cached pathspec-1.1.1-py3-none-any.whl (57 kB)
#13 58.95 pipenv-install install: Collecting platformdirs==4.9.6 (from -r
#13 58.95 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.95 pipenv-install install: q-hashed-reqs.txt (line 36))
#13 58.95 pipenv-install install:   Using cached platformdirs-4.9.6-py3-none-any.whl (21 kB)
#13 58.95 pipenv-install install: Collecting pycparser==3.0 (from -r
#13 58.95 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.95 pipenv-install install: q-hashed-reqs.txt (line 37))
#13 58.95 pipenv-install install:   Using cached pycparser-3.0-py3-none-any.whl (48 kB)
#13 58.95 pipenv-install install: Collecting pydantic==2.8.2 (from pydantic==2.8.2->-r
#13 58.95 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.95 pipenv-install install: q-hashed-reqs.txt (line 38))
#13 58.95 pipenv-install install:   Using cached pydantic-2.8.2-py3-none-any.whl (423 kB)
#13 58.95 pipenv-install install: Collecting pydantic-core==2.20.1 (from -r
#13 58.95 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.95 pipenv-install install: q-hashed-reqs.txt (line 39))
#13 58.95 pipenv-install install:   Using cached pydantic_core-2.20.1.tar.gz (388 kB)
#13 58.95 pipenv-install install:   Installing build dependencies: started
#13 58.95 pipenv-install install:   Installing build dependencies: finished with status 'done'
#13 58.95 pipenv-install install:   Getting requirements to build wheel: started
#13 58.95 pipenv-install install:   Getting requirements to build wheel: finished with status 'done'
#13 58.95 pipenv-install install:   Installing backend dependencies: started
#13 58.96 pipenv-install install:   Installing backend dependencies: finished with status 'done'
#13 58.96 pipenv-install install:   Preparing metadata (pyproject.toml): started
#13 58.96 pipenv-install install:   Preparing metadata (pyproject.toml): finished with status 'done'
#13 58.96 pipenv-install install: Collecting pyjwt==2.8.0 (from pyjwt==2.8.0->-r
#13 58.96 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.96 pipenv-install install: q-hashed-reqs.txt (line 40))
#13 58.96 pipenv-install install:   Using cached PyJWT-2.8.0-py3-none-any.whl (22 kB)
#13 58.96 pipenv-install install: Collecting pylint==3.1.0 (from -r
#13 58.96 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.96 pipenv-install install: q-hashed-reqs.txt (line 41))
#13 58.96 pipenv-install install:   Using cached pylint-3.1.0-py3-none-any.whl (515 kB)
#13 58.96 pipenv-install install: Collecting pypika-tortoise==0.2.2 (from -r
#13 58.96 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.96 pipenv-install install: q-hashed-reqs.txt (line 42))
#13 58.96 pipenv-install install:   Using cached pypika_tortoise-0.2.2-py3-none-any.whl (50 kB)
#13 58.96 pipenv-install install: Collecting python-dateutil==2.9.0.post0 (from -r
#13 58.96 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.96 pipenv-install install: q-hashed-reqs.txt (line 43))
#13 58.96 pipenv-install install:   Using cached python_dateutil-2.9.0.post0-py2.py3-none-any.whl (229 kB)
#13 58.96 pipenv-install install: Collecting python-dotenv==1.2.2 (from -r
#13 58.96 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.96 pipenv-install install: q-hashed-reqs.txt (line 44))
#13 58.96 pipenv-install install:   Using cached python_dotenv-1.2.2-py3-none-any.whl (22 kB)
#13 58.96 pipenv-install install: Collecting python-engineio==4.13.1 (from -r
#13 58.96 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.96 pipenv-install install: q-hashed-reqs.txt (line 45))
#13 58.96 pipenv-install install:   Using cached python_engineio-4.13.1-py3-none-any.whl (59 kB)
#13 58.96 pipenv-install install: Collecting python-socketio==5.11.4 (from -r
#13 58.96 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.96 pipenv-install install: q-hashed-reqs.txt (line 46))
#13 58.97 pipenv-install install:   Using cached python_socketio-5.11.4-py3-none-any.whl (76 kB)
#13 58.97 pipenv-install install: Collecting pytokens==0.4.1 (from -r
#13 58.97 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.97 pipenv-install install: q-hashed-reqs.txt (line 47))
#13 58.97 pipenv-install install:   Using cached
#13 58.97 pipenv-install install: pytokens-0.4.1-cp314-cp314-manylinux2014_x86_64.manylinux_2_17_x86_64.manylinux_
#13 58.97 pipenv-install install: 2_28_x86_64.whl (268 kB)
#13 58.97 pipenv-install install: Collecting pytz==2026.2 (from -r
#13 58.97 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.97 pipenv-install install: q-hashed-reqs.txt (line 48))
#13 58.97 pipenv-install install:   Using cached pytz-2026.2-py2.py3-none-any.whl (510 kB)
#13 58.97 pipenv-install install: Requirement already satisfied: pyyaml==6.0.3 in /usr/lib/python3/dist-packages
#13 58.97 pipenv-install install: (from -r
#13 58.97 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.97 pipenv-install install: q-hashed-reqs.txt (line 49)) (6.0.3)
#13 58.97 pipenv-install install: Collecting reactivex==4.0.4 (from -r
#13 58.97 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.97 pipenv-install install: q-hashed-reqs.txt (line 50))
#13 58.97 pipenv-install install:   Using cached reactivex-4.0.4-py3-none-any.whl (217 kB)
#13 58.97 pipenv-install install: Collecting requests==2.34.2 (from -r
#13 58.97 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.97 pipenv-install install: q-hashed-reqs.txt (line 51))
#13 58.97 pipenv-install install:   Using cached requests-2.34.2-py3-none-any.whl (73 kB)
#13 58.97 pipenv-install install: Collecting schedule==1.2.2 (from -r
#13 58.97 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.97 pipenv-install install: q-hashed-reqs.txt (line 52))
#13 58.97 pipenv-install install:   Using cached schedule-1.2.2-py3-none-any.whl (12 kB)
#13 58.97 pipenv-install install: Collecting simple-websocket==1.1.0 (from -r
#13 58.97 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.97 pipenv-install install: q-hashed-reqs.txt (line 53))
#13 58.97 pipenv-install install:   Using cached simple_websocket-1.1.0-py3-none-any.whl (13 kB)
#13 58.97 pipenv-install install: Collecting six==1.17.0 (from -r
#13 58.98 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.98 pipenv-install install: q-hashed-reqs.txt (line 54))
#13 58.98 pipenv-install install:   Using cached six-1.17.0-py2.py3-none-any.whl (11 kB)
#13 58.98 pipenv-install install: Collecting sniffio==1.3.1 (from -r
#13 58.98 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.98 pipenv-install install: q-hashed-reqs.txt (line 55))
#13 58.98 pipenv-install install:   Using cached sniffio-1.3.1-py3-none-any.whl (10 kB)
#13 58.98 pipenv-install install: Collecting starlette==0.36.3 (from -r
#13 58.98 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.98 pipenv-install install: q-hashed-reqs.txt (line 56))
#13 58.98 pipenv-install install:   Using cached starlette-0.36.3-py3-none-any.whl (71 kB)
#13 58.98 pipenv-install install: Collecting termcolor==2.4.0 (from -r
#13 58.98 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.98 pipenv-install install: q-hashed-reqs.txt (line 57))
#13 58.98 pipenv-install install:   Using cached termcolor-2.4.0-py3-none-any.whl (7.7 kB)
#13 58.98 pipenv-install install: Collecting tomlkit==0.15.0 (from -r
#13 58.98 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.98 pipenv-install install: q-hashed-reqs.txt (line 58))
#13 58.98 pipenv-install install:   Using cached tomlkit-0.15.0-py3-none-any.whl (41 kB)
#13 58.98 pipenv-install install: Collecting tortoise-orm==0.21.7 (from -r
#13 58.98 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.98 pipenv-install install: q-hashed-reqs.txt (line 59))
#13 58.98 pipenv-install install:   Using cached tortoise_orm-0.21.7-py3-none-any.whl (175 kB)
#13 58.98 pipenv-install install: Requirement already satisfied: typing-extensions==4.15.0 in
#13 58.98 pipenv-install install: /usr/lib/python3/dist-packages (from -r
#13 58.98 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.98 pipenv-install install: q-hashed-reqs.txt (line 60)) (4.15.0)
#13 58.98 pipenv-install install: Collecting urllib3==2.7.0 (from -r
#13 58.98 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.98 pipenv-install install: q-hashed-reqs.txt (line 61))
#13 58.98 pipenv-install install:   Using cached urllib3-2.7.0-py3-none-any.whl (131 kB)
#13 58.98 pipenv-install install: Collecting uvicorn==0.28.1 (from uvicorn==0.28.1->-r
#13 58.98 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.98 pipenv-install install: q-hashed-reqs.txt (line 62))
#13 58.98 pipenv-install install:   Using cached uvicorn-0.28.1-py3-none-any.whl (60 kB)
#13 58.98 pipenv-install install: Collecting uvloop==0.22.1 (from -r
#13 58.98 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.98 pipenv-install install: q-hashed-reqs.txt (line 63))
#13 58.98 pipenv-install install:   Using cached
#13 58.98 pipenv-install install: uvloop-0.22.1-cp314-cp314-manylinux2014_x86_64.manylinux_2_17_x86_64.manylinux_2
#13 58.98 pipenv-install install: _28_x86_64.whl (4.3 MB)
#13 58.98 pipenv-install install: Collecting watchfiles==1.1.1 (from -r
#13 58.98 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.98 pipenv-install install: q-hashed-reqs.txt (line 64))
#13 58.99 pipenv-install install:   Using cached
#13 58.99 pipenv-install install: watchfiles-1.1.1-cp314-cp314-manylinux_2_17_x86_64.manylinux2014_x86_64.whl (455
#13 58.99 pipenv-install install: kB)
#13 58.99 pipenv-install install: Collecting websocket-client==1.7.0 (from -r
#13 58.99 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.99 pipenv-install install: q-hashed-reqs.txt (line 65))
#13 58.99 pipenv-install install:   Using cached websocket_client-1.7.0-py3-none-any.whl (58 kB)
#13 58.99 pipenv-install install: Collecting websockets==16.0 (from -r
#13 58.99 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.99 pipenv-install install: q-hashed-reqs.txt (line 66))
#13 58.99 pipenv-install install:   Using cached
#13 58.99 pipenv-install install: websockets-16.0-cp314-cp314-manylinux1_x86_64.manylinux_2_28_x86_64.manylinux_2_
#13 58.99 pipenv-install install: 5_x86_64.whl (185 kB)
#13 58.99 pipenv-install install: Collecting werkzeug==3.1.8 (from -r
#13 58.99 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.99 pipenv-install install: q-hashed-reqs.txt (line 67))
#13 58.99 pipenv-install install:   Using cached werkzeug-3.1.8-py3-none-any.whl (226 kB)
#13 58.99 pipenv-install install: Collecting wsproto==1.3.2 (from -r
#13 58.99 pipenv-install install: /ws/pipenv-install/node_modules/.tmp/pipenv-pr7llja7-requirements/pipenv-ejr0fkh
#13 58.99 pipenv-install install: q-hashed-reqs.txt (line 68))
#13 58.99 pipenv-install install:   Using cached wsproto-1.3.2-py3-none-any.whl (24 kB)
#13 58.99 pipenv-install install: Building wheels for collected packages: asyncpg, pydantic-core
#13 58.99 pipenv-install install:   Building wheel for asyncpg (pyproject.toml): started
#13 58.99 pipenv-install install:   Building wheel for asyncpg (pyproject.toml): finished with status 'error'
#13 58.99 pipenv-install install:   Building wheel for pydantic-core (pyproject.toml): started
#13 58.99 pipenv-install install:   Building wheel for pydantic-core (pyproject.toml): finished with status
#13 58.99 pipenv-install install: 'error'
#13 58.99 pipenv-install install: Failed to build asyncpg pydantic-core
#13 58.99 pipenv-install install: error: subprocess-exited-with-error
#13 58.99 pipenv-install install:
#13 58.99 pipenv-install install:   × Building wheel for asyncpg (pyproject.toml) did not run successfully.
#13 58.99 pipenv-install install:   │ exit code: 1
#13 58.99 pipenv-install install:   ╰─> [146 lines of output]
#13 58.99 pipenv-install install:       /ws/pipenv-install/node_modules/.tmp/pip-build-env-1isy3v1f/overlay/lib/py
#13 58.99 pipenv-install install: thon3.14/site-packages/setuptools/config/_apply_pyprojecttoml.py:82:
#13 58.99 pipenv-install install: SetuptoolsDeprecationWarning: `project.license` as a TOML table is deprecated
#13 58.99 pipenv-install install:       !!
#13 58.99 pipenv-install install:
#13 58.99 pipenv-install install:               ******************************************************************
#13 58.99 pipenv-install install: **************
#13 59.00 pipenv-install install:               Please use a simple string containing a SPDX expression for
#13 59.00 pipenv-install install: `project.license`. You can also use `project.license-files`. (Both options
#13 59.00 pipenv-install install: available on setuptools>=77.0.0).
#13 59.00 pipenv-install install:
#13 59.00 pipenv-install install:               By 2027-Feb-18, you need to update your project and remove
#13 59.00 pipenv-install install: deprecated calls
#13 59.00 pipenv-install install:               or your builds will no longer be supported.
#13 59.00 pipenv-install install:
#13 59.00 pipenv-install install:               See
#13 59.00 pipenv-install install: https://packaging.python.org/en/latest/guides/writing-pyproject-toml/#license
#13 59.00 pipenv-install install: for details.
#13 59.00 pipenv-install install:               ******************************************************************
#13 59.00 pipenv-install install: **************
#13 59.00 pipenv-install install:
#13 59.00 pipenv-install install:       !!
#13 59.00 pipenv-install install:         corresp(dist, value, root_dir)
#13 59.00 pipenv-install install:       /ws/pipenv-install/node_modules/.tmp/pip-build-env-1isy3v1f/overlay/lib/py
#13 59.00 pipenv-install install: thon3.14/site-packages/setuptools/config/_apply_pyprojecttoml.py:61:
#13 59.00 pipenv-install install: SetuptoolsDeprecationWarning: License classifiers are deprecated.
#13 59.00 pipenv-install install:       !!
#13 59.00 pipenv-install install:
#13 59.00 pipenv-install install:               ******************************************************************
#13 59.00 pipenv-install install: **************
#13 59.00 pipenv-install install:               Please consider removing the following classifiers in favor of a
#13 59.00 pipenv-install install: SPDX license expression:
#13 59.00 pipenv-install install:
#13 59.00 pipenv-install install:               License :: OSI Approved :: Apache Software License
#13 59.00 pipenv-install install:
#13 59.00 pipenv-install install:               See
#13 59.00 pipenv-install install: https://packaging.python.org/en/latest/guides/writing-pyproject-toml/#license
#13 59.00 pipenv-install install: for details.
#13 59.00 pipenv-install install:               ******************************************************************
#13 59.00 pipenv-install install: **************
#13 59.00 pipenv-install install:
#13 59.00 pipenv-install install:       !!
#13 59.00 pipenv-install install:         dist._finalize_license_expression()
#13 59.00 pipenv-install install:       /ws/pipenv-install/node_modules/.tmp/pip-build-env-1isy3v1f/overlay/lib/py
#13 59.00 pipenv-install install: thon3.14/site-packages/setuptools/dist.py:765: SetuptoolsDeprecationWarning:
#13 59.00 pipenv-install install: License classifiers are deprecated.
#13 59.00 pipenv-install install:       !!
#13 59.00 pipenv-install install:
#13 59.00 pipenv-install install:               ******************************************************************
#13 59.00 pipenv-install install: **************
#13 59.01 pipenv-install install:               Please consider removing the following classifiers in favor of a
#13 59.01 pipenv-install install: SPDX license expression:
#13 59.01 pipenv-install install:
#13 59.01 pipenv-install install:               License :: OSI Approved :: Apache Software License
#13 59.01 pipenv-install install:
#13 59.01 pipenv-install install:               See
#13 59.01 pipenv-install install: https://packaging.python.org/en/latest/guides/writing-pyproject-toml/#license
#13 59.01 pipenv-install install: for details.
#13 59.01 pipenv-install install:               ******************************************************************
#13 59.01 pipenv-install install: **************
#13 59.01 pipenv-install install:
#13 59.01 pipenv-install install:       !!
#13 59.01 pipenv-install install:         self._finalize_license_expression()
#13 59.01 pipenv-install install:       running bdist_wheel
#13 59.01 pipenv-install install:       running build
#13 59.01 pipenv-install install:       running build_py
#13 59.01 pipenv-install install:       creating build/lib.linux-x86_64-cpython-314/asyncpg
#13 59.01 pipenv-install install:       copying asyncpg/connection.py ->
#13 59.01 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg
#13 59.01 pipenv-install install:       copying asyncpg/connect_utils.py ->
#13 59.01 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg
#13 59.01 pipenv-install install:       copying asyncpg/types.py -> build/lib.linux-x86_64-cpython-314/asyncpg
#13 59.01 pipenv-install install:       copying asyncpg/pool.py -> build/lib.linux-x86_64-cpython-314/asyncpg
#13 59.01 pipenv-install install:       copying asyncpg/cluster.py -> build/lib.linux-x86_64-cpython-314/asyncpg
#13 59.01 pipenv-install install:       copying asyncpg/_version.py -> build/lib.linux-x86_64-cpython-314/asyncpg
#13 59.02 pipenv-install install:       copying asyncpg/__init__.py -> build/lib.linux-x86_64-cpython-314/asyncpg
#13 59.02 pipenv-install install:       copying asyncpg/introspection.py ->
#13 59.02 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg
#13 59.02 pipenv-install install:       copying asyncpg/compat.py -> build/lib.linux-x86_64-cpython-314/asyncpg
#13 59.02 pipenv-install install:       copying asyncpg/transaction.py ->
#13 59.02 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg
#13 59.02 pipenv-install install:       copying asyncpg/serverversion.py ->
#13 59.02 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg
#13 59.02 pipenv-install install:       copying asyncpg/_asyncio_compat.py ->
#13 59.02 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg
#13 59.02 pipenv-install install:       copying asyncpg/cursor.py -> build/lib.linux-x86_64-cpython-314/asyncpg
#13 59.02 pipenv-install install:       copying asyncpg/connresource.py ->
#13 59.02 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg
#13 59.02 pipenv-install install:       copying asyncpg/utils.py -> build/lib.linux-x86_64-cpython-314/asyncpg
#13 59.02 pipenv-install install:       copying asyncpg/prepared_stmt.py ->
#13 59.02 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg
#13 59.02 pipenv-install install:       creating build/lib.linux-x86_64-cpython-314/asyncpg/exceptions
#13 59.02 pipenv-install install:       copying asyncpg/exceptions/_base.py ->
#13 59.02 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/exceptions
#13 59.02 pipenv-install install:       copying asyncpg/exceptions/__init__.py ->
#13 59.02 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/exceptions
#13 59.02 pipenv-install install:       creating build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 59.02 pipenv-install install:       copying asyncpg/pgproto/types.py ->
#13 59.02 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 59.02 pipenv-install install:       copying asyncpg/pgproto/__init__.py ->
#13 59.02 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 59.02 pipenv-install install:       creating build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 59.02 pipenv-install install:       copying asyncpg/protocol/__init__.py ->
#13 59.02 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 59.03 pipenv-install install:       creating build/lib.linux-x86_64-cpython-314/asyncpg/_testbase
#13 59.03 pipenv-install install:       copying asyncpg/_testbase/fuzzer.py ->
#13 59.03 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/_testbase
#13 59.03 pipenv-install install:       copying asyncpg/_testbase/__init__.py ->
#13 59.03 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/_testbase
#13 59.03 pipenv-install install:       creating build/lib.linux-x86_64-cpython-314/asyncpg/protocol/codecs
#13 59.03 pipenv-install install:       copying asyncpg/protocol/codecs/__init__.py ->
#13 59.03 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol/codecs
#13 59.03 pipenv-install install:       running egg_info
#13 59.03 pipenv-install install:       writing asyncpg.egg-info/PKG-INFO
#13 59.03 pipenv-install install:       writing dependency_links to asyncpg.egg-info/dependency_links.txt
#13 59.03 pipenv-install install:       writing requirements to asyncpg.egg-info/requires.txt
#13 59.03 pipenv-install install:       writing top-level names to asyncpg.egg-info/top_level.txt
#13 59.03 pipenv-install install:       reading manifest file 'asyncpg.egg-info/SOURCES.txt'
#13 59.03 pipenv-install install:       reading manifest template 'MANIFEST.in'
#13 59.03 pipenv-install install:       warning: no files found matching '*.py' under directory 'examples'
#13 59.03 pipenv-install install:       adding license file 'LICENSE'
#13 59.03 pipenv-install install:       adding license file 'AUTHORS'
#13 59.03 pipenv-install install:       writing manifest file 'asyncpg.egg-info/SOURCES.txt'
#13 59.03 pipenv-install install:       copying asyncpg/pgproto/__init__.pxd ->
#13 59.03 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 59.03 pipenv-install install:       copying asyncpg/pgproto/buffer.pxd ->
#13 59.03 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 59.03 pipenv-install install:       copying asyncpg/pgproto/buffer.pyx ->
#13 59.03 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 59.03 pipenv-install install:       copying asyncpg/pgproto/consts.pxi ->
#13 59.03 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 59.03 pipenv-install install:       copying asyncpg/pgproto/cpythonx.pxd ->
#13 59.03 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 59.04 pipenv-install install:       copying asyncpg/pgproto/debug.pxd ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 59.04 pipenv-install install:       copying asyncpg/pgproto/frb.pxd ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 59.04 pipenv-install install:       copying asyncpg/pgproto/frb.pyx ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 59.04 pipenv-install install:       copying asyncpg/pgproto/hton.pxd ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 59.04 pipenv-install install:       copying asyncpg/pgproto/pgproto.pxd ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 59.04 pipenv-install install:       copying asyncpg/pgproto/pgproto.pyx ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 59.04 pipenv-install install:       copying asyncpg/pgproto/tohex.pxd ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 59.04 pipenv-install install:       copying asyncpg/pgproto/uuid.pyx ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto
#13 59.04 pipenv-install install:       copying asyncpg/protocol/consts.pxi ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 59.04 pipenv-install install:       copying asyncpg/protocol/coreproto.pxd ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 59.04 pipenv-install install:       copying asyncpg/protocol/coreproto.pyx ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 59.04 pipenv-install install:       copying asyncpg/protocol/cpythonx.pxd ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 59.04 pipenv-install install:       copying asyncpg/protocol/encodings.pyx ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 59.04 pipenv-install install:       copying asyncpg/protocol/pgtypes.pxi ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 59.04 pipenv-install install:       copying asyncpg/protocol/prepared_stmt.pxd ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 59.04 pipenv-install install:       copying asyncpg/protocol/prepared_stmt.pyx ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 59.04 pipenv-install install:       copying asyncpg/protocol/protocol.pxd ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 59.04 pipenv-install install:       copying asyncpg/protocol/protocol.pyx ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 59.04 pipenv-install install:       copying asyncpg/protocol/scram.pxd ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 59.04 pipenv-install install:       copying asyncpg/protocol/scram.pyx ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 59.04 pipenv-install install:       copying asyncpg/protocol/settings.pxd ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 59.04 pipenv-install install:       copying asyncpg/protocol/settings.pyx ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol
#13 59.04 pipenv-install install:       creating build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 59.04 pipenv-install install:       copying asyncpg/pgproto/codecs/__init__.pxd ->
#13 59.04 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 59.04 pipenv-install install:       copying asyncpg/pgproto/codecs/bits.pyx ->
#13 59.05 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 59.05 pipenv-install install:       copying asyncpg/pgproto/codecs/bytea.pyx ->
#13 59.05 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 59.05 pipenv-install install:       copying asyncpg/pgproto/codecs/context.pyx ->
#13 59.05 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 59.05 pipenv-install install:       copying asyncpg/pgproto/codecs/datetime.pyx ->
#13 59.05 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 59.05 pipenv-install install:       copying asyncpg/pgproto/codecs/float.pyx ->
#13 59.05 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 59.05 pipenv-install install:       copying asyncpg/pgproto/codecs/geometry.pyx ->
#13 59.05 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 59.05 pipenv-install install:       copying asyncpg/pgproto/codecs/hstore.pyx ->
#13 59.05 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 59.05 pipenv-install install:       copying asyncpg/pgproto/codecs/int.pyx ->
#13 59.05 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 59.05 pipenv-install install:       copying asyncpg/pgproto/codecs/json.pyx ->
#13 59.05 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 59.05 pipenv-install install:       copying asyncpg/pgproto/codecs/jsonpath.pyx ->
#13 59.05 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 59.05 pipenv-install install:       copying asyncpg/pgproto/codecs/misc.pyx ->
#13 59.05 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 59.05 pipenv-install install:       copying asyncpg/pgproto/codecs/network.pyx ->
#13 59.05 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 59.05 pipenv-install install:       copying asyncpg/pgproto/codecs/numeric.pyx ->
#13 59.05 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 59.05 pipenv-install install:       copying asyncpg/pgproto/codecs/pg_snapshot.pyx ->
#13 59.05 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 59.05 pipenv-install install:       copying asyncpg/pgproto/codecs/text.pyx ->
#13 59.05 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 59.05 pipenv-install install:       copying asyncpg/pgproto/codecs/tid.pyx ->
#13 59.05 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 59.05 pipenv-install install:       copying asyncpg/pgproto/codecs/uuid.pyx ->
#13 59.05 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/pgproto/codecs
#13 59.05 pipenv-install install:       creating build/lib.linux-x86_64-cpython-314/asyncpg/protocol/record
#13 59.05 pipenv-install install:       copying asyncpg/protocol/record/__init__.pxd ->
#13 59.05 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol/record
#13 59.05 pipenv-install install:       copying asyncpg/protocol/codecs/array.pyx ->
#13 59.05 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol/codecs
#13 59.06 pipenv-install install:       copying asyncpg/protocol/codecs/base.pxd ->
#13 59.06 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol/codecs
#13 59.06 pipenv-install install:       copying asyncpg/protocol/codecs/base.pyx ->
#13 59.06 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol/codecs
#13 59.06 pipenv-install install:       copying asyncpg/protocol/codecs/pgproto.pyx ->
#13 59.06 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol/codecs
#13 59.06 pipenv-install install:       copying asyncpg/protocol/codecs/range.pyx ->
#13 59.06 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol/codecs
#13 59.06 pipenv-install install:       copying asyncpg/protocol/codecs/record.pyx ->
#13 59.06 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol/codecs
#13 59.06 pipenv-install install:       copying asyncpg/protocol/codecs/textutils.pyx ->
#13 59.06 pipenv-install install: build/lib.linux-x86_64-cpython-314/asyncpg/protocol/codecs
#13 59.06 pipenv-install install:       warning: build_py: byte-compiling is disabled, skipping.
#13 59.06 pipenv-install install:
#13 59.06 pipenv-install install:       running build_ext
#13 59.06 pipenv-install install:       building 'asyncpg.pgproto.pgproto' extension
#13 59.06 pipenv-install install:       creating build/temp.linux-x86_64-cpython-314/asyncpg/pgproto
#13 59.06 pipenv-install install:       x86_64-linux-gnu-gcc -fno-strict-overflow -Wsign-compare -DNDEBUG -g -O2
#13 59.06 pipenv-install install: -Wall -fPIC -I/ws/.venv/include -I/usr/include/python3.14 -c
#13 59.06 pipenv-install install: asyncpg/pgproto/pgproto.c -o
#13 59.06 pipenv-install install: build/temp.linux-x86_64-cpython-314/asyncpg/pgproto/pgproto.o -O2 -fsigned-char
#13 59.06 pipenv-install install: -Wall -Wsign-compare -Wconversion
#13 59.06 pipenv-install install:       error: command 'x86_64-linux-gnu-gcc' failed: No such file or directory
#13 59.06 pipenv-install install:
#13 59.06 pipenv-install install:
#13 59.06 pipenv-install install:   note: This error originates from a subprocess, and is likely not a problem
#13 59.06 pipenv-install install: with pip.
#13 59.06 pipenv-install install:   ERROR: Failed building wheel for asyncpg
#13 59.06 pipenv-install install:   error: subprocess-exited-with-error
#13 59.06 pipenv-install install:
#13 59.06 pipenv-install install:   × Building wheel for pydantic-core (pyproject.toml) did not run successfully.
#13 59.06 pipenv-install install:   │ exit code: 1
#13 59.06 pipenv-install install:   ╰─> [40 lines of output]
#13 59.07 pipenv-install install:       Python reports SOABI: cpython-314-x86_64-linux-gnu
#13 59.07 pipenv-install install:       Computed rustc target triple: x86_64-unknown-linux-gnu
#13 59.07 pipenv-install install:       Installation directory: /root/.cache/puccinialin
#13 59.07 pipenv-install install:       Rustup already downloaded
#13 59.07 pipenv-install install:       Installing rust to /root/.cache/puccinialin/rustup
#13 59.07 pipenv-install install:       warn: It looks like you have an existing rustup settings file at:
#13 59.07 pipenv-install install:       warn: /root/.cache/puccinialin/rustup/settings.toml
#13 59.07 pipenv-install install:       warn: Rustup will install the default toolchain as specified in the
#13 59.07 pipenv-install install: settings file,
#13 59.07 pipenv-install install:       warn: instead of the one inferred from the default host triple.
#13 59.07 pipenv-install install:       info: profile set to minimal
#13 59.07 pipenv-install install:       info: setting default host triple to x86_64-unknown-linux-gnu
#13 59.07 pipenv-install install:       warn: Updating existing toolchain, profile choice will be ignored
#13 59.07 pipenv-install install:       info: syncing channel updates for stable-x86_64-unknown-linux-gnu
#13 59.07 pipenv-install install:       info: default toolchain set to stable-x86_64-unknown-linux-gnu
#13 59.07 pipenv-install install:       warn: no default linker (`cc`) was found in your PATH
#13 59.07 pipenv-install install:       warn: many Rust crates require a system C toolchain to build
#13 59.07 pipenv-install install:       Checking if cargo is installed
#13 59.07 pipenv-install install:       cargo 1.97.1 (c980f4866 2026-06-30)
#13 59.07 pipenv-install install:       Rust not found, installing into a temporary directory
#13 59.07 pipenv-install install:       Running `maturin pep517 build-wheel -i /ws/.venv/bin/python
#13 59.07 pipenv-install install: --compatibility off`
#13 59.07 pipenv-install install:       📦 Including license file `LICENSE`
#13 59.07 pipenv-install install:       🍹 Building a mixed python/rust project
#13 59.07 pipenv-install install:       🐍 Found CPython 3.14 at /ws/.venv/bin/python
#13 59.07 pipenv-install install:       🔗 Found pyo3 bindings
#13 59.08 pipenv-install install:       📡 Using build options features, bindings from pyproject.toml
#13 59.08 pipenv-install install:          Compiling target-lexicon v0.12.14
#13 59.08 pipenv-install install:          Compiling python3-dll-a v0.2.10
#13 59.08 pipenv-install install:          Compiling once_cell v1.19.0
#13 59.08 pipenv-install install:          Compiling proc-macro2 v1.0.86
#13 59.08 pipenv-install install:       error: linker `cc` not found
#13 59.08 pipenv-install install:         |
#13 59.08 pipenv-install install:         = note: No such file or directory (os error 2)
#13 59.08 pipenv-install install:
#13 59.08 pipenv-install install:       error: could not compile `proc-macro2` (build script) due to 1 previous
#13 59.08 pipenv-install install: error
#13 59.08 pipenv-install install:       warning: build failed, waiting for other jobs to finish...
#13 59.08 pipenv-install install:       error: could not compile `target-lexicon` (build script) due to 1 previous
#13 59.08 pipenv-install install: error
#13 59.08 pipenv-install install:       💥 maturin failed
#13 59.08 pipenv-install install:         Caused by: Failed to build a native library through cargo
#13 59.08 pipenv-install install:         Caused by: Cargo build finished with "exit status: 101": `env -u CARGO
#13 59.08 pipenv-install install: PYO3_BUILD_EXTENSION_MODULE="1" PYO3_ENVIRONMENT_SIGNATURE="cpython-3.14-64bit"
#13 59.08 pipenv-install install: PYO3_PYTHON="/ws/.venv/bin/python" PYTHON_SYS_EXECUTABLE="/ws/.venv/bin/python"
#13 59.08 pipenv-install install: "cargo" "rustc" "--profile" "release" "--features" "pyo3/extension-module"
#13 59.08 pipenv-install install: "--message-format" "json-render-diagnostics" "--manifest-path"
#13 59.08 pipenv-install install: "/ws/pipenv-install/node_modules/.tmp/pip-install-jwnaf3ww/pydantic-core_c6b9c5e
#13 59.08 pipenv-install install: 86d1d43f7945aff33a6662894/Cargo.toml" "--lib" "--crate-type" "cdylib"`
#13 59.08 pipenv-install install:       Error: command ['maturin', 'pep517', 'build-wheel', '-i',
#13 59.08 pipenv-install install: '/ws/.venv/bin/python', '--compatibility', 'off'] returned non-zero exit status
#13 59.08 pipenv-install install: 1
#13 59.08 pipenv-install install:
#13 59.08 pipenv-install install:
#13 59.09 pipenv-install install:   note: This error originates from a subprocess, and is likely not a problem
#13 59.09 pipenv-install install: with pip.
#13 59.09 pipenv-install install:   ERROR: Failed building wheel for pydantic-core
#13 59.09 pipenv-install install: error: failed-wheel-build-for-install
#13 59.09 pipenv-install install: × Failed to build installable wheels for some pyproject.toml based projects
#13 59.09 pipenv-install install: ╰─> asyncpg, pydantic-core
#13 59.10 pipenv-install install: ERROR: Couldn't install package(s): aiofiles==23.2.1; python_version >= '3.7'
#13 59.10 pipenv-install install: --hash=sha256:19297512c647d4b27a2cf7c34caa7e405c0d60b5560618a29a9fe027b18b0107
#13 59.10 pipenv-install install: --hash=sha256:84ec2218d8419404abcb9f0c02df3f34c6e0a68ed41072acfb1cef5cbc29051a,
#13 59.10 pipenv-install install: aiosqlite==0.20.0; python_version >= '3.8'
#13 59.10 pipenv-install install: --hash=sha256:36a1deaca0cac40ebe32aac9977a6e2bbc7f5189f23f4a54d5908986729e5bd6
#13 59.10 pipenv-install install: --hash=sha256:6d35c8c256637f4672f843c31021464090805bf925385ac39473fb16eaaca3d7,
#13 59.10 pipenv-install install: annotated-types==0.7.0; python_version >= '3.8'
#13 59.10 pipenv-install install: --hash=sha256:1f02e8b43a8fbbc3f3e0d4f0f4bfc8131bcb4eebe8849b8e5c773f3a1c582a53
#13 59.10 pipenv-install install: --hash=sha256:aff07c09a53a08bc8cfccb9c85b05f1aa9a2a6f23728d790723543408344ce89,
#13 59.10 pipenv-install install: anyio==4.13.0; python_version >= '3.10'
#13 59.10 pipenv-install install: --hash=sha256:08b310f9e24a9594186fd75b4f73f4a4152069e3853f1ed8bfbf58369f4ad708
#13 59.10 pipenv-install install: --hash=sha256:334b70e641fd2221c1505b3890c69882fe4a2df910cba14d97019b90b24439dc,
#13 59.10 pipenv-install install: argcomplete==3.6.3; python_version >= '3.8'
#13 59.10 pipenv-install install: --hash=sha256:62e8ed4fd6a45864acc8235409461b72c9a28ee785a2011cc5eb78318786c89c
#13 59.10 pipenv-install install: --hash=sha256:f5007b3a600ccac5d25bbce33089211dfd49eab4a7718da3f10e3082525a92ce,
#13 59.10 pipenv-install install: astroid==3.1.0; python_full_version >= '3.8.0'
#13 59.10 pipenv-install install: --hash=sha256:951798f922990137ac090c53af473db7ab4e70c770e6d7fae0cec59f74411819
#13 59.10 pipenv-install install: --hash=sha256:ac248253bfa4bd924a0de213707e7ebeeb3138abeb48d798784ead1e56d419d4,
#13 59.10 pipenv-install install: asyncpg==0.29.0; python_full_version >= '3.8.0'
#13 59.10 pipenv-install install: --hash=sha256:0009a300cae37b8c525e5b449233d59cd9868fd35431abc470a3e364d2b85cb9
#13 59.10 pipenv-install install: --hash=sha256:000c996c53c04770798053e1730d34e30cb645ad95a63265aec82da9093d88e7
#13 59.10 pipenv-install install: --hash=sha256:012d01df61e009015944ac7543d6ee30c2dc1eb2f6b10b62a3f598beb6531548
#13 59.10 pipenv-install install: --hash=sha256:039a261af4f38f949095e1e780bae84a25ffe3e370175193174eb08d3cecab23
#13 59.10 pipenv-install install: --hash=sha256:103aad2b92d1506700cbf51cd8bb5441e7e72e87a7b3a2ca4e32c840f051a6a3
#13 59.10 pipenv-install install: --hash=sha256:1e186427c88225ef730555f5fdda6c1812daa884064bfe6bc462fd3a71c4b675
#13 59.10 pipenv-install install: --hash=sha256:2245be8ec5047a605e0b454c894e54bf2ec787ac04b1cb7e0d3c67aa1e32f0fe
#13 59.10 pipenv-install install: --hash=sha256:37a2ec1b9ff88d8773d3eb6d3784dc7e3fee7756a5317b67f923172a4748a175
#13 59.10 pipenv-install install: --hash=sha256:48e7c58b516057126b363cec8ca02b804644fd012ef8e6c7e23386b7d5e6ce83
#13 59.10 pipenv-install install: --hash=sha256:52e8f8f9ff6e21f9b39ca9f8e3e33a5fcdceaf5667a8c5c32bee158e313be385
#13 59.10 pipenv-install install: --hash=sha256:5340dd515d7e52f4c11ada32171d87c05570479dc01dc66d03ee3e150fb695da
#13 59.10 pipenv-install install: --hash=sha256:54858bc25b49d1114178d65a88e48ad50cb2b6f3e475caa0f0c092d5f527c106
#13 59.10 pipenv-install install: --hash=sha256:5b52e46f165585fd6af4863f268566668407c76b2c72d366bb8b522fa66f1870
#13 59.10 pipenv-install install: --hash=sha256:5bbb7f2cafd8d1fa3e65431833de2642f4b2124be61a449fa064e1a08d27e449
#13 59.10 pipenv-install install: --hash=sha256:5cad1324dbb33f3ca0cd2074d5114354ed3be2b94d48ddfd88af75ebda7c43cc
#13 59.10 pipenv-install install: --hash=sha256:6011b0dc29886ab424dc042bf9eeb507670a3b40aece3439944006aafe023178
#13 59.10 pipenv-install install: --hash=sha256:642a36eb41b6313ffa328e8a5c5c2b5bea6ee138546c9c3cf1bffaad8ee36dd9
#13 59.10 pipenv-install install: --hash=sha256:6feaf2d8f9138d190e5ec4390c1715c3e87b37715cd69b2c3dfca616134efd2b
#13 59.10 pipenv-install install: --hash=sha256:72fd0ef9f00aeed37179c62282a3d14262dbbafb74ec0ba16e1b1864d8a12169
#13 59.10 pipenv-install install: --hash=sha256:746e80d83ad5d5464cfbf94315eb6744222ab00aa4e522b704322fb182b83610
#13 59.10 pipenv-install install: --hash=sha256:76c3ac6530904838a4b650b2880f8e7af938ee049e769ec2fba7cd66469d7772
#13 59.10 pipenv-install install: --hash=sha256:797ab8123ebaed304a1fad4d7576d5376c3a006a4100380fb9d517f0b59c1ab2
#13 59.10 pipenv-install install: --hash=sha256:8d36c7f14a22ec9e928f15f92a48207546ffe68bc412f3be718eedccdf10dc5c
#13 59.10 pipenv-install install: --hash=sha256:97eb024685b1d7e72b1972863de527c11ff87960837919dac6e34754768098eb
#13 59.10 pipenv-install install: --hash=sha256:a65c1dcd820d5aea7c7d82a3fdcb70e096f8f70d1a8bf93eb458e49bfad036ac
#13 59.10 pipenv-install install: --hash=sha256:a921372bbd0aa3a5822dd0409da61b4cd50df89ae85150149f8c119f23e8c408
#13 59.10 pipenv-install install: --hash=sha256:a9e6823a7012be8b68301342ba33b4740e5a166f6bbda0aee32bc01638491a22
#13 59.10 pipenv-install install: --hash=sha256:b544ffc66b039d5ec5a7454667f855f7fec08e0dfaf5a5490dfafbb7abbd2cfb
#13 59.10 pipenv-install install: --hash=sha256:bb1292d9fad43112a85e98ecdc2e051602bce97c199920586be83254d9dafc02
#13 59.10 pipenv-install install: --hash=sha256:bde17a1861cf10d5afce80a36fca736a86769ab3579532c03e45f83ba8a09c59
#13 59.10 pipenv-install install: --hash=sha256:cce08a178858b426ae1aa8409b5cc171def45d4293626e7aa6510696d46decd8
#13 59.10 pipenv-install install: --hash=sha256:cfe73ffae35f518cfd6e4e5f5abb2618ceb5ef02a2365ce64f132601000587d3
#13 59.10 pipenv-install install: --hash=sha256:d1c49e1f44fffafd9a55e1a9b101590859d881d639ea2922516f5d9c512d354e
#13 59.10 pipenv-install install: --hash=sha256:d4900ee08e85af01adb207519bb4e14b1cae8fd21e0ccf80fac6aa60b6da37b4
#13 59.10 pipenv-install install: --hash=sha256:d84156d5fb530b06c493f9e7635aa18f518fa1d1395ef240d211cb563c4e2364
#13 59.10 pipenv-install install: --hash=sha256:dc600ee8ef3dd38b8d67421359779f8ccec30b463e7aec7ed481c8346decf99f
#13 59.10 pipenv-install install: --hash=sha256:e0bfe9c4d3429706cf70d3249089de14d6a01192d617e9093a8e941fea8ee775
#13 59.10 pipenv-install install: --hash=sha256:e17b52c6cf83e170d3d865571ba574577ab8e533e7361a2b8ce6157d02c665d3
#13 59.10 pipenv-install install: --hash=sha256:f100d23f273555f4b19b74a96840aa27b85e99ba4b1f18d4ebff0734e78dc090
#13 59.10 pipenv-install install: --hash=sha256:f9ea3f24eb4c49a615573724d88a48bd1b7821c890c2effe04f05382ed9e8810
#13 59.10 pipenv-install install: --hash=sha256:ff8e8109cd6a46ff852a5e6bab8b0a047d7ea42fcb7ca5ae6eaae97d8eacf397,
#13 59.10 pipenv-install install: bidict==0.23.1; python_version >= '3.8'
#13 59.10 pipenv-install install: --hash=sha256:03069d763bc387bbd20e7d49914e75fc4132a41937fa3405417e1a5a2d006d71
#13 59.10 pipenv-install install: --hash=sha256:5dae8d4d79b552a71cbabc7deb25dfe8ce710b17ff41711e13010ead2abfc3e5,
#13 59.10 pipenv-install install: black==26.3.1; python_version >= '3.10'
#13 59.10 pipenv-install install: --hash=sha256:0126ae5b7c09957da2bdbd91a9ba1207453feada9e9fe51992848658c6c8e01c
#13 59.10 pipenv-install install: --hash=sha256:0f76ff19ec5297dd8e66eb64deda23631e642c9393ab592826fd4bdc97a4bce7
#13 59.10 pipenv-install install: --hash=sha256:28ef38aee69e4b12fda8dba75e21f9b4f979b490c8ac0baa7cb505369ac9e1ff
#13 59.10 pipenv-install install: --hash=sha256:2bd5aa94fc267d38bb21a70d7410a89f1a1d318841855f698746f8e7f51acd1b
#13 59.10 pipenv-install install: --hash=sha256:2c50f5063a9641c7eed7795014ba37b0f5fa227f3d408b968936e24bc0566b07
#13 59.10 pipenv-install install: --hash=sha256:2d6bfaf7fd0993b420bed691f20f9492d53ce9a2bcccea4b797d34e947318a78
#13 59.10 pipenv-install install: --hash=sha256:41cd2012d35b47d589cb8a16faf8a32ef7a336f56356babd9fcf70939ad1897f
#13 59.10 pipenv-install install: --hash=sha256:474c27574d6d7037c1bc875a81d9be0a9a4f9ee95e62800dab3cfaadbf75acd5
#13 59.10 pipenv-install install: --hash=sha256:5602bdb96d52d2d0672f24f6ffe5218795736dd34807fd0fd55ccd6bf206168b
#13 59.10 pipenv-install install: --hash=sha256:5e9d0d86df21f2e1677cc4bd090cd0e446278bcbbe49bf3659c308c3e402843e
#13 59.10 pipenv-install install: --hash=sha256:5ed0ca58586c8d9a487352a96b15272b7fa55d139fc8496b519e78023a8dab0a
#13 59.10 pipenv-install install: --hash=sha256:6c54a4a82e291a1fee5137371ab488866b7c86a3305af4026bdd4dc78642e1ac
#13 59.10 pipenv-install install: --hash=sha256:6e131579c243c98f35bce64a7e08e87fb2d610544754675d4a0e73a070a5aa3a
#13 59.10 pipenv-install install: --hash=sha256:855822d90f884905362f602880ed8b5df1b7e3ee7d0db2502d4388a954cc8c54
#13 59.11 pipenv-install install: --hash=sha256:86a8b5035fce64f5dcd1b794cf8ec4d31fe458cf6ce3986a30deb434df82a1d2
#13 59.11 pipenv-install install: --hash=sha256:8a33d657f3276328ce00e4d37fe70361e1ec7614da5d7b6e78de5426cb56332f
#13 59.11 pipenv-install install: --hash=sha256:92c0ec1f2cc149551a2b7b47efc32c866406b6891b0ee4625e95967c8f4acfb1
#13 59.11 pipenv-install install: --hash=sha256:9a5e9f45e5d5e1c5b5c29b3bd4265dcc90e8b92cf4534520896ed77f791f4da5
#13 59.11 pipenv-install install: --hash=sha256:afc622538b430aa4c8c853f7f63bc582b3b8030fd8c80b70fb5fa5b834e575c2
#13 59.11 pipenv-install install: --hash=sha256:b07fc0dab849d24a80a29cfab8d8a19187d1c4685d8a5e6385a5ce323c1f015f
#13 59.11 pipenv-install install: --hash=sha256:b5e6f89631eb88a7302d416594a32faeee9fb8fb848290da9d0a5f2903519fc1
#13 59.11 pipenv-install install: --hash=sha256:bf9bf162ed91a26f1adba8efda0b573bc6924ec1408a52cc6f82cb73ec2b142c
#13 59.11 pipenv-install install: --hash=sha256:c7e72339f841b5a237ff14f7d3880ddd0fc7f98a1199e8c4327f9a4f478c1839
#13 59.11 pipenv-install install: --hash=sha256:ddb113db38838eb9f043623ba274cfaf7d51d5b0c22ecb30afe58b1bb8322983
#13 59.11 pipenv-install install: --hash=sha256:dfdd51fc3e64ea4f35873d1b3fb25326773d55d2329ff8449139ebaad7357efb
#13 59.11 pipenv-install install: --hash=sha256:f1cd08e99d2f9317292a311dfe578fd2a24b15dbce97792f9c4d752275c1fa56
#13 59.11 pipenv-install install: --hash=sha256:f89f2ab047c76a9c03f78d0d66ca519e389519902fa27e7a91117ef7611c0568,
#13 59.11 pipenv-install install: certifi==2026.4.22; python_version >= '3.7'
#13 59.11 pipenv-install install: --hash=sha256:3cb2210c8f88ba2318d29b0388d1023c8492ff72ecdde4ebdaddbb13a31b1c4a
#13 59.11 pipenv-install install: --hash=sha256:8d455352a37b71bf76a79caa83a3d6c25afee4a385d632127b6afb3963f1c580,
#13 59.11 pipenv-install install: ...
#13 59.11 pipenv-install install: Package installation failed...
#13 59.17 pipenv-install install: Failed
#13 59.19 [ELIFECYCLE] Command failed with exit code 1.
#13 ERROR: process "/bin/sh -c cd /ws   && (pnpm install --filter rmf-dashboard-framework... || (pnpm approve-builds --all && pnpm install --filter rmf-dashboard-framework...))" did not complete successfully: exit code: 1
------
 > [stage-0 6/7] RUN cd /ws   && (pnpm install --filter rmf-dashboard-framework... || (pnpm approve-builds --all && pnpm install --filter rmf-dashboard-framework...)):
59.11 pipenv-install install: --hash=sha256:dfdd51fc3e64ea4f35873d1b3fb25326773d55d2329ff8449139ebaad7357efb
59.11 pipenv-install install: --hash=sha256:f1cd08e99d2f9317292a311dfe578fd2a24b15dbce97792f9c4d752275c1fa56
59.11 pipenv-install install: --hash=sha256:f89f2ab047c76a9c03f78d0d66ca519e389519902fa27e7a91117ef7611c0568,
59.11 pipenv-install install: certifi==2026.4.22; python_version >= '3.7'
59.11 pipenv-install install: --hash=sha256:3cb2210c8f88ba2318d29b0388d1023c8492ff72ecdde4ebdaddbb13a31b1c4a
59.11 pipenv-install install: --hash=sha256:8d455352a37b71bf76a79caa83a3d6c25afee4a385d632127b6afb3963f1c580,
59.11 pipenv-install install: ...
59.11 pipenv-install install: Package installation failed...
59.17 pipenv-install install: Failed
59.19 [ELIFECYCLE] Command failed with exit code 1.
------
WARNING: No output specified with docker-container driver. Build result will only remain in the build cache. To push result image into registry use --push or to load image into docker use --load

 1 warning found (use docker --debug to expand):
 - InvalidDefaultArgInFrom: Default value for ARG $BASE_IMAGE results in empty or invalid base image name (line 4)
Dockerfile:25
--------------------
  24 |     # fallback to approve builds if pnpm install fails due to ignored scripts in non-interactive environment
  25 | >>> RUN cd /ws \
  26 | >>>   && (pnpm install --filter rmf-dashboard-framework... || (pnpm approve-builds --all && pnpm install --filter rmf-dashboard-framework...))
  27 |
--------------------
ERROR: failed to build: failed to solve: process "/bin/sh -c cd /ws   && (pnpm install --filter rmf-dashboard-framework... || (pnpm approve-builds --all && pnpm install --filter rmf-dashboard-framework...))" did not complete successfully: exit code: 1
Error: buildx failed with: ERROR: failed to build: failed to solve: process "/bin/sh -c cd /ws   && (pnpm install --filter rmf-dashboard-framework... || (pnpm approve-builds --all && pnpm install --filter rmf-dashboard-framework...))" did not complete successfully: exit code: 1
```