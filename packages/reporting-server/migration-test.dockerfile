FROM postgres:latest

ENV POSTGRES_PASSWORD postgres 
ENV POSTGRES_DB reporting
ENV TZ UTC
EXPOSE 5432:5432

RUN apt-get update && apt-get install -y python3-pip
# Postgres:latest is installing an older version of pip, which causes problems when installing rust (rust is required by a python library we are using). This is fixed by installing a new version of pip which installs precompiled packages
RUN pip3 install --upgrade pip

SHELL ["bash", "-c"]
COPY dist .
RUN pip3 install $(ls -1 | grep '.*.whl')[postgres]

COPY migrations /root/reporting-server/

RUN echo -e '#!/bin/bash\n\
  cd root/reporting-server &&  aerich upgrade && aerich downgrade --yes -v 0 \n\ ' > /migrate.sh && chmod +x /migrate.sh
