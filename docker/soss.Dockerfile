FROM rmf/rmf

ADD soss.yaml.in /root/soss/
ADD certs /root/soss/certs
RUN sed 's/{{pwd}}/\/root\/soss/' /root/soss/soss.yaml.in > /root/soss/soss.yaml
