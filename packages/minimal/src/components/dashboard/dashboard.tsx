import React from 'react';
import { AppConfigContext, DataConfigContext } from '../app-contexts';
import { Configuration, SioClient } from 'api-client';
import appConfig from '../../app-config';
import axios from 'axios';
import LoopTaskPage from '../single-responsibility/send-loop-task';

export default function Dashboard(_props: {}): React.ReactElement {
  const { authenticator } = React.useContext(AppConfigContext);
  const data = React.useContext(DataConfigContext);

  const token = appConfig.authenticator.token;
  const url = new URL(appConfig.rmfServerUrl);
  const path = url.pathname === '/' ? '' : url.pathname;

  const options: ConstructorParameters<typeof SioClient>[1] = {
    path: `${path}/socket.io`,
  };
  if (token) {
    options.auth = { token };
  }
  const sioClient = new SioClient(url.origin, options);
  sioClient.sio.on('error', console.error);

  const axiosInst = axios.create();
  axiosInst.interceptors.request.use(async (req) => {
    await authenticator.refreshToken();
    const token = authenticator.token;
    if (!token) {
      return req;
    }
    req.headers['Authorization'] = `Bearer ${token}`;
    return req;
  });

  const apiConfig: Configuration = {
    accessToken: authenticator.token,
    basePath: appConfig.rmfServerUrl,
  };

  return (
    <div>
      <LoopTaskPage data={data} sioClient={sioClient} apiConfig={apiConfig} axiosInst={axiosInst} />
    </div>
  );
}
