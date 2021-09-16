import React from 'react';
import { AppConfigContext, DataConfigContext } from '../app-contexts';
import { Configuration, SioClient } from 'api-client';
import appConfig from '../../app-config';
import axios from 'axios';
import LoopTaskPage from '../single-responsibility/send-loop-task';
import SingleButtonPage from '../single-responsibility/single-button';
import * as RmfModels from 'rmf-models';
import { BuildingMapContext } from '../rmf-app';
import { Button, ButtonGroup } from '@material-ui/core';

export default function Dashboard(_props: {}): React.ReactElement {
  const { authenticator } = React.useContext(AppConfigContext);
  const [buildingMap, setBuildingMap] = React.useState<RmfModels.BuildingMap | null>(null);
  const { REACT_APP_USER } = process.env;
  const [role, setRole] = React.useState(REACT_APP_USER);
  console.log(REACT_APP_USER);
  const [displayType, setDisplayType] = React.useState(1);
  const data = React.useContext(DataConfigContext);

  const token = appConfig.authenticator.token;

  const sioClient = React.useMemo(() => {
    const url = new URL(appConfig.rmfServerUrl);
    const path = url.pathname === '/' ? '' : url.pathname;

    const options: ConstructorParameters<typeof SioClient>[1] = {
      path: `${path}/socket.io`,
    };
    if (token) {
      options.auth = { token };
    }
    return new SioClient(url.origin, options);
  }, [token]);

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

  React.useEffect(() => {
    if (!sioClient) {
      return;
    }
    const sub = sioClient.subscribeBuildingMap(setBuildingMap);

    return () => {
      sioClient.unsubscribe(sub);
    };
  }, [sioClient]);

  return (
    <BuildingMapContext.Provider value={buildingMap}>
      <ButtonGroup
        style={{ justifyContent: 'flex-end' }}
        color="primary"
        aria-label="outlined primary button group"
      >
        <Button onClick={() => setDisplayType(0)}>SINGLE BUTTON</Button>
        <Button onClick={() => setDisplayType(1)}>FORM</Button>
        <Button onClick={() => setDisplayType(2)}>MULTI-PANEL</Button>
      </ButtonGroup>
      {displayType === 0 && (
        <SingleButtonPage
          data={data}
          sioClient={sioClient}
          apiConfig={apiConfig}
          axiosInst={axiosInst}
        />
      )}
      {displayType === 1 && (
        <LoopTaskPage
          data={data}
          sioClient={sioClient}
          apiConfig={apiConfig}
          axiosInst={axiosInst}
        />
      )}
      {displayType === 2 && <div>Multipanel Page</div>}
    </BuildingMapContext.Provider>
  );
}
