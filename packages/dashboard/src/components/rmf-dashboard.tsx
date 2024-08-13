import React from 'react';

import { Authenticator } from '../services/authenticator';
import { RmfApi } from '../services/rmf-api';

export * from '../services/rmf-api';

export const RmfApiContext = React.createContext<RmfApi | null>(null);

export interface RmfDashboardProps extends React.PropsWithChildren<{}> {
  apiServerUrl: string;
  trajectoryServerUrl: string;
  authenticator: Authenticator;
}

export function RmfDashboard({
  apiServerUrl,
  trajectoryServerUrl,
  authenticator,
  children,
}: RmfDashboardProps) {
  const rmfApi = React.useMemo(
    () => new RmfApi(apiServerUrl, trajectoryServerUrl, authenticator),
    [],
  );

  React.useEffect(() => {
    authenticator.init();
  }, []);

  return <RmfApiContext.Provider value={rmfApi}>{children}</RmfApiContext.Provider>;
}
