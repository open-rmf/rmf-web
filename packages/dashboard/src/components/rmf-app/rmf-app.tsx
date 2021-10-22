import React from 'react';
import { UserProfileProvider } from 'rmf-auth';
import appConfig from '../../app-config';
import { AppConfigContext } from '../app-contexts';
import { RmfIngressContext, RxRmfContext } from './contexts';
import { RmfIngress } from './rmf-ingress';
import { RxRmf } from './rx-rmf';

function RmfIngressProvider(props: React.PropsWithChildren<{}>): JSX.Element {
  const { authenticator } = React.useContext(AppConfigContext);
  const [rmfIngress, setRmfIngress] = React.useState<RmfIngress | undefined>(undefined);
  const rxRmf = React.useMemo(() => (rmfIngress ? new RxRmf(rmfIngress) : null), [rmfIngress]);

  React.useEffect(() => {
    if (authenticator.user) {
      return setRmfIngress(new RmfIngress(authenticator));
    } else {
      authenticator.once('userChanged', () => setRmfIngress(new RmfIngress(authenticator)));
      return undefined;
    }
  }, [authenticator]);

  return (
    <RmfIngressContext.Provider value={rmfIngress}>
      <RxRmfContext.Provider value={rxRmf}>{props.children}</RxRmfContext.Provider>
    </RmfIngressContext.Provider>
  );
}

export interface RmfAppProps extends React.PropsWithChildren<{}> {}

/**
 * Provides the following contexts:
 *
 * - UserProfileContext
 * - RmfIngressContext
 * - RxRmfContext
 */
export function RmfApp(props: RmfAppProps): JSX.Element {
  return (
    <UserProfileProvider authenticator={appConfig.authenticator} basePath={appConfig.rmfServerUrl}>
      <RmfIngressProvider>{props.children}</RmfIngressProvider>
    </UserProfileProvider>
  );
}
