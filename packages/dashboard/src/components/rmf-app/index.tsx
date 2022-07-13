import React from 'react';
import { UserProfileProvider } from 'rmf-auth';
import appConfig from '../../app-config';
import { AppConfigContext } from '../app-contexts';
import { RmfIngress } from './rmf-ingress';

export * from './rmf-ingress';

export const RmfAppContext = React.createContext<RmfIngress | undefined>(undefined);

export interface RmfAppProps extends React.PropsWithChildren<{}> {}

export function RmfApp(props: RmfAppProps): JSX.Element {
  const { authenticator } = React.useContext(AppConfigContext);
  const [rmfIngress, setRmfIngress] = React.useState<RmfIngress | undefined>(undefined);

  React.useEffect(() => {
    if (authenticator.user) {
      return setRmfIngress(new RmfIngress(authenticator));
    } else {
      authenticator.once('userChanged', () => setRmfIngress(new RmfIngress(authenticator)));
      return undefined;
    }
  }, [authenticator]);

  return (
    <UserProfileProvider authenticator={appConfig.authenticator} basePath={appConfig.rmfServerUrl}>
      <RmfAppContext.Provider value={rmfIngress}>{props.children}</RmfAppContext.Provider>
    </UserProfileProvider>
  );
}
