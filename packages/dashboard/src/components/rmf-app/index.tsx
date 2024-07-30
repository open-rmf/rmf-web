import React from 'react';
import { UserProfileProvider } from 'rmf-auth';
import { AppConfigContext, AuthenticatorContext } from '../../app-config';
import { RmfIngress } from './rmf-ingress';

export * from './rmf-ingress';

export const RmfAppContext = React.createContext<RmfIngress | undefined>(undefined);

export interface RmfAppProps extends React.PropsWithChildren<{}> {}

export function RmfApp(props: RmfAppProps): JSX.Element {
  const appConfig = React.useContext(AppConfigContext);
  const authenticator = React.useContext(AuthenticatorContext);
  const [rmfIngress, setRmfIngress] = React.useState<RmfIngress | undefined>(undefined);

  React.useEffect(() => {
    if (authenticator.user) {
      return setRmfIngress(new RmfIngress(appConfig, authenticator));
    } else {
      authenticator.once('userChanged', () =>
        setRmfIngress(new RmfIngress(appConfig, authenticator)),
      );
      return undefined;
    }
  }, [authenticator]);

  return (
    <UserProfileProvider authenticator={authenticator} basePath={appConfig.rmfServerUrl}>
      <RmfAppContext.Provider value={rmfIngress}>{props.children}</RmfAppContext.Provider>
    </UserProfileProvider>
  );
}
