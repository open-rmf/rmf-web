import React from 'react';

import { AuthenticatorContext } from '../../app-config';
import { UserProfileProvider } from '../../auth';
import { RmfIngress } from './rmf-ingress';

export * from './rmf-ingress';

export const RmfAppContext = React.createContext<RmfIngress | undefined>(undefined);

export interface RmfAppProps extends React.PropsWithChildren<{}> {}

export function RmfApp(props: RmfAppProps): JSX.Element {
  const authenticator = React.useContext(AuthenticatorContext);
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
    <UserProfileProvider authenticator={authenticator} basePath={RMF_SERVER_URL}>
      <RmfAppContext.Provider value={rmfIngress}>{props.children}</RmfAppContext.Provider>
    </UserProfileProvider>
  );
}
