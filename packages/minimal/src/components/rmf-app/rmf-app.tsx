import React from 'react';
import * as RmfModels from 'rmf-models';
import { AppConfigContext } from '../app-contexts';
import { User, UserContext } from '../auth/contexts';
import { BuildingMapContext, PlacesContext, RmfIngressContext } from './contexts';
import { RmfIngress } from './rmf-ingress';
import { getPlaces } from './utils';

function PlacesProvider({ children }: React.PropsWithChildren<unknown>): JSX.Element {
  const buildingMap = React.useContext(BuildingMapContext);
  const places = React.useMemo(() => {
    if (!buildingMap) {
      return [];
    }
    return getPlaces(buildingMap);
  }, [buildingMap]);

  return <PlacesContext.Provider value={places}>{children}</PlacesContext.Provider>;
}

function BuildingMapProvider(props: React.PropsWithChildren<{}>): JSX.Element {
  const { sioClient } = React.useContext(RmfIngressContext) || {};
  const [buildingMap, setBuildingMap] = React.useState<RmfModels.BuildingMap | null>(null);

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
      <PlacesProvider>{props.children}</PlacesProvider>
    </BuildingMapContext.Provider>
  );
}

function RmfIngressProvider(props: React.PropsWithChildren<{}>): JSX.Element {
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
    <RmfIngressContext.Provider value={rmfIngress}>{props.children}</RmfIngressContext.Provider>
  );
}

function UserProvider(props: React.PropsWithChildren<{}>) {
  const rmfIngress = React.useContext(RmfIngressContext);
  const [user, setUser] = React.useState<User | null>(null);

  React.useEffect(() => {
    if (!rmfIngress) {
      return;
    }
    let cancel = false;
    (async () => {
      const getUserResp = await rmfIngress.defaultApi.getUserUserGet();
      const getPermResp = await rmfIngress.defaultApi.getEffectivePermissionsPermissionsGet();
      if (cancel || getUserResp.status !== 200 || getPermResp.status !== 200) return;
      setUser({
        profile: getUserResp.data,
        permissions: getPermResp.data,
      });
    })();
    return () => {
      cancel = true;
    };
  }, [rmfIngress]);

  return <UserContext.Provider value={user}>{user && props.children}</UserContext.Provider>;
}

export interface RmfAppProps extends React.PropsWithChildren<{}> {}

/**
 * Provides the following contexts:
 *
 * - RmfIngressContext
 * - UserContext
 * - BuildingMapContext
 * - PlacesContext
 * - FleetsContext
 * - DispensersContext
 * - IngestorsContext
 * - RmfHealthContext
 *
 * TODO: Move non changing contexts like building map, places, fleets to `RmfIngress`.
 * Lazily fetch them and cache the results to improve initial load times.
 */
export function RmfApp(props: RmfAppProps): JSX.Element {
  return (
    <RmfIngressProvider>
      <UserProvider>
        <BuildingMapProvider>{props.children}</BuildingMapProvider>
      </UserProvider>
    </RmfIngressProvider>
  );
}
