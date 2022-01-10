import { BuildingMap, FleetState, Dispenser, Ingestor } from 'api-client';
import React from 'react';
import { getPlaces } from 'react-components';
import { UserProfileProvider } from 'rmf-auth';
import appConfig from '../../app-config';
import { AppConfigContext } from '../app-contexts';
import {
  BuildingMapContext,
  DispensersContext,
  FleetsContext,
  IngestorsContext,
  PlacesContext,
  RmfIngressContext,
} from './contexts';
import { RmfIngress } from './rmf-ingress';

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
  const [buildingMap, setBuildingMap] = React.useState<BuildingMap | null>(null);

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

function DispensersProvider(props: React.PropsWithChildren<{}>): JSX.Element {
  const { sioClient, dispensersApi } = React.useContext(RmfIngressContext) || {};
  const [dispensers, setDispensers] = React.useState<Dispenser[]>([]);

  React.useEffect(() => {
    if (!sioClient || !dispensersApi) {
      return;
    }
    let cancel = false;
    (async () => {
      const results = await dispensersApi.getDispensersDispensersGet();
      if (cancel || results.status !== 200) return;
      setDispensers(results.data);
    })();

    return () => {
      cancel = true;
    };
  }, [sioClient, dispensersApi]);

  return (
    <DispensersContext.Provider value={dispensers}>{props.children}</DispensersContext.Provider>
  );
}

function IngestorsProvider(props: React.PropsWithChildren<{}>): JSX.Element {
  const { sioClient, ingestorsApi } = React.useContext(RmfIngressContext) || {};
  const [ingestors, setIngestors] = React.useState<Ingestor[]>([]);

  React.useEffect(() => {
    if (!sioClient || !ingestorsApi) {
      return;
    }
    let cancel = false;
    (async () => {
      const results = await ingestorsApi.getIngestorsIngestorsGet();
      if (cancel || results.status !== 200) return;
      setIngestors(results.data);
    })();

    return () => {
      cancel = true;
    };
  }, [sioClient, ingestorsApi]);

  return <IngestorsContext.Provider value={ingestors}>{props.children}</IngestorsContext.Provider>;
}

function FleetsProvider(props: React.PropsWithChildren<{}>): JSX.Element {
  const { sioClient, fleetsApi } = React.useContext(RmfIngressContext) || {};
  const [fleets, setFleets] = React.useState<FleetState[]>([]);

  React.useEffect(() => {
    if (!sioClient || !fleetsApi) {
      return;
    }
    let cancel = false;
    (async () => {
      const results = await fleetsApi.queryFleetsFleetsGet();
      if (cancel || results.status !== 200) return;
      setFleets(results.data);
    })();

    return () => {
      cancel = true;
    };
  }, [sioClient, fleetsApi]);

  return <FleetsContext.Provider value={fleets}>{props.children}</FleetsContext.Provider>;
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

export interface RmfAppProps extends React.PropsWithChildren<{}> {}

/**
 * Provides the following contexts:
 *
 * - UserProfileContext
 * - RmfIngressContext
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
    <UserProfileProvider authenticator={appConfig.authenticator} basePath={appConfig.rmfServerUrl}>
      <RmfIngressProvider>
        <BuildingMapProvider>
          <FleetsProvider>
            <DispensersProvider>
              <IngestorsProvider>{props.children}</IngestorsProvider>
            </DispensersProvider>
          </FleetsProvider>
        </BuildingMapProvider>
      </RmfIngressProvider>
    </UserProfileProvider>
  );
}
