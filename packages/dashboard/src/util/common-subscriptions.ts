import React from 'react';
import * as RmfModels from 'rmf-models';
import { FleetState, SioClient, Ingestor, Dispenser, Subscription } from 'api-client';
import { RmfIngress } from '../components/rmf-app/rmf-ingress';

export const useFleets = (
  rmfIngress: RmfIngress | undefined,
  setFleets: React.Dispatch<React.SetStateAction<FleetState[]>>,
) => {
  React.useEffect(() => {
    if (!rmfIngress) return;
    let cancel = false;
    (async () => {
      const result = await rmfIngress.fleetsApi.getFleetsFleetsGet();
      if (cancel || result.status !== 200) return;
      setFleets(result.data);
    })();
    return () => {
      cancel = true;
    };
  }, [rmfIngress, setFleets]);
};

export const useFleetStateRef = (sioClient: SioClient | undefined, fleets: FleetState[]) => {
  const fleetStatesRef = React.useRef<Record<string, FleetState>>({});
  React.useEffect(() => {
    if (!sioClient) return;
    const subs = fleets.reduce((acc, f) => {
      if (!f.name) {
        return acc;
      }
      const fleetName = f.name;
      acc.push(
        sioClient.subscribeFleetState(fleetName, (state) => {
          fleetStatesRef.current[fleetName] = state;
        }),
      );
      return acc;
    }, [] as Subscription[]);
    return () => {
      subs.forEach((s) => sioClient.unsubscribe(s));
    };
  }, [sioClient, fleets]);
  return fleetStatesRef;
};

export const useIngestorStatesRef = (sioClient: SioClient | undefined, ingestors: Ingestor[]) => {
  const ingestorStatesRef = React.useRef<Record<string, RmfModels.IngestorState>>({});
  React.useEffect(() => {
    if (!sioClient) return;
    const subs = ingestors.map((d) =>
      sioClient.subscribeIngestorState(
        d.guid,
        (state) => (ingestorStatesRef.current[d.guid] = state),
      ),
    );
    return () => {
      subs.filter(() => null).forEach((s) => sioClient.unsubscribe(s));
    };
  }, [sioClient, ingestors, ingestorStatesRef]);
  return ingestorStatesRef;
};

export const useDispenserStatesRef = (
  sioClient: SioClient | undefined,
  dispensers: Dispenser[],
) => {
  const dispenserStatesRef = React.useRef<Record<string, RmfModels.DispenserState>>({});
  React.useEffect(() => {
    if (!sioClient) return;
    const subs = dispensers.map((d) =>
      sioClient.subscribeDispenserState(
        d.guid,
        (state) => (dispenserStatesRef.current[d.guid] = state),
      ),
    );
    return () => {
      subs.forEach((s) => sioClient.unsubscribe(s));
    };
  }, [sioClient, dispensers, dispenserStatesRef]);
  return dispenserStatesRef;
};
