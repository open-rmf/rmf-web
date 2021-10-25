import React from 'react';
import * as RmfModels from 'rmf-models';
import { Fleet, SioClient, Ingestor, Dispenser } from 'api-client';
import { RmfIngress } from '../components/rmf-app/rmf-ingress';

export const GetFleets = (
  rmfIngress: RmfIngress | undefined,
  setFleets: React.Dispatch<React.SetStateAction<Fleet[]>>,
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

export const SubscribeFleet = (
  sioClient: SioClient | undefined,
  fleets: Fleet[],
  fleetStatesRef: React.MutableRefObject<Record<string, RmfModels.FleetState>>,
) => {
  React.useEffect(() => {
    if (!sioClient) return;
    const subs = fleets.map((f) =>
      sioClient.subscribeFleetState(f.name, (state) => (fleetStatesRef.current[f.name] = state)),
    );
    return () => {
      subs.forEach((s) => sioClient.unsubscribe(s));
    };
  }, [sioClient, fleets, fleetStatesRef]);
};

export const SubscribeIngestor = (
  sioClient: SioClient | undefined,
  ingestors: Ingestor[],
  ingestorStatesRef: React.MutableRefObject<Record<string, RmfModels.IngestorState>>,
) => {
  React.useEffect(() => {
    if (!sioClient) return;
    const subs = ingestors.map((d) =>
      sioClient.subscribeIngestorState(
        d.guid,
        (state) => (ingestorStatesRef.current[d.guid] = state),
      ),
    );
    return () => {
      subs.forEach((s) => sioClient.unsubscribe(s));
    };
  }, [sioClient, ingestors, ingestorStatesRef]);
};

export const SubscribeDispenser = (
  sioClient: SioClient | undefined,
  dispensers: Dispenser[],
  dispenserStatesRef: React.MutableRefObject<Record<string, RmfModels.DispenserState>>,
) => {
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
};
