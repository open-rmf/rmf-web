import React from 'react';
import * as RmfModels from 'rmf-models';
import { Dispenser, Door, Fleet, Ingestor, Lift, LiftState, SioClient } from 'api-client';
import { RmfIngress } from '../components/rmf-app/rmf-ingress';

export const useFleets = (
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

export const useFleetStateRef = (sioClient: SioClient | undefined, fleets: Fleet[]) => {
  const fleetStatesRef = React.useRef<Record<string, RmfModels.FleetState>>({});
  React.useEffect(() => {
    if (!sioClient) return;
    const subs = fleets.map((f) =>
      sioClient.subscribeFleetState(f.name, (state) => (fleetStatesRef.current[f.name] = state)),
    );
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
      subs.forEach((s) => sioClient.unsubscribe(s));
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

export const useDoors = (
  rmfIngress: RmfIngress | undefined,
  setDoors: React.Dispatch<React.SetStateAction<Door[]>>,
) => {
  React.useEffect(() => {
    if (!rmfIngress) return;
    let cancel = false;
    (async () => {
      const result = await rmfIngress.doorsApi.getDoorsDoorsGet();
      if (cancel || result.status !== 200) return;
      setDoors(result.data);
    })();
    return () => {
      cancel = true;
    };
  }, [rmfIngress, setDoors]);
};

export const useDoorStatesRef = (sioClient: SioClient | undefined, doors: Door[]) => {
  const doorStatesRef = React.useRef<Record<string, RmfModels.DoorState>>({});

  React.useEffect(() => {
    if (!sioClient) return;
    const subs = doors.map((d) =>
      sioClient.subscribeDoorState(d.name, (state) => (doorStatesRef.current[d.name] = state)),
    );
    return () => {
      subs.forEach((s) => sioClient.unsubscribe(s));
    };
  }, [sioClient, doors]);

  return doorStatesRef;
};

export const useLifts = (
  rmfIngress: RmfIngress | undefined,
  setLifts: React.Dispatch<React.SetStateAction<Lift[]>>,
) => {
  React.useEffect(() => {
    if (!rmfIngress) return;
    let cancel = false;
    (async () => {
      const result = await rmfIngress.liftsApi.getLiftsLiftsGet();
      if (cancel || result.status !== 200) return;
      setLifts(result.data);
    })();
    return () => {
      cancel = true;
    };
  }, [rmfIngress, setLifts]);
};

export const useLiftStatesRef = (sioClient: SioClient | undefined, lifts: Lift[]) => {
  const liftStatesRef = React.useRef<Record<string, LiftState>>({});

  React.useEffect(() => {
    if (!sioClient) return;
    const subs = lifts.map((l) =>
      sioClient.subscribeLiftState(l.name, (state) => (liftStatesRef.current[l.name] = state)),
    );
    return () => {
      subs.forEach((s) => sioClient.unsubscribe(s));
    };
  }, [sioClient, lifts]);

  return liftStatesRef;
};
