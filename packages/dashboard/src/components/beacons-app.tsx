import { BeaconState } from 'api-client';
import React from 'react';
import { BeaconDataGridTable } from 'react-components';

import { createMicroApp } from './micro-app';
import { RmfApiContext } from './rmf-dashboard';

export const BeaconsApp = createMicroApp('Beacons', () => {
  const rmfApi = React.useContext(RmfApiContext);
  const [beacons, setBeacons] = React.useState<Record<string, BeaconState>>({});

  React.useEffect(() => {
    if (!rmfApi) {
      return;
    }

    (async () => {
      const { data } = await rmfApi.beaconsApi.getBeaconsBeaconsGet();
      for (const state of data) {
        setBeacons((prev) => {
          return {
            ...prev,
            [state.id]: state,
          };
        });
      }
    })();

    const sub = rmfApi.beaconsObsStore.subscribe(async (beaconState) => {
      setBeacons((prev) => {
        return {
          ...prev,
          [beaconState.id]: beaconState,
        };
      });
    });
    return () => sub.unsubscribe();
  }, [rmfApi]);

  return <BeaconDataGridTable beacons={Object.values(beacons)} />;
});
