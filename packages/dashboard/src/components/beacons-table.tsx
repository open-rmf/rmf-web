import { BeaconState } from 'api-client';
import React from 'react';
import { BeaconDataGridTable } from 'react-components';

import { useRmfApi } from '../hooks/use-rmf-api';

export const BeaconsTable = () => {
  const rmfApi = useRmfApi();
  const [beacons, setBeacons] = React.useState<Record<string, BeaconState>>({});

  React.useEffect(() => {
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
};

export default BeaconsTable;
