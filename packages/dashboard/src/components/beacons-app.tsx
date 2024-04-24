import { TortoiseContribPydanticCreatorApiServerModelsTortoiseModelsBeaconsBeaconStateLeaf as BeaconState } from 'api-client';
import React from 'react';
import { BeaconDataGridTable } from 'react-components';
import { createMicroApp } from './micro-app';
import { RmfAppContext } from './rmf-app';

export const BeaconsApp = createMicroApp('Beacons', () => {
  const rmf = React.useContext(RmfAppContext);
  const [beacons, setBeacons] = React.useState<Record<string, BeaconState>>({});

  React.useEffect(() => {
    if (!rmf) {
      return;
    }

    (async () => {
      const { data } = await rmf.beaconsApi.getBeaconsBeaconsGet();
      for (const state of data) {
        setBeacons((prev) => {
          return {
            ...prev,
            [state.id]: state,
          };
        });
      }
    })();

    const sub = rmf.beaconsObsStore.subscribe(async (beaconState) => {
      setBeacons((prev) => {
        return {
          ...prev,
          [beaconState.id]: beaconState,
        };
      });
    });
    return () => sub.unsubscribe();
  }, [rmf]);

  return <BeaconDataGridTable beacons={Object.values(beacons)} />;
});
