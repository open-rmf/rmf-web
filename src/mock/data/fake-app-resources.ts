import { ResourceConfigurationsType } from '../../resource-manager';

export default async function fakeAppResources(): Promise<ResourceConfigurationsType> {
  return {
    robots: {
      FleetA: {
        icons: { FleetA: '/robots/magni/magni.png' },
        places: {
          hardware_2: ['coke_ingestor'],
          pantry: ['coke_dispenser'],
        },
      },
      magni: {
        icons: { magni: '/robots/magni/magni.png' },
        places: {
          supplies: [],
          magni2_charger: [],
          coe: [],
          magni1_charger: [],
          hardware_2: ['coke_ingestor'],
          cubicle_2: [],
          pantry: ['coke_dispenser'],
          station_1: [],
          lounge: [],
          cubicle_1: [],
          hardware_1: [],
          station_2: [],
        },
      },
    },
    dispenser: {
      dispenser1: '/img/',
    },
  };
}
