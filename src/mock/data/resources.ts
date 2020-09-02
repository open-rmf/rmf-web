import { ResourceConfigurationsType } from '../../resource-manager';

export default function fakeResources(): ResourceConfigurationsType {
  return {
    dispensers: {
      coke_ingestor: {
        icons: { coke_ingestor: '' },
        location: { x: 20.95, y: -6.95, yaw: 0, level_name: 'L1' },
      },
      coke_dispenser: {
        icons: { coke_dispenser: '' },
        location: { x: 16.85, y: -4.8, yaw: 0, level_name: 'L1' },
      },
    },
    robots: {
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
      mir100: { icons: { mir100: '/robots/mir100/mir100.png' }, places: {} },
    },
  };
}
