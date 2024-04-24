import type { TortoiseContribPydanticCreatorApiServerModelsTortoiseModelsBeaconsBeaconStateLeaf as BeaconState } from 'api-client';

export function makeBeaconState(state?: Partial<BeaconState>): BeaconState {
  return {
    id: 'test_beacon',
    online: true,
    level: 'L3',
    category: 'Audio, Visual',
    activated: false,
    ...state,
  };
}
