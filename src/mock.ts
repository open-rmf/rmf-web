import axios from 'axios';

import { IFloor } from './models/Floor';

// Data dump by running rmf cgh dp1++ 
export async function getFloors() {
  const floors: IFloor[] = [{
    name: 'B1',
    elevation: 0,
    image: {
      data: new Uint8Array(),
      encoding: 'svgz',
      name: '',
      pose: {
        x: 0, y: 0, theta: 0,
      },
      scale: 0.01323084719479084,
    },
  }, {
    name: 'L1',
    elevation: 10,
    image: {
      data: new Uint8Array(),
      encoding: 'svgz',
      name: '',
      pose: {
        x: 8.70765726416178, y: -6.18003974078817, theta: 2.501489805517566e-8,
      },
      scale: 0.013230848126113415,
    },
  }, {
    name: 'L2',
    elevation: 20,
    image: {
      data: new Uint8Array(),
      encoding: 'svgz',
      name: '',
      pose: {
        x: -0.000004227897746558317, y: 0.0000019762215498307504, theta: 2.547079860507988e-8,
      },
      scale: 0.013230848126113415,
    },
  }, {
    name: 'L3',
    elevation: 30,
    image: {
      data: new Uint8Array(),
      encoding: 'svgz',
      name: '',
      pose: {
        x: -0.000004227897746558317, y: 0.0000019762215498307504, theta: 2.547079860507988e-8,
      },
      scale: 0.013230848126113415,
    },
  }, {
    name: 'L4',
    elevation: 40,
    image: {
      data: new Uint8Array(),
      encoding: 'svgz',
      name: '',
      pose: {
        x: 0.04656812660730849, y: -0.15806161149718637, theta: 2.501489902329013e-8,
      },
      scale: 0.013230848126113415,
    },
  }];

  const floorSvgUrls = [
    '/mock/B1_lift_26.svgz',
    '/mock/L1_lift_26.svgz',
    '/mock/L2_lift_26.svgz',
    '/mock/L3_lift_26.svgz',
    '/mock/L4_lift_26.svgz',
  ]

  for (let i = 0; i < floors.length; ++i) {
    const floor = floors[i];
    const floorSvgUrl = floorSvgUrls[i];
    floor.image.data = new Uint8Array(
      (await axios.get(floorSvgUrl, { responseType: 'arraybuffer'})).data
    );
  }

  return floors;
}