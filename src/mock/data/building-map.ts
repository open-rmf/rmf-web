import * as RomiCore from '@osrf/romi-js-core-interfaces';
import airportPngUrl from './airport_terminal.png';
import officePngUrl from './office.png';
import officeMap from './building-map-office';
import airportMap from './building-map-airport';

export default async function buildingMap(): Promise<RomiCore.BuildingMap> {
  let officePng: Uint8Array;
  let airportPng: Uint8Array;
  if (process.env.NODE_ENV === 'test') {
    const fs = await import('fs');
    officePng = fs.readFileSync(`${__dirname}/office.png`);
    airportPng = fs.readFileSync(`${__dirname}/airport_terminal.png`);
  } else {
    const officePngResp = await fetch(officePngUrl);
    officePng = new Uint8Array(await officePngResp.arrayBuffer());
    const airportPngResp = await fetch(airportPngUrl);
    airportPng = new Uint8Array(await airportPngResp.arrayBuffer());
  }

  return {
    name: 'test building',
    levels: [
      {
        name: 'L1',
        elevation: 0,
        images: [
          {
            data: officePng,
            encoding: 'png',
            name: 'office',
            scale: 0.008465494960546494,
            x_offset: 0,
            y_offset: 0,
            yaw: 0,
          },
        ],
        doors: officeMap.doors,
        places: officeMap.places,
        nav_graphs: [],
        wall_graph: {
          name: 'wallgraph',
          vertices: [],
          edges: [],
          params: [],
        },
      },
      {
        name: 'Airport Terminal',
        elevation: 10,
        images: [
          {
            data: airportPng,
            encoding: 'png',
            name: 'airport_terminal',
            scale: 0.08212187141180038,
            x_offset: 0,
            y_offset: 0,
            yaw: 0,
          },
        ],
        doors: airportMap.doors,
        places: airportMap.places,
        nav_graphs: [],
        wall_graph: {
          name: 'wallgraph',
          vertices: [],
          edges: [],
          params: [],
        },
      },
    ],
    lifts: officeMap.lifts
  };
}
