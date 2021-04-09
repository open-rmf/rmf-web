import * as RmfModels from 'rmf-models';
import airportMap from './building-map-airport';
import officeMap from './building-map-office';

export default async function buildingMap(): Promise<RmfModels.BuildingMap> {
  return {
    name: 'test building',
    levels: [
      {
        name: 'L1',
        elevation: 0,
        images: [
          {
            data: '',
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
        nav_graphs: officeMap.nav_graphs,
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
            data: '',
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
    lifts: officeMap.lifts,
  };
}
