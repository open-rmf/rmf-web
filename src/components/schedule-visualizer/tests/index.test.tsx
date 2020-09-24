import { createMount } from '@material-ui/core/test-utils';
import React from 'react';
import ScheduleVisualizer from '../index';
import toJson from 'enzyme-to-json';

const mount = createMount();

it('renders without crashing', async () => {
  URL.createObjectURL = jest.fn();

  const wrapper = await mount(
    <ScheduleVisualizer
      negotiationTrajStore={{}}
      fleets={[]}
      mapFloorLayerSorted={['name']}
      buildingMap={{
        name: 'name',
        levels: [
          {
            name: 'name',
            places: [],
            elevation: 1,
            images: [],
            doors: [],
            nav_graphs: [],
            wall_graph: { name: '', vertices: [], edges: [], params: [] },
          },
        ],
        lifts: [],
      }}
    />,
  );
  expect(toJson(wrapper)).toMatchSnapshot();
});
