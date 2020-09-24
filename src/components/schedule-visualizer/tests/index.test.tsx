import React from 'react';
import ScheduleVisualizer from '../index';
import toJson from 'enzyme-to-json';
import { shallow } from 'enzyme';

it('renders without crashing', async () => {
  URL.createObjectURL = jest.fn();

  const wrapper = await shallow(
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
