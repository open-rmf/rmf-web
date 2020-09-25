import React from 'react';
import ScheduleVisualizer from '../index';
import toJson from 'enzyme-to-json';
import { shallow } from 'enzyme';

const mockBuildingMap = {
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
};

it('renders without crashing', () => {
  URL.createObjectURL = jest.fn();

  const wrapper = shallow(
    <ScheduleVisualizer
      negotiationTrajStore={{}}
      fleets={[]}
      mapFloorLayerSorted={['name']}
      buildingMap={mockBuildingMap}
    />,
  );
  expect(toJson(wrapper)).toMatchSnapshot();
});
