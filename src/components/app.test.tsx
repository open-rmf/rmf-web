import { createMount } from '@material-ui/core/test-utils';
import { ReactWrapper } from 'enzyme';
import React from 'react';
import ReactTestUtils from 'react-dom/test-utils';
import appConfig from '../app-config';
import App from './app';

const mount = createMount();

// react-leaflet doesn't work well in jsdom.
jest.mock('./schedule-visualizer', () => () => null);

it('renders without crashing', async () => {
  URL.createObjectURL = jest.fn();

  let wrapper: ReactWrapper | undefined;
  await ReactTestUtils.act(async () => {
    wrapper = mount(<App appConfig={appConfig} />);
  });
  wrapper?.unmount();

  (URL.createObjectURL as jest.Mock).mockReset();
});
