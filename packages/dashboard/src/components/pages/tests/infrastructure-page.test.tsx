import React from 'react';
import { render } from '../../tests/test-utils';
import { InfrastructurePage } from '../infrastructure-page';

// react-leaflet doesn't work well in jsdom.
jest.mock('./../../schedule-visualizer', () => () => null);

it('renders without crashing', async () => {
  URL.createObjectURL = jest.fn();

  const root = render(<InfrastructurePage />);
  root.unmount();
  (URL.createObjectURL as jest.Mock).mockReset();
});
