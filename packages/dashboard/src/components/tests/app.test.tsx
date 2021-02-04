import React from 'react';
import { render, waitFor } from '@testing-library/react';
import App from '../app';

// react-leaflet doesn't work well in jsdom.
jest.mock('./../schedule-visualizer', () => () => null);

it('renders without crashing', async () => {
  let root;
  await waitFor(() => {
    root = render(<App />);
  });
  root.unmount();
});
