import { RenderResult } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import TrajectoryTimeControl from '../trajectory-time-control';
import { render } from './leaflet-fixture';

describe('TrajectoryTimeControl', () => {
  const getControl = (root: RenderResult) => root.getByLabelText('trajectory time control');
  const getSlider = (root: RenderResult) =>
    root.getByRole('slider', { name: 'Trajectory Time (min)' });

  test('shows slider only when mouse hover', () => {
    const root = render(<TrajectoryTimeControl value={60000} min={60000} max={600000} />);
    expect(() => getSlider(root)).toThrow();
    userEvent.hover(getControl(root));
    expect(() => getSlider(root)).not.toThrow();
    userEvent.unhover(getControl(root));
    expect(() => getSlider(root)).toThrow();
  });
});
