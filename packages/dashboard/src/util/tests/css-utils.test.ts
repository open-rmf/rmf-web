import { viewBoxFromLeafletBounds } from '../css-utils';
import L from 'leaflet';

test('Correct calculus of bounds', () => {
  const bounds1 = viewBoxFromLeafletBounds(new L.LatLngBounds([0, 25.7], [-14, 0]));
  expect(bounds1).toBe('0 0 25.7 14');
  const bounds2 = viewBoxFromLeafletBounds([
    [-58, -25.7],
    [0, 0],
  ]);
  expect(bounds2).toBe('-25.7 0 25.7 58');
});
