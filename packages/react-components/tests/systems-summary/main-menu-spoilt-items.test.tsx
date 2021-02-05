import React from 'react';
import { MainMenuSpoiltItems } from '../../lib';
import { render } from '@testing-library/react';

test('smoke test', () => {
  render(<MainMenuSpoiltItems spoiltItems={[{ summary: 'item - state' }]} />);
});
