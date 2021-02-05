import React from 'react';
import { MainMenuBanner } from '../../lib';
import { render } from '@testing-library/react';

test('smoke test', () => {
  render(<MainMenuBanner bannerUrl={''} isError={false} />);
});
