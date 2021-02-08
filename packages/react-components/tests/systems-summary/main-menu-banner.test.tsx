import React from 'react';
import { SystemSummaryBanner } from '../../lib';
import { render } from '@testing-library/react';

test('smoke test', () => {
  render(<SystemSummaryBanner bannerUrl={''} isError={false} />);
});
