import { render } from '@testing-library/react';
import React from 'react';
import SimpleOmniPanel from '../../examples/simple-omni-panel';

test('renders correctly', () => {
  const root = render(<SimpleOmniPanel />);
  expect(root.container).toMatchSnapshot();
});
