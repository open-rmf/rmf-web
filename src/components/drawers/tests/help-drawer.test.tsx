import React from 'react';
import { createMount, createShallow } from '@material-ui/core/test-utils';
import HelpDrawer from '../help-drawer';

const mount = createShallow();

test('Renders correctly', () => {
  const root = mount(
    <HelpDrawer handleCloseButton={() => jest.fn()} setShowHotkeyDialog={() => jest.fn()} />,
  );
  expect(root).toMatchSnapshot();
});
