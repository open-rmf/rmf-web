import React from 'react';
import { createMount, createShallow } from '@material-ui/core/test-utils';
import HelpDrawer from '../help-drawer';
import DrawerHeader from '../help-drawer-header';

const mount = createShallow();

test('Renders correctly', () => {
  const root = mount(
    <DrawerHeader handleCloseButton={() => jest.fn()} setShowHotkeyDialog={() => jest.fn()} />,
  );
  expect(root).toMatchSnapshot();
});
