import { Story, Meta } from '@storybook/react';
import React from 'react';
import { LogTable } from '../lib';

export default {
  title: 'Log table',
  component: LogTable,
  argTypes: {},
} as Meta;

const rows = [
  {
    message: 'test',
    level: 'ERROR',
    timestamp: 'Mon Jan  1 00:00:00 UTC 2001',
  },
  {
    message: `npm ERR! code ELIFECYCLE
    npm ERR! errno 1
    npm ERR! react-components@0.0.1 build: 'npm run lint && tsc --build'
    npm ERR! Exit status 1
    npm ERR! 
    npm ERR! Failed at the react-components@0.0.1 build script.
    npm ERR! This is probably not a problem with npm. There is likely additional logging output above.
    
    npm ERR! A complete log of this run can be found in:
    npm ERR!     /home/ekumen/.npm/_logs/2021-01-18T21_20_07_480Z-debug.log`,
    level: 'INFO',
    timestamp: 'Mon Jan  1 00:00:01 UTC 2001',
  },
  {
    message: 'Test warning',
    level: 'WARN',
    timestamp: 'Mon Jan  1 00:00:02 UTC 2001',
  },
];

for (let i = 0; i < 100; i++) {
  rows.push({
    message: 'Test' + i,
    level: 'WARN',
    timestamp: 'Mon Jan  1 00:00:02 UTC 2001',
  });
}

export const SimpleLogTable: Story = (args) => <LogTable rows={rows} {...args} />;
